#!/usr/bin/env python3
"""
CUSUM Detector Node — Core TFM Contribution (Fase 3).

Implements the sequential CUSUM algorithm to detect meaconing attacks by
comparing the GNSS-derived distance (D_GNSS) with the UWB-measured distance (D_UWB).    Algorithm:
    δ_k = D_UWB(k) - D_GNSS(k)
    S_k = max(0, S_{k-1} + δ_k - β)
    Alarm if S_k > τ

    Uses signed difference (not absolute) so noise cancels under H₀
    but the attack signal (D_GNSS collapse) consistently adds positive drift.

References:
    Bhatti & Humphreys (2017). Hostile Control of Ships via False GPS Signals.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger
import numpy as np


class CUSUMDetectorNode(Node):
    """
    Collaborative meaconing detector using CUSUM sequential analysis.

    Publishes:
        /system/cusum_value   — Current CUSUM statistic S_k
        /system/meaconing_alert — Boolean alert (True = meaconing detected)
    """

    def __init__(self):
        super().__init__('cusum_detector_node')

        # --- Parameters ---
        self.beta = self.declare_parameter('beta', 0.5).value
        self.tau = self.declare_parameter('tau', 2.0).value
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        self.startup_delay = self.declare_parameter('startup_delay', 5.0).value  # s

        # --- CUSUM state ---
        self.S_k = 0.0
        self.alert_active = False
        self.alert_start_time = None
        self.start_time = self.get_clock().now()

        # --- Latest values ---
        self.gnss_a = None  # robot1 spoofed GNSS
        self.gnss_b = None  # robot2 spoofed GNSS
        self.uwb_dist = None  # UWB distance

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/robot1/gnss_spoofed', self._cb_gnss_a, 10)
        self.create_subscription(
            PoseStamped, '/robot2/gnss_spoofed', self._cb_gnss_b, 10)
        self.create_subscription(
            Float64, '/robots/uwb_distance', self._cb_uwb, 10)

        # --- Publishers ---
        self.pub_alert = self.create_publisher(Bool, '/system/meaconing_alert', 10)
        self.pub_cusum = self.create_publisher(Float64, '/system/cusum_value', 10)
        self.pub_delta = self.create_publisher(Float64, '/system/delta_value', 10)

        # --- Service: reset CUSUM state ---
        self.srv_reset = self.create_service(Trigger, '/system/reset_cusum', self._reset_callback)

        # --- Timer for periodic CUSUM update (match data rate) ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'CUSUM Detector started (beta={self.beta:.2f}, tau={self.tau:.2f})')

    def _cb_gnss_a(self, msg: PoseStamped):
        self.gnss_a = msg.pose.position

    def _cb_gnss_b(self, msg: PoseStamped):
        self.gnss_b = msg.pose.position

    def _cb_uwb(self, msg: Float64):
        self.uwb_dist = msg.data

    def _timer_callback(self):
        if self.gnss_a is None or self.gnss_b is None or self.uwb_dist is None:
            return

        # Warmup: ignore first startup_delay seconds to let the data pipeline stabilize
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            return

        # Compute GNSS distance
        dx = self.gnss_a.x - self.gnss_b.x
        dy = self.gnss_a.y - self.gnss_b.y
        dz = self.gnss_a.z - self.gnss_b.z
        d_gnss = np.sqrt(dx * dx + dy * dy + dz * dz)

        # Signed innovation: D_UWB - D_GNSS
        #   H₀ (no attack): oscillates around 0 → S_k stays near 0
        #   H₁ (meaconing):  D_GNSS → 0, D_UWB ≈ true distance → signal > 0 consistently
        # Using signed difference (not absolute) lets noise cancel out under H₀
        delta = self.uwb_dist - d_gnss

        # CUSUM update (positive signal = meaconing detected)
        self.S_k = max(0.0, self.S_k + delta - self.beta)

        # Detection decision
        alert = self.S_k > self.tau

        # Publish CUSUM value
        cusum_msg = Float64()
        cusum_msg.data = self.S_k
        self.pub_cusum.publish(cusum_msg)

        # Publish delta
        delta_msg = Float64()
        delta_msg.data = delta
        self.pub_delta.publish(delta_msg)

        # Publish alert
        alert_msg = Bool()
        alert_msg.data = alert
        self.pub_alert.publish(alert_msg)

        # Log transitions
        if alert and not self.alert_active:
            self.alert_active = True
            self.alert_start_time = self.get_clock().now()
            self.get_logger().warn(
                f'🚨 MEACONING DETECTED! S_k = {self.S_k:.3f} > tau = {self.tau:.2f}')
        elif not alert and self.alert_active:
            self.alert_active = False
            if self.alert_start_time is not None:
                elapsed = (self.get_clock().now() - self.alert_start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'✅ Alert cleared after {elapsed:.2f} s (S_k = {self.S_k:.3f} ≤ tau)')

    def _reset_callback(self, request: Trigger.Request, response: Trigger.Response):
        """ROS 2 service to reset the CUSUM accumulator."""
        self.reset_cusum()
        response.success = True
        response.message = 'CUSUM state reset to zero'
        return response

    def get_cusum_state(self):
        """Return current CUSUM statistic (useful for external queries)."""
        return self.S_k

    def reset_cusum(self):
        """Reset the CUSUM accumulator (useful between experiments)."""
        self.S_k = 0.0
        self.alert_active = False
        self.alert_start_time = None
        self.start_time = self.get_clock().now()  # restart warmup clock
        self.get_logger().info('CUSUM state reset to zero')


def main(args=None):
    rclpy.init(args=args)
    node = CUSUMDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
