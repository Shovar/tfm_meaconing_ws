#!/usr/bin/env python3
"""
CUSUM Detector Node

Implements the sequential CUSUM algorithm to detect meaconing attacks by
comparing the GNSS-derived distance (D_GNSS) with the UWB-measured distance (D_UWB).

    Algorithm:
        δ_k = D_UWB(k) - D_GNSS(k)
        δ̄_k = moving_average(δ, filter_window)   # low-pass to suppress GNSS noise
        S_k = max(0, S_{k-1} + δ̄_k - β)
        Alarm if S_k > τ for at least alert_confirm_time (confirmation window)

    Uses signed difference (not absolute) so noise cancels under H₀
    but the attack signal (D_GNSS collapse) consistently adds positive drift.

    The raw δ is dominated by GNSS noise (std ≈ sqrt(σ_uwb² + 2·σ_gnss²) ≈ 1.4 m),
    comparable to the ~1.2 m attack signal. A moving average over filter_window
    samples (≈1 s at 30 Hz) reduces that noise ~5.5× so the baseline (no attack)
    stays clean and does not fire false alarms.

References:
    Bhatti & Humphreys (2017). Hostile Control of Ships via False GPS Signals.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger
from collections import deque
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
        self.tau = self.declare_parameter('tau', 3.0).value
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        self.startup_delay = self.declare_parameter('startup_delay', 10.0).value  # s
        self.filter_window = self.declare_parameter('filter_window', 30).value  # samples
        self.alert_confirm_time = self.declare_parameter('alert_confirm_time', 2.0).value  # s

        # --- CUSUM state ---
        self.S_k = 0.0
        self.alert_active = False
        self.candidate_since = None  # when S_k first exceeded tau (pending confirmation)
        self.first_data_time = None  # set when the first complete measurement arrives
        # Moving window of recent deltas for the low-pass filter (see module docstring).
        self.delta_history = deque(maxlen=max(1, int(self.filter_window)))

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
        self.pub_delta_raw = self.create_publisher(Float64, '/system/delta_raw', 10)

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

        # Warmup: start the clock when the FIRST complete measurement arrives,
        # not when the node starts. The data pipeline (bridge → odom → GNSS/UWB)
        # takes ~10s to produce its first samples, so a node-start clock expires
        # before any data flows and the detector accumulates the startup
        # transient (δ spikes to ~1.8 m → false S_k jump → false alarm).
        now = self.get_clock().now()
        if self.first_data_time is None:
            self.first_data_time = now
        elapsed = (now - self.first_data_time).nanoseconds / 1e9
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

        # Low-pass filter on delta (see module docstring). Averaging over the
        # window suppresses the GNSS noise that otherwise dwarfs the attack signal.
        self.delta_history.append(delta)
        delta_filtered = float(np.mean(self.delta_history))

        # CUSUM update (positive signal = meaconing detected)
        self.S_k = max(0.0, self.S_k + delta_filtered - self.beta)

        # Detection decision with a confirmation window: S_k must stay above tau
        # for alert_confirm_time seconds before the alarm fires. This rejects
        # short noise transients (a baseline spike that crosses tau for ~1 s and
        # then recovers) while the sustained attack signal keeps S_k above tau.
        if self.S_k > self.tau:
            if self.candidate_since is None:
                self.candidate_since = now
            confirmed = (now - self.candidate_since).nanoseconds / 1e9 >= self.alert_confirm_time
        else:
            self.candidate_since = None
            confirmed = False

        # Publish CUSUM value
        cusum_msg = Float64()
        cusum_msg.data = self.S_k
        self.pub_cusum.publish(cusum_msg)

        # Publish delta (raw for diagnostics; filtered is what the CUSUM uses)
        delta_raw_msg = Float64()
        delta_raw_msg.data = delta
        self.pub_delta_raw.publish(delta_raw_msg)

        delta_msg = Float64()
        delta_msg.data = delta_filtered
        self.pub_delta.publish(delta_msg)

        # Publish alert (confirmed only)
        alert_msg = Bool()
        alert_msg.data = confirmed
        self.pub_alert.publish(alert_msg)

        # Log transitions
        if confirmed and not self.alert_active:
            self.alert_active = True
            self.get_logger().warn(
                f'🚨 MEACONING DETECTED! S_k = {self.S_k:.3f} > tau = {self.tau:.2f}')
        elif not confirmed and self.alert_active:
            self.alert_active = False
            self.get_logger().info(f'✅ Alert cleared (S_k = {self.S_k:.3f} ≤ tau)')

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
        self.candidate_since = None
        self.first_data_time = None  # restart warmup clock on next data arrival
        self.delta_history.clear()
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
