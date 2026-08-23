#!/usr/bin/env python3
"""
CUSUM Detector Node — Two-tailed sequential detector.

Compares the GNSS-derived distance (D_GNSS) with the UWB-measured distance
(D_UWB) to detect meaconing attacks.  A *meaconing* attack consists of
receiving legitimate GNSS signals, delaying them, and rebroadcasting them —
causing all victim receivers to report the same fake position.

Algorithm (two-tailed CUSUM — standard in statistical process control):

    δ_raw = D_UWB(k) - D_GNSS(k)                     (raw signed innovation)
    δ = δ_raw - median(δ_raw during startup_delay)   (baseline-corrected)
    δ̄ = moving_average(δ, filter_window)             (low-pass filter)

    S⁺_k = max(0, S⁺_{k-1} +  δ̄ - β)                 (positive drift  — D_GNSS collapses)
    S⁻_k = max(0, S⁻_{k-1} -  δ̄ - β)                 (negative drift  — D_GNSS inflates)

    ALARM if S⁺_k > τ or S⁻_k > τ  for at least alert_confirm_time.

Why two-tailed instead of absolute value:

  - The Euclidean GNSS range has a non-zero noise bias even when the position
    noise is zero-mean. The attack-free startup median removes that operating-
    point bias before the signed two-tailed CUSUM runs.
  - Each branch then uses signed δ with E[δ] ≈ 0 under H₀ → the max(0,·)
    truncation keeps both accumulators at zero during normal operation.
  - Absolute value would bias the detector: E[|δ|] ≈ 1.14 m under H₀,
    requiring β > 1.14 to avoid false alarms and wasting sensitivity.
  - Two-tailed detects meaconing (D_GNSS → 0, δ > 0) AND pattern-based
    attacks where D_GNSS is artificially inflated (δ < 0) — both without
    the folded-normal noise-floor penalty.

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


def update_cusum(s_plus, s_minus, delta_filtered, beta):
    """Apply one two-tailed CUSUM update and return both accumulators."""
    next_plus = max(0.0, s_plus + delta_filtered - beta)
    next_minus = max(0.0, s_minus - delta_filtered - beta)
    return next_plus, next_minus


class CUSUMDetectorNode(Node):
    """
    Two-tailed collaborative meaconing detector using CUSUM.

    Maintains two independent accumulators:
      - S_plus  monitors δ > 0  (D_GNSS collapses, single-antenna meaconing)
      - S_minus monitors δ < 0  (D_GNSS inflates, pattern-based attack)

    Publishes:
        /system/cusum_value      — max(S_plus, S_minus)
        /system/cusum_plus       — S_plus accumulator
        /system/cusum_minus      — S_minus accumulator
        /system/meaconing_alert  — Boolean alert (True = meaconing detected)
    """

    def __init__(self):
        super().__init__('cusum_detector_node')

        # --- Parameters ---
        self.beta = self.declare_parameter('beta', 0.5).value
        self.tau = self.declare_parameter('tau', 3.0).value
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        self.startup_delay = self.declare_parameter('startup_delay', 10.0).value
        self.filter_window = self.declare_parameter('filter_window', 30).value
        self.alert_confirm_time = self.declare_parameter('alert_confirm_time', 2.0).value

        # --- Two-tailed CUSUM state ---
        self.S_plus = 0.0
        self.S_minus = 0.0
        self.alert_active = False
        self.candidate_since = None       # when S_k first exceeded τ
        self.candidate_branch = None      # 'plus' or 'minus'
        self.first_data_time = None
        self.baseline_delta = 0.0
        self.baseline_ready = False
        self.calibration_deltas = deque(
            maxlen=max(1, int(round(self.startup_delay * self.update_rate))))
        self.attack_active = False

        # Moving window of recent, baseline-corrected deltas for the low-pass filter
        self.delta_history = deque(maxlen=max(1, int(self.filter_window)))

        # --- Latest values ---
        self.gnss_a = None    # robot1 GNSS (from meaconing injector)
        self.gnss_b = None    # robot2 GNSS (from meaconing injector)
        self.uwb_dist = None  # UWB distance

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/robot1/gnss_spoofed', self._cb_gnss_a, 10)
        self.create_subscription(
            PoseStamped, '/robot2/gnss_spoofed', self._cb_gnss_b, 10)
        self.create_subscription(
            Float64, '/robots/uwb_distance', self._cb_uwb, 10)
        self.create_subscription(
            Bool, '/meaconing/active', self._cb_attack_status, 10)

        # --- Publishers ---
        self.pub_alert = self.create_publisher(Bool, '/system/meaconing_alert', 10)
        self.pub_cusum = self.create_publisher(Float64, '/system/cusum_value', 10)
        self.pub_plus = self.create_publisher(Float64, '/system/cusum_plus', 10)
        self.pub_minus = self.create_publisher(Float64, '/system/cusum_minus', 10)
        self.pub_delta = self.create_publisher(Float64, '/system/delta_value', 10)
        self.pub_delta_raw = self.create_publisher(Float64, '/system/delta_raw', 10)

        # --- Service ---
        self.srv_reset = self.create_service(
            Trigger, '/system/reset_cusum', self._reset_callback)

        # --- Timer ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'CUSUM Detector (two-tailed) started '
            f'(beta={self.beta:.2f}, tau={self.tau:.2f})')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _cb_gnss_a(self, msg: PoseStamped):
        self.gnss_a = msg.pose.position

    def _cb_gnss_b(self, msg: PoseStamped):
        self.gnss_b = msg.pose.position

    def _cb_uwb(self, msg: Float64):
        self.uwb_dist = msg.data

    def _cb_attack_status(self, msg: Bool):
        self.attack_active = msg.data

    # ------------------------------------------------------------------ #
    #  Main detection loop                                                #
    # ------------------------------------------------------------------ #
    def _timer_callback(self):
        if self.gnss_a is None or self.gnss_b is None or self.uwb_dist is None:
            return

        # Start the clock when the first complete measurement arrives, not
        # when the node starts.
        now = self.get_clock().now()
        if self.first_data_time is None:
            self.first_data_time = now
        elapsed = (now - self.first_data_time).nanoseconds / 1e9

        # --- GNSS distance and raw innovation ---------------------------
        dx = self.gnss_a.x - self.gnss_b.x
        dy = self.gnss_a.y - self.gnss_b.y
        dz = self.gnss_a.z - self.gnss_b.z
        d_gnss = np.sqrt(dx * dx + dy * dy + dz * dz)
        delta_raw = self.uwb_dist - d_gnss

        # GNSS range noise is not zero-mean after taking the Euclidean norm.
        # Estimate that operating-point bias during the attack-free warmup so
        # normal sensor geometry does not drive the negative CUSUM branch.
        if elapsed < self.startup_delay:
            if not self.attack_active:
                self.calibration_deltas.append(delta_raw)
            return

        if not self.baseline_ready:
            if self.calibration_deltas:
                self.baseline_delta = float(np.median(self.calibration_deltas))
            self.baseline_ready = True
            self.get_logger().info(
                f'CUSUM baseline calibrated (delta0={self.baseline_delta:.3f} m, '
                f'samples={len(self.calibration_deltas)})')

        # --- Baseline-corrected signed innovation -----------------------
        delta = delta_raw - self.baseline_delta

        # --- Low-pass filter ---
        self.delta_history.append(delta)
        delta_filtered = float(np.mean(self.delta_history))

        # --- Two-tailed CUSUM update ---
        # Baseline calibration removes the normal GNSS-range operating-point
        # bias; the signed branches then detect deviations in either direction.
        self.S_plus, self.S_minus = update_cusum(
            self.S_plus, self.S_minus, delta_filtered, self.beta)

        # Effective CUSUM value is the max of both branches
        S_effective = max(self.S_plus, self.S_minus)

        # --- Detection decision ---
        # Alarm fires if EITHER branch exceeds τ for alert_confirm_time.
        if S_effective > self.tau:
            if self.candidate_since is None:
                self.candidate_since = now
                self.candidate_branch = 'plus' if self.S_plus > self.S_minus else 'minus'
            confirmed = (now - self.candidate_since).nanoseconds / 1e9 >= self.alert_confirm_time
        else:
            self.candidate_since = None
            self.candidate_branch = None
            confirmed = False

        # --- Publish ---
        cusum_msg = Float64()
        cusum_msg.data = S_effective
        self.pub_cusum.publish(cusum_msg)

        plus_msg = Float64()
        plus_msg.data = self.S_plus
        self.pub_plus.publish(plus_msg)

        minus_msg = Float64()
        minus_msg.data = self.S_minus
        self.pub_minus.publish(minus_msg)

        delta_raw_msg = Float64()
        delta_raw_msg.data = delta_raw
        self.pub_delta_raw.publish(delta_raw_msg)

        delta_msg = Float64()
        delta_msg.data = delta_filtered
        self.pub_delta.publish(delta_msg)

        alert_msg = Bool()
        alert_msg.data = confirmed
        self.pub_alert.publish(alert_msg)

        # --- Log transitions ---
        if confirmed and not self.alert_active:
            self.alert_active = True
            side = self.candidate_branch if self.candidate_branch else 'plus'
            self.get_logger().warn(
                f'🚨 MEACONING DETECTED (S_{side} = '
                f'{self.S_plus if side == "plus" else self.S_minus:.3f} '
                f'> tau = {self.tau:.2f})')
        elif not confirmed and self.alert_active:
            self.alert_active = False
            self.get_logger().info(
                f'✅ Alert cleared (S_plus = {self.S_plus:.3f}, '
                f'S_minus = {self.S_minus:.3f} ≤ tau)')

    # ------------------------------------------------------------------ #
    #  Service: reset                                                     #
    # ------------------------------------------------------------------ #
    def _reset_callback(self, request: Trigger.Request,
                        response: Trigger.Response):
        self.reset_cusum()
        response.success = True
        response.message = 'CUSUM state reset to zero'
        return response

    def get_cusum_state(self):
        return max(self.S_plus, self.S_minus)

    def reset_cusum(self):
        self.S_plus = 0.0
        self.S_minus = 0.0
        self.alert_active = False
        self.candidate_since = None
        self.candidate_branch = None
        self.first_data_time = None
        self.baseline_delta = 0.0
        self.baseline_ready = False
        self.calibration_deltas.clear()
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
