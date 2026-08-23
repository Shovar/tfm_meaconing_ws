#!/usr/bin/env python3
"""
Meaconing Injector Node

Subscribes to /robot1/gnss_clean and /robot2/gnss_clean, and when activated,
publishes GNSS positions where both robots are gradually dragged
toward a common fake target (single-antenna meaconing attack — legitimate
signal is received, artificially delayed, and retransmitted).

Activation is controlled via a ROS 2 service: /meaconing/set_active (std_srvs/SetBool).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import numpy as np


class MeaconingInjector(Node):
    """
    Injects a single-antenna 'drag-off' meaconing attack.

    When active: both robots' reported GNSS positions are pulled from their true
    positions toward a common fake target at `drift_velocity` (m/s). D_GNSS
    collapses from the true inter-robot distance down to ~noise at a rate set by
    drift_velocity, while D_UWB keeps reading the true distance. Slow drift ⇒ the
    detector's δ signal (and CUSUM) rises slowly; fast drift ⇒ it rises quickly.
    """

    def __init__(self):
        super().__init__('meaconing_injector')

        # --- Parameters ---
        self.drift_velocity = self.declare_parameter('drift_velocity', 0.2).value  # m/s (drag-off speed)
        self.activation_delay = self.declare_parameter('activation_delay', 30.0).value  # s
        self.sigma_gnss = self.declare_parameter('sigma_gnss', 2.0).value
        self.attack_type = self.declare_parameter('attack_type', 'single_antenna').value
        random_seed = self.declare_parameter('random_seed', 42).value
        np.random.seed(random_seed)

        # Validate attack_type
        valid_types = {'single_antenna', 'pattern'}
        if self.attack_type not in valid_types:
            self.get_logger().warn(
                f"Unknown attack_type='{self.attack_type}'. "
                f"Falling back to 'single_antenna'. Valid: {valid_types}"
            )
            self.attack_type = 'single_antenna'

        # --- State ---
        self.active = False
        self.start_time = None
        self.last_clean_a = None  # Last clean GNSS position for robot1
        self.last_clean_b = None  # Last clean GNSS position for robot2
        self.fake_origin = None   # Common fake target (spoofed point)
        self.p0_a = None          # robot1 true position at attack start
        self.p0_b = None          # robot2 true position at attack start
        self.drag_d0_a = None     # initial distance robot1 → fake target
        self.drag_d0_b = None     # initial distance robot2 → fake target

        # --- Subscribers (clean GNSS) ---
        self.create_subscription(
            PoseStamped, '/robot1/gnss_clean', self._cb_gnss_a, 10)
        self.create_subscription(
            PoseStamped, '/robot2/gnss_clean', self._cb_gnss_b, 10)

        # --- Publishers ---
        self.pub_a = self.create_publisher(PoseStamped, '/robot1/gnss_spoofed', 10)
        self.pub_b = self.create_publisher(PoseStamped, '/robot2/gnss_spoofed', 10)
        self.pub_status = self.create_publisher(Bool, '/meaconing/active', 10)

        # --- Service ---
        self.srv = self.create_service(SetBool, '/meaconing/set_active', self._srv_callback)

        # --- Timer (drift update + passthrough republish) ---
        # Republish at the same rate as the incoming GNSS data (update_rate).
        # The old 10 Hz timer quantized gnss_spoofed to 10 Hz, which correlated
        # consecutive delta samples and cut the detector's moving-average filter
        # effectiveness ~3× (effective window ~10 samples instead of 30), leaving
        # enough residual noise to fire false alarms in the no-attack baseline.
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        self.timer = self.create_timer(1.0 / self.update_rate, self._timer_callback)

        # --- Activation delay timer ---
        if self.activation_delay > 0:
            self.delay_timer = self.create_timer(
                self.activation_delay, self._auto_activate)
            self.get_logger().info(
                f'Attack will auto-activate in {self.activation_delay:.1f} s')
        else:
            self.delay_timer = None

        self.get_logger().info(
            f'Meaconing Injector started '
            f'(type={self.attack_type}, '
            f'drift_velocity={self.drift_velocity:.2f} m/s, '
            f'sigma_gnss={self.sigma_gnss:.2f} m)')

    def _auto_activate(self):
        """Auto-activate after the configured delay (one-shot)."""
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None
        self.active = True
        self.start_time = self.get_clock().now()
        self.get_logger().info('🛑 MEACONING ATTACK ACTIVATED (auto-delay)')

    def _cb_gnss_a(self, msg: PoseStamped):
        self.last_clean_a = msg

    def _cb_gnss_b(self, msg: PoseStamped):
        self.last_clean_b = msg

    def _srv_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Service to manually activate/deactivate the attack."""
        self.active = request.data
        if self.active:
            self.start_time = self.get_clock().now()
            self.get_logger().info('🛑 MEACONING ATTACK ACTIVATED (manual)')
            response.success = True
            response.message = 'Attack activated'
        else:
            self.fake_origin = None
            self.p0_a = None
            self.p0_b = None
            self.drag_d0_a = None
            self.drag_d0_b = None
            self.get_logger().info('✅ Meaconing attack deactivated')
            response.success = True
            response.message = 'Attack deactivated'
        return response

    def _init_fake_origin(self):
        """Snapshot the attack start state: true positions + the common fake target.

        Captures each robot's current reported (clean) position and the midpoint
        between them as the fake target. The reported positions are then dragged
        toward that target at `drift_velocity`, so D_GNSS collapses from the true
        inter-robot distance down to ~noise at a rate set by drift_velocity.
        """
        if self.fake_origin is None:
            if self.last_clean_a is not None and self.last_clean_b is not None:
                ax = self.last_clean_a.pose.position.x
                ay = self.last_clean_a.pose.position.y
                bx = self.last_clean_b.pose.position.x
                by = self.last_clean_b.pose.position.y
                self.p0_a = (ax, ay)
                self.p0_b = (bx, by)
                self.fake_origin = ((ax + bx) / 2.0, (ay + by) / 2.0)
            else:
                self.p0_a = (0.0, 0.0)
                self.p0_b = (0.0, 0.0)
                self.fake_origin = (0.0, 0.0)

            fx, fy = self.fake_origin
            self.drag_d0_a = max(np.hypot(self.p0_a[0] - fx, self.p0_a[1] - fy), 1e-3)
            self.drag_d0_b = max(np.hypot(self.p0_b[0] - fx, self.p0_b[1] - fy), 1e-3)
            self.get_logger().info(
                f'Drag-off attack: fake target at ({fx:.2f}, {fy:.2f}), '
                f'pulling robot1 over {self.drag_d0_a:.2f} m and robot2 over '
                f'{self.drag_d0_b:.2f} m at {self.drift_velocity:.2f} m/s')

    def _timer_callback(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # --- Publish status ---
        status = Bool()
        status.data = self.active
        self.pub_status.publish(status)

        if not self.active or self.start_time is None:
            # Passthrough: forward clean GNSS as-is
            if self.last_clean_a is not None:
                msg = self._copy_pose(self.last_clean_a)
                msg.header.stamp = now_msg
                self.pub_a.publish(msg)
            if self.last_clean_b is not None:
                msg = self._copy_pose(self.last_clean_b)
                msg.header.stamp = now_msg
                self.pub_b.publish(msg)
            return

        # --- Attack active: dispatch by attack type ---
        if self.attack_type == 'single_antenna':
            self._attack_single_antenna(now, now_msg)
        elif self.attack_type == 'pattern':
            # TODO: pattern-based meaconing
            self._attack_single_antenna(now, now_msg)

    def _attack_single_antenna(self, now, now_msg):
        """
        Single-antenna 'drag-off' meaconing: both robots' reported positions are
        pulled from their true positions toward a common fake target at
        `drift_velocity` (m/s), each keeping independent GNSS noise.

        D_GNSS collapses from the true inter-robot distance down to ~noise at a
        rate proportional to drift_velocity, so a slow drift makes the detector's
        δ signal (and the CUSUM) rise slowly, a fast drift makes it rise quickly.
        """
        self._init_fake_origin()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        fx, fy = self.fake_origin

        for p0, d0, pub in [
            (self.p0_a, self.drag_d0_a, self.pub_a),
            (self.p0_b, self.drag_d0_b, self.pub_b),
        ]:
            # Linear drag from the true position toward the fake target.
            progress = min(1.0, self.drift_velocity * elapsed / d0)
            x = p0[0] + progress * (fx - p0[0])
            y = p0[1] + progress * (fy - p0[1])

            msg = PoseStamped()
            msg.header.stamp = now_msg
            msg.header.frame_id = 'odom'
            msg.pose.position = Point(
                x=x + np.random.normal(0.0, self.sigma_gnss),
                y=y + np.random.normal(0.0, self.sigma_gnss),
                z=0.0,
            )
            msg.pose.orientation.w = 1.0
            pub.publish(msg)

    @staticmethod
    def _copy_pose(src: PoseStamped) -> PoseStamped:
        """Deep-value copy of a PoseStamped for safe passthrough."""
        msg = PoseStamped()
        msg.header.stamp = src.header.stamp
        msg.header.frame_id = src.header.frame_id
        msg.pose.position.x = src.pose.position.x
        msg.pose.position.y = src.pose.position.y
        msg.pose.position.z = src.pose.position.z
        msg.pose.orientation.x = src.pose.orientation.x
        msg.pose.orientation.y = src.pose.orientation.y
        msg.pose.orientation.z = src.pose.orientation.z
        msg.pose.orientation.w = src.pose.orientation.w
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MeaconingInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
