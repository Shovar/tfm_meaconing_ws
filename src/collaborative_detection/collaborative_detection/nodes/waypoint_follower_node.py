#!/usr/bin/env python3
"""
Waypoint Follower Node (E5 experiment)

Simple proportional controller that steers one or two robots toward a fixed
waypoint. The key design choice:

  - Robot1 uses /robot1/gnss_spoofed (meaconing-injector output) as its
    position input → under attack, the controller steers robot1 AWAY from
    the true waypoint.  Physical drift is the E5 metric.

  - Robot2 (when robot2_waypoint_mode=True) uses /robot2/odom (ground truth,
    unaffected by meaconing) as its position input → robot2 always moves
    straight toward the waypoint.  The robots stay ~constant distance apart
    (side-by-side spawn), so inter-robot D_UWB ≈ D_GNSS and delta stays
    clean — no false positives during the baseline period.

Legacy mode (robot_mover_node replacement):
  - publish_robot2=True → robot2 runs open-loop circular motion (E0–E4 mode)

Design (intentionally basic):
  heading_error = atan2(waypoint_y - pos_y, waypoint_x - pos_x) - yaw
  ω = angular_gain * heading_error   (proportional only)
  v = min(linear_speed, linear_gain * distance_to_waypoint)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np


class WaypointFollowerNode(Node):
    """Steers robot(s) toward a waypoint."""

    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --- Waypoint parameters ---
        self.waypoint_x = self.declare_parameter('waypoint_x', 5.0).value
        self.waypoint_y = self.declare_parameter('waypoint_y', 0.0).value

        # --- Controller gains ---
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.linear_gain = self.declare_parameter('linear_gain', 0.3).value
        self.angular_gain = self.declare_parameter('angular_gain', 1.0).value
        self.update_rate = self.declare_parameter('update_rate', 20.0).value

        # --- Robot2 legacy: open-loop circle (publish_robot2=True) ---
        self.publish_robot2 = self.declare_parameter('publish_robot2', False).value
        self.r2_linear = self.declare_parameter('robot2_linear_vel', 0.12).value
        self.r2_angular = self.declare_parameter('robot2_angular_vel', 0.25).value

        # --- Robot2 new: waypoint following via odometry (E5 mode) ---
        self.r2_waypoint = self.declare_parameter('robot2_waypoint_mode', False).value

        # --- State: robot1 (GNSS-spoofed position + odom yaw) ---
        self.gnss_x = None       # from /robot1/gnss_spoofed
        self.gnss_y = None
        self.yaw = None          # from /robot1/odom

        # --- State: robot2 (odometry ground truth) ---
        self.r2_odom_x = None    # from /robot2/odom
        self.r2_odom_y = None
        self.r2_yaw = None

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/robot1/gnss_spoofed', self._cb_gnss, 10)
        self.create_subscription(
            Odometry, '/robot1/odom', self._cb_odom_r1, 10)

        if self.r2_waypoint:
            self.create_subscription(
                Odometry, '/robot2/odom', self._cb_odom_r2, 10)

        # --- Publishers ---
        self.pub_cmd = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)

        # Robot2 cmd_vel publisher: used for both circle mode and waypoint mode
        self.r2_needs_cmd = self.publish_robot2 or self.r2_waypoint
        self.pub_cmd_r2 = self.create_publisher(
            TwistStamped, '/robot2/cmd_vel', 10) if self.r2_needs_cmd else None

        # --- Timer ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        mode_desc = 'gnss_spoofed → waypoint'
        if self.r2_waypoint:
            mode_desc += ' | robot2: odom → waypoint'
        elif self.publish_robot2:
            mode_desc += ' | robot2: open-loop circle'
        self.get_logger().info(
            f'Waypoint Follower started | {mode_desc} | '
            f'waypoint=({self.waypoint_x:.1f}, {self.waypoint_y:.1f}) | '
            f'v_max={self.linear_speed:.2f}, ω_gain={self.angular_gain:.2f}'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _cb_gnss(self, msg: PoseStamped):
        """Store the (potentially spoofed) GNSS position for robot1."""
        self.gnss_x = msg.pose.position.x
        self.gnss_y = msg.pose.position.y

    def _cb_odom_r1(self, msg: Odometry):
        self.yaw = self._yaw_from_quat(msg.pose.pose.orientation)

    def _cb_odom_r2(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.r2_odom_x = pos.x
        self.r2_odom_y = pos.y
        self.r2_yaw = self._yaw_from_quat(msg.pose.pose.orientation)

    @staticmethod
    def _yaw_from_quat(q):
        """Extract yaw (Z rotation) from a quaternion. Returns radians."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny, cosy))

    # ------------------------------------------------------------------ #
    #  Control logic                                                      #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _control(pos_x: float, pos_y: float, yaw: float,
                 way_x: float, way_y: float,
                 v_max: float, v_gain: float, w_gain: float):
        """
        Compute (linear, angular) velocities for a robot given its current
        (x, y, yaw) and a target waypoint.

        Returns (linear_x, angular_z), both floats.  Returns (0.0, 0.0) if
        any input is None or the robot is within 0.05 m of the waypoint.
        """
        if pos_x is None or pos_y is None or yaw is None:
            return 0.0, 0.0

        dx = way_x - pos_x
        dy = way_y - pos_y
        dist = np.hypot(dx, dy)

        # Stop when close enough to avoid jitter
        if dist < 0.05:
            return 0.0, 0.0

        bearing = np.arctan2(dy, dx)
        error = bearing - yaw
        error = np.arctan2(np.sin(error), np.cos(error))   # wrap to [-π, π]

        w = w_gain * error
        v = min(v_max, v_gain * dist)

        w = float(np.clip(w, -1.5, 1.5))
        v = float(np.clip(v, 0.0, v_max))
        return v, w

    def _make_twist(self, linear: float, angular: float) -> TwistStamped:
        """Build a TwistStamped with the given velocities."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        return msg

    def _timer_callback(self):
        # --- Robot1: GNSS-spoofed waypoint following ---
        v1, w1 = self._control(
            self.gnss_x, self.gnss_y, self.yaw,
            self.waypoint_x, self.waypoint_y,
            self.linear_speed, self.linear_gain, self.angular_gain,
        )
        if v1 != 0.0 or w1 != 0.0:
            self.pub_cmd.publish(self._make_twist(v1, w1))

        # --- Robot2 dispatch ---
        if self.publish_robot2:
            # Legacy: open-loop circle
            self.pub_cmd_r2.publish(
                self._make_twist(self.r2_linear, self.r2_angular))
        elif self.r2_waypoint:
            v2, w2 = self._control(
                self.r2_odom_x, self.r2_odom_y, self.r2_yaw,
                self.waypoint_x, self.waypoint_y,
                self.linear_speed, self.linear_gain, self.angular_gain,
            )
            if v2 != 0.0 or w2 != 0.0:
                self.pub_cmd_r2.publish(self._make_twist(v2, w2))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()