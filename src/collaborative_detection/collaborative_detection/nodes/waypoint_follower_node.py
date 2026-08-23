#!/usr/bin/env python3
"""
Waypoint Follower Node (E5 experiment) — Pure Pursuit controller.

Both robots navigate multi-waypoint routes via GNSS as position source.
Routes are offset by ~2 m in Y so inter-robot D_UWB stays ≈ 2 m throughout
the experiment → the CUSUM detector sees a strong δ ≈ 2 m under attack.

  - Robot1 uses /robot1/gnss_spoofed (meaconing output) → drifts under attack.
  - Robot2 uses /robot2/gnss_clean (not meaconed) → always on its true route.

  - Each robot follows its OWN waypoint list (waypoints1_* / waypoints2_*).
  - Controller: Pure Pursuit with fixed lookahead distance.
  - Heading (yaw) for both comes from odometry.

Legacy mode (publish_robot2=True): robot2 runs open-loop circle (E0-E4 mode).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np


# ====================================================================== #
#  Pure Pursuit helpers                                                  #
# ====================================================================== #

def _dist_to_segment(px, py, ax, ay, bx, by):
    """Signed distance + projection parameter t from point P to segment AB.

    Returns (t, d, closest_x, closest_y) where t in [0,1] is the
    projection parameter along AB, d is the normal distance, and
    (closest_x, closest_y) is the closest point on the segment.
    """
    abx, aby = bx - ax, by - ay
    seg2 = abx * abx + aby * aby
    if seg2 < 1e-12:
        return 0.0, np.hypot(px - ax, py - ay), ax, ay
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / seg2))
    cx = ax + t * abx
    cy = ay + t * aby
    d = np.hypot(px - cx, py - cy)
    # Sign: positive if P is to the LEFT of AB (2D cross product > 0)
    side = abx * (py - ay) - aby * (px - ax)
    return t, d if side >= 0 else -d, cx, cy


def _lookahead_point(px, py, waypoints, wp_idx, lookahead):
    """Find the point `lookahead` metres ahead of the robot along the path.

    Walks forward from the robot's projection onto the current segment,
    consuming segments until the accumulated arc-length reaches `lookahead`.

    Returns (tx, ty, new_idx) — the target point and the index of the
    segment that contains it.
    """
    n = len(waypoints)
    # Walk forward to find the segment whose cumulative length reaches lookahead
    accumulated = 0.0
    idx = wp_idx

    for _ in range(n):
        a = waypoints[idx % n]
        b = waypoints[(idx + 1) % n]
        seg_len = np.hypot(b[0] - a[0], b[1] - a[1])

        # Project robot onto this segment
        _, _, cx, cy = _dist_to_segment(px, py, a[0], a[1], b[0], b[1])
        # Distance from projection to end of segment
        d_to_end = np.hypot(b[0] - cx, b[1] - cy)
        needed = lookahead - accumulated

        if needed <= d_to_end:
            # Lookahead point is on this segment
            t = needed / seg_len if seg_len > 1e-9 else 0.0
            tx = cx + t * (b[0] - a[0])
            ty = cy + t * (b[1] - a[1])
            return tx, ty, idx
        else:
            accumulated += d_to_end
            idx = (idx + 1) % n

    # Fallback: use the current waypoint itself
    w = waypoints[wp_idx]
    return w[0], w[1], wp_idx


def _step_waypoint(px, py, waypoints, idx, arrival_dist):
    """Move to next waypoint if robot is within arrival_dist of idx+1.

    Returns the (possibly advanced) index.
    """
    n = len(waypoints)
    nxt = (idx + 1) % n
    wx, wy = waypoints[nxt]
    if np.hypot(px - wx, py - wy) < arrival_dist:
        return nxt
    return idx


def pure_pursuit(pos_x, pos_y, yaw, waypoints, wp_idx,
                 linear_speed, lookahead, wheelbase, arrival_dist):
    """
    Compute (linear_x, angular_z, new_wp_idx) via Pure Pursuit.

    Parameters
    ----------
    pos_x, pos_y : float
        Current robot position (world frame).
    yaw : float
        Current heading angle (rad).
    waypoints : list of (x, y)
        Ordered list of waypoints.
    wp_idx : int
        Index of the current *target* waypoint on the path.
    linear_speed : float
        Desired linear velocity (m/s).
    lookahead : float
        Pure-pursuit lookahead distance (m).
    wheelbase : float
        Not used in the kinematic (curvature) formulation; kept for API.
    arrival_dist : float
        Distance threshold for advancing to next waypoint.

    Returns
    -------
    (v, w, new_idx)
    """
    if pos_x is None or pos_y is None or yaw is None:
        return 0.0, 0.0, wp_idx

    # Advance waypoint if close enough
    wp_idx = _step_waypoint(pos_x, pos_y, waypoints, wp_idx, arrival_dist)

    # Compute lookahead target
    tx, ty, _ = _lookahead_point(pos_x, pos_y, waypoints, wp_idx, lookahead)

    # Vector from robot to target (world frame)
    dx = tx - pos_x
    dy = ty - pos_y

    # Transform to robot frame
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    lx = dx * cos_yaw + dy * sin_yaw   # longitudinal (forward)
    ly = -dx * sin_yaw + dy * cos_yaw  # lateral

    # Curvature: κ = 2 * ly / (lx² + ly²)  (classic pure pursuit formula)
    ld2 = lx * lx + ly * ly
    if ld2 < 1e-9:
        return 0.0, 0.0, wp_idx

    curvature = 2.0 * ly / ld2
    w = float(np.clip(linear_speed * curvature, -1.5, 1.5))
    # Reduce speed on sharp turns
    v = float(np.clip(linear_speed / (1.0 + abs(curvature) * 0.5), 0.0, linear_speed))
    return v, w, wp_idx


# ====================================================================== #
#  Node                                                                  #
# ====================================================================== #

class WaypointFollowerNode(Node):
    """Pure Pursuit waypoint follower for two robots."""

    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --- Route: robot1 ---
        wp1_x = list(self.declare_parameter('waypoints1_x', [5.0, 5.0, 0.0]).value)
        wp1_y = list(self.declare_parameter('waypoints1_y', [0.0, 5.0, 5.0]).value)
        self.route1 = list(zip(wp1_x, wp1_y))

        # --- Route: robot2 (offset ~2 m from robot1) ---
        wp2_x = list(self.declare_parameter('waypoints2_x', [5.0, 5.0, 0.0]).value)
        wp2_y = list(self.declare_parameter('waypoints2_y', [2.0, 7.0, 7.0]).value)
        self.route2 = list(zip(wp2_x, wp2_y))

        # --- Pure Pursuit params ---
        self.linear_speed   = self.declare_parameter('linear_speed', 0.2).value
        self.lookahead      = self.declare_parameter('lookahead', 0.8).value
        self.arrival_dist   = self.declare_parameter('waypoint_arrival_dist', 0.5).value
        self.update_rate    = self.declare_parameter('update_rate', 20.0).value

        # --- State ---
        self.wp_idx1 = 0
        self.wp_idx2 = 0

        self.gnss_x = None       # robot1: /robot1/gnss_spoofed
        self.gnss_y = None
        self.yaw1 = None

        self.r2_gnss_x = None    # robot2 GNSS (source depends on r2_gnss_source)
        self.r2_gnss_y = None
        self.yaw2 = None

        # --- Legacy robot2 circle mode ---
        self.publish_robot2 = self.declare_parameter('publish_robot2', False).value
        self.r2_linear = self.declare_parameter('robot2_linear_vel', 0.12).value
        self.r2_angular = self.declare_parameter('robot2_angular_vel', 0.25).value

        # --- E5/E6 mode ---
        self.r2_waypoint = self.declare_parameter('robot2_waypoint_mode', False).value
        # 'clean' = robot2 uses gnss_clean (E5: only robot1 attacked)
        # 'spoofed' = robot2 uses gnss_spoofed (E6: both robots attacked)
        self.r2_gnss_source = self.declare_parameter('r2_gnss_source', 'clean').value

        # --- Subscribers ---
        self.create_subscription(PoseStamped, '/robot1/gnss_spoofed',
                                 self._cb_gnss, 10)
        self.create_subscription(Odometry, '/robot1/odom',
                                 self._cb_odom_r1, 10)
        if self.r2_waypoint:
            r2_topic = ('/robot2/gnss_spoofed' if self.r2_gnss_source == 'spoofed'
                        else '/robot2/gnss_clean')
            self.get_logger().info(
                f'Robot2 navigates via {r2_topic} (r2_gnss_source={self.r2_gnss_source})')
            self.create_subscription(PoseStamped, r2_topic,
                                     self._cb_gnss_r2, 10)
            self.create_subscription(Odometry, '/robot2/odom',
                                     self._cb_odom_r2, 10)

        # --- Publishers ---
        self.pub_cmd1 = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        r2_needs = self.publish_robot2 or self.r2_waypoint
        self.pub_cmd2 = (self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)
                         if r2_needs else None)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.update_rate, self._timer_callback)

        # --- Log ---
        r1s = ' → '.join(f'({x:.0f},{y:.0f})' for x, y in self.route1)
        r2s = ' → '.join(f'({x:.0f},{y:.0f})' for x, y in self.route2)
        self.get_logger().info(
            f'Waypoint Follower (Pure Pursuit) | '
            f'lookahead={self.lookahead:.1f}m | v={self.linear_speed:.1f}m/s')
        self.get_logger().info(f'  Robot1 route: {r1s}')
        if self.r2_waypoint:
            self.get_logger().info(f'  Robot2 route: {r2s}')
        elif self.publish_robot2:
            self.get_logger().info('  Robot2: open-loop circle')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _cb_gnss(self, msg: PoseStamped):
        self.gnss_x = msg.pose.position.x
        self.gnss_y = msg.pose.position.y

    def _cb_gnss_r2(self, msg: PoseStamped):
        self.r2_gnss_x = msg.pose.position.x
        self.r2_gnss_y = msg.pose.position.y

    def _cb_odom_r1(self, msg: Odometry):
        self.yaw1 = self._yaw_from_quat(msg.pose.pose.orientation)

    def _cb_odom_r2(self, msg: Odometry):
        self.yaw2 = self._yaw_from_quat(msg.pose.pose.orientation)

    @staticmethod
    def _yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny, cosy))

    @staticmethod
    def _make_twist(linear, angular):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        return msg

    def _timer_callback(self):
        now = self.get_clock().now().to_msg()

        # --- Robot1 (meaconed GNSS → drifts under attack) ---
        v1, w1, self.wp_idx1 = pure_pursuit(
            self.gnss_x, self.gnss_y, self.yaw1,
            self.route1, self.wp_idx1,
            self.linear_speed, self.lookahead, 0.0, self.arrival_dist,
        )
        cmd = self._make_twist(v1, w1)
        cmd.header.stamp = now
        self.pub_cmd1.publish(cmd)

        # --- Robot2 ---
        if self.publish_robot2:
            cmd2 = self._make_twist(self.r2_linear, self.r2_angular)
            cmd2.header.stamp = now
            self.pub_cmd2.publish(cmd2)
        elif self.r2_waypoint:
            v2, w2, self.wp_idx2 = pure_pursuit(
                self.r2_gnss_x, self.r2_gnss_y, self.yaw2,
                self.route2, self.wp_idx2,
                self.linear_speed, self.lookahead, 0.0, self.arrival_dist,
            )
            cmd2 = self._make_twist(v2, w2)
            cmd2.header.stamp = now
            self.pub_cmd2.publish(cmd2)


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