#!/usr/bin/env python3
"""
GNSS Simulator Node — Fase 1 del TFM.

Subscribes to /robot1/odom and /robot2/odom (ground truth from Gazebo),
converts from local odom frame to global world frame using spawn offsets,
adds Gaussian noise, and publishes simulated GNSS positions.

Noise model: Gaussian with sigma from params.yaml (default 2.0 m — GPS civil).

The DiffDrive plugin publishes odometry in a per-robot local frame that starts
at (0,0) regardless of world spawn position. We add the known spawn offset to
obtain world-frame coordinates so D_GNSS and D_UWB are comparable.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
import numpy as np


class GNSSSimNode(Node):
    """Simulates GNSS readings by adding Gaussian noise to Gazebo ground truth."""

    def __init__(self):
        super().__init__('gnss_sim_node')

        # --- Parameters ---
        self.sigma_gnss = self.declare_parameter('sigma_gnss', 2.0).value
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        random_seed = self.declare_parameter('random_seed', 42).value
        np.random.seed(random_seed)

        # --- State: store latest odom positions ---
        self.pos_a = None  # robot1
        self.pos_b = None  # robot2
        self.ts_a = None
        self.ts_b = None

        # --- Subscribers ---
        self.create_subscription(
            Odometry, '/robot1/odom', self._cb_odom_a, 10)
        self.create_subscription(
            Odometry, '/robot2/odom', self._cb_odom_b, 10)

        # --- Spawn offsets (odom → world conversion) ---
        self.spawn_a_x = self.declare_parameter('robot1.x', 0.0).value
        self.spawn_a_y = self.declare_parameter('robot1.y', 0.0).value
        self.spawn_b_x = self.declare_parameter('robot2.x', 3.0).value
        self.spawn_b_y = self.declare_parameter('robot2.y', 0.0).value

        # --- Publishers ---
        self.pub_a = self.create_publisher(PoseStamped, '/robot1/gnss_clean', 10)
        self.pub_b = self.create_publisher(PoseStamped, '/robot2/gnss_clean', 10)

        # --- Timer for periodic publishing ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'GNSS Sim Node started (sigma_gnss={self.sigma_gnss:.2f} m, '
            f'rate={self.update_rate:.1f} Hz)')

    def _cb_odom_a(self, msg: Odometry):
        self.pos_a = msg.pose.pose.position
        self.ts_a = msg.header.stamp

    def _cb_odom_b(self, msg: Odometry):
        self.pos_b = msg.pose.pose.position
        self.ts_b = msg.header.stamp

    def _add_noise(self, x, y):
        """Return a new Point with Gaussian noise added to x, y."""
        return Point(
            x=x + np.random.normal(0.0, self.sigma_gnss),
            y=y + np.random.normal(0.0, self.sigma_gnss),
            z=0.0,
        )

    def _timer_callback(self):
        now = self.get_clock().now().to_msg()

        if self.pos_a is not None:
            # Convert odom frame → world frame
            wx = self.pos_a.x + self.spawn_a_x
            wy = self.pos_a.y + self.spawn_a_y
            world_pos = self._add_noise(wx, wy)

            msg = PoseStamped()
            msg.header.stamp = self.ts_a if self.ts_a else now
            msg.header.frame_id = 'world'
            msg.pose.position = world_pos
            msg.pose.orientation.w = 1.0
            self.pub_a.publish(msg)

        if self.pos_b is not None:
            wx = self.pos_b.x + self.spawn_b_x
            wy = self.pos_b.y + self.spawn_b_y
            world_pos = self._add_noise(wx, wy)

            msg = PoseStamped()
            msg.header.stamp = self.ts_b if self.ts_b else now
            msg.header.frame_id = 'world'
            msg.pose.position = world_pos
            msg.pose.orientation.w = 1.0
            self.pub_b.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSSSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
