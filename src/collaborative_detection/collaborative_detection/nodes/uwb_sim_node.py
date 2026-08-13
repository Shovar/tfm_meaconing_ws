#!/usr/bin/env python3
"""
UWB Ranging Simulator Node

Subscribes to /robot1/odom and /robot2/odom (ground truth from Gazebo),
converts from local odom frame to global world frame using spawn offsets,
computes the Euclidean distance, adds Gaussian noise, and publishes it.

Noise model: Gaussian with sigma from params.yaml (default 0.24 m — Fishberg et al. 2024).

The DiffDrive plugin publishes odometry in a per-robot local frame that starts
at (0,0) regardless of world spawn position. We add the known spawn offset to
obtain world-frame coordinates for true inter-robot distance.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np


class UWBSimNode(Node):
    """Simulates UWB ranging by computing distance from Gazebo ground truth + noise."""

    def __init__(self):
        super().__init__('uwb_sim_node')

        # --- Parameters ---
        self.sigma_uwb = self.declare_parameter('sigma_uwb', 0.24).value
        self.update_rate = self.declare_parameter('update_rate', 30.0).value
        random_seed = self.declare_parameter('random_seed', 42).value
        np.random.seed(random_seed)

        # --- State ---
        self.pos_a = None  # robot1
        self.pos_b = None  # robot2

        # --- Spawn offsets (odom → world conversion) ---
        self.spawn_a_x = self.declare_parameter('robot1.x', 0.0).value
        self.spawn_a_y = self.declare_parameter('robot1.y', 0.0).value
        self.spawn_b_x = self.declare_parameter('robot2.x', 3.0).value
        self.spawn_b_y = self.declare_parameter('robot2.y', 0.0).value

        # --- Subscribers ---
        self.create_subscription(
            Odometry, '/robot1/odom', self._cb_odom_a, 10)
        self.create_subscription(
            Odometry, '/robot2/odom', self._cb_odom_b, 10)

        # --- Publisher ---
        self.pub = self.create_publisher(Float64, '/robots/uwb_distance', 10)

        # --- Timer ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'UWB Sim Node started (sigma_uwb={self.sigma_uwb:.3f} m, '
            f'rate={self.update_rate:.1f} Hz)')

    def _cb_odom_a(self, msg: Odometry):
        self.pos_a = msg.pose.pose.position

    def _cb_odom_b(self, msg: Odometry):
        self.pos_b = msg.pose.pose.position

    def _timer_callback(self):
        if self.pos_a is None or self.pos_b is None:
            return

        # Euclidean distance in WORLD frame (convert odom → world)
        wx_a = self.pos_a.x + self.spawn_a_x
        wy_a = self.pos_a.y + self.spawn_a_y
        wx_b = self.pos_b.x + self.spawn_b_x
        wy_b = self.pos_b.y + self.spawn_b_y

        dx = wx_a - wx_b
        dy = wy_a - wy_b
        dz = self.pos_a.z - self.pos_b.z
        dist_real = np.sqrt(dx * dx + dy * dy + dz * dz)

        # Add Gaussian noise
        dist_noisy = dist_real + np.random.normal(0.0, self.sigma_uwb)

        msg = Float64()
        msg.data = float(dist_noisy)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
