#!/usr/bin/env python3
"""
GNSS Position Visualizer — Visualiza posiciones GNSS (real vs spoofed) como
markers en RViz2 para facilitar la comprensión del ataque de meaconing.

Publica:
  /visualization/gnss_clean_robot1  — Esfera AZUL  (posición GNSS limpia robot1)
  /visualization/gnss_clean_robot2  — Esfera AZUL  (posición GNSS limpia robot2)
  /visualization/gnss_spoofed_robot1 — Esfera ROJA  (posición GNSS spoofed robot1)
  /visualization/gnss_spoofed_robot2 — Esfera ROJA  (posición GNSS spoofed robot2)

Bajo ataque, las dos esferas ROJAS colapsan al mismo punto derivando,
mientras las esferas AZULES siguen a los robots reales.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np


class GNSSVisualizer(Node):
    """Publica markers RViz2 que muestran las posiciones GNSS limpias y spoofed."""

    def __init__(self):
        super().__init__('gnss_viz_node')

        # --- State ---
        self.clean_a = None   # robot1 clean GNSS
        self.clean_b = None   # robot2 clean GNSS
        self.spoofed_a = None  # robot1 spoofed GNSS
        self.spoofed_b = None  # robot2 spoofed GNSS

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/robot1/gnss_clean', self._cb_clean_a, 10)
        self.create_subscription(
            PoseStamped, '/robot2/gnss_clean', self._cb_clean_b, 10)
        self.create_subscription(
            PoseStamped, '/robot1/gnss_spoofed', self._cb_spoofed_a, 10)
        self.create_subscription(
            PoseStamped, '/robot2/gnss_spoofed', self._cb_spoofed_b, 10)

        # --- Publishers (one per marker) ---
        self.pub_clean_a = self.create_publisher(
            Marker, '/visualization/gnss_clean_robot1', 10)
        self.pub_clean_b = self.create_publisher(
            Marker, '/visualization/gnss_clean_robot2', 10)
        self.pub_spoofed_a = self.create_publisher(
            Marker, '/visualization/gnss_spoofed_robot1', 10)
        self.pub_spoofed_b = self.create_publisher(
            Marker, '/visualization/gnss_spoofed_robot2', 10)

        # --- Timer (10 Hz, enough for visualization) ---
        self.timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info('GNSS Visualizer started (markers on /visualization/*)')

    def _cb_clean_a(self, msg: PoseStamped):
        self.clean_a = msg

    def _cb_clean_b(self, msg: PoseStamped):
        self.clean_b = msg

    def _cb_spoofed_a(self, msg: PoseStamped):
        self.spoofed_a = msg

    def _cb_spoofed_b(self, msg: PoseStamped):
        self.spoofed_b = msg

    def _make_marker(self, ns: str, marker_id: int, pos: Point,
                     r: float, g: float, b: float, alpha: float = 0.8) -> Marker:
        """Create a sphere Marker at the given position."""
        now = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = pos
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = float(alpha)
        marker.lifetime.sec = 1  # disappear after 1s if no update
        return marker

    def _timer_callback(self):
        """Publish markers for all available positions."""
        # --- Clean GNSS (blue) ---
        if self.clean_a is not None:
            self.pub_clean_a.publish(
                self._make_marker('clean', 0, self.clean_a.pose.position,
                                  r=0.2, g=0.4, b=1.0))
        if self.clean_b is not None:
            self.pub_clean_b.publish(
                self._make_marker('clean', 1, self.clean_b.pose.position,
                                  r=0.2, g=0.4, b=1.0))

        # --- Spoofed GNSS (red) ---
        if self.spoofed_a is not None:
            self.pub_spoofed_a.publish(
                self._make_marker('spoofed', 0, self.spoofed_a.pose.position,
                                  r=1.0, g=0.2, b=0.2))
        if self.spoofed_b is not None:
            self.pub_spoofed_b.publish(
                self._make_marker('spoofed', 1, self.spoofed_b.pose.position,
                                  r=1.0, g=0.2, b=0.2))


def main(args=None):
    rclpy.init(args=args)
    node = GNSSVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
