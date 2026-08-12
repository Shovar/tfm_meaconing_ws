#!/usr/bin/env python3
"""
Autonomous Robot Mover Node — Fase 0 (complemento).

Publishes cmd_vel (TwistStamped) to both TurtleBot3 robots so they move
autonomously during experiments. This ensures D_UWB varies naturally over time,
making the meaconing detection experiments more realistic.

Movement pattern: each robot follows a circular trajectory with independently
configurable linear and angular velocities. Different speeds → inter-robot
distance varies continuously.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class RobotMoverNode(Node):
    """Publishes constant cmd_vel to two robots for autonomous circular motion."""

    def __init__(self):
        super().__init__('robot_mover_node')

        # --- Parameters ---
        self.robot1_linear = self.declare_parameter('robot1_linear_vel', 0.15).value   # m/s
        self.robot1_angular = self.declare_parameter('robot1_angular_vel', 0.30).value  # rad/s
        self.robot2_linear = self.declare_parameter('robot2_linear_vel', 0.12).value   # m/s
        self.robot2_angular = self.declare_parameter('robot2_angular_vel', 0.25).value  # rad/s
        self.update_rate = self.declare_parameter('update_rate', 20.0).value            # Hz

        # --- Publishers ---
        self.pub_1 = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.pub_2 = self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)

        # --- Timer ---
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._timer_callback)

        r1 = self.robot1_linear / self.robot1_angular if self.robot1_angular != 0 else float('inf')
        r2 = self.robot2_linear / self.robot2_angular if self.robot2_angular != 0 else float('inf')
        self.get_logger().info(
            f'Robot Mover started | '
            f'R1: v={self.robot1_linear:.2f} m/s, ω={self.robot1_angular:.2f} rad/s (r≈{r1:.1f} m) | '
            f'R2: v={self.robot2_linear:.2f} m/s, ω={self.robot2_angular:.2f} rad/s (r≈{r2:.1f} m)'
        )

    def _make_twist(self, linear: float, angular: float) -> TwistStamped:
        """Build a TwistStamped message with the given velocities."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        return msg

    def _timer_callback(self):
        self.pub_1.publish(self._make_twist(self.robot1_linear, self.robot1_angular))
        self.pub_2.publish(self._make_twist(self.robot2_linear, self.robot2_angular))


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
