#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DummyController(Node):
    """
    Placeholder controller that publishes zero velocity.
    Replace with MPC later.
    """
    def __init__(self):
        super().__init__('dummy_controller')
        self.pub = self.create_publisher(Twist, '/omni_robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_zero)
        self.get_logger().info('Dummy controller started (publishing zero velocity)')

    def publish_zero(self):
        twist = Twist()
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DummyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()