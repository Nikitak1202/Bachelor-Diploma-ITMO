#!/usr/bin/env python3
"""Relay /cmd_vel (Nav2) -> /omni_robot/cmd_vel (Gazebo plugin)."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('out_topic', '/omni_robot/cmd_vel')
        out_topic = self.get_parameter('out_topic').value
        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('Relay /cmd_vel -> %s' % out_topic)

    def _cb(self, msg: Twist):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
