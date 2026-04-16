#!/usr/bin/env python3
"""Publish TF odom -> base_link from /omni_robot/odom Odometry."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.declare_parameter('odom_topic', '/omni_robot/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        self._odom_topic = self.get_parameter('odom_topic').value
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value

        self._tf_pub = TransformBroadcaster(self)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 30)
        self.get_logger().info(
            'Publishing TF %s -> %s from %s'
            % (self._parent, self._child, self._odom_topic))

    def _on_odom(self, msg: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self._parent
        tf_msg.child_frame_id = self._child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._tf_pub.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
