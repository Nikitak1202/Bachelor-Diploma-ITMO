#!/usr/bin/env python3
"""Relay /omni_robot/scan -> /scan for Nav2 + SLAM defaults."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanBridge(Node):
    def __init__(self):
        super().__init__('scan_bridge')
        self.declare_parameter('in_topic', '/omni_robot/scan')
        self.declare_parameter('out_topic', '/scan')
        t_in = self.get_parameter('in_topic').value
        t_out = self.get_parameter('out_topic').value
        # Receive best-effort sensor stream and republish as reliable for Nav2/SLAM subscribers.
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(LaserScan, t_out, nav_qos)
        self.create_subscription(LaserScan, t_in, self._cb, qos_profile_sensor_data)
        self.get_logger().info('Relay %s -> %s' % (t_in, t_out))

    def _cb(self, msg: LaserScan):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
