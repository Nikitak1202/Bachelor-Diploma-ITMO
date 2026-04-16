#!/usr/bin/env python3
"""Blue target detection from camera with simple odom-frame target estimation."""

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')

        self.declare_parameter('camera_topic', '/omni_robot/camera/image_raw')
        self.declare_parameter('odom_topic', '/omni_robot/odom')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('min_blue_area_px', 30)
        self.declare_parameter('hsv_blue_lower', [90, 50, 50])
        self.declare_parameter('hsv_blue_upper', [140, 255, 255])
        self.declare_parameter('camera_hfov_rad', 1.047)
        self.declare_parameter('range_gain', 70.0)
        self.declare_parameter('range_min_m', 0.6)
        self.declare_parameter('range_max_m', 6.0)

        self._camera_topic = self.get_parameter('camera_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._output_frame = self.get_parameter('output_frame').value
        self._min_area = int(self.get_parameter('min_blue_area_px').value)
        self._hfov = float(self.get_parameter('camera_hfov_rad').value)
        self._range_gain = float(self.get_parameter('range_gain').value)
        self._range_min = float(self.get_parameter('range_min_m').value)
        self._range_max = float(self.get_parameter('range_max_m').value)
        lo = self.get_parameter('hsv_blue_lower').value
        hi = self.get_parameter('hsv_blue_upper').value
        self._hsv_lo = np.array(lo, dtype=np.uint8)
        self._hsv_hi = np.array(hi, dtype=np.uint8)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._visible_cam = False
        self._bearing = 0.0
        self._range_est = self._range_max
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._has_odom = False
        self._prev_visible = False

        self._pub_pose = self.create_publisher(PoseStamped, '/target_tracker/target_pose', 10)
        self._pub_vis = self.create_publisher(Bool, '/target_tracker/target_visible', 10)
        self._pub_range = self.create_publisher(Float32, '/target_tracker/range_m', 10)
        self._pub_horizontal_err = self.create_publisher(Float32, '/target_tracker/horizontal_error', 10)

        self._img_sub = self.create_subscription(
            Image, self._camera_topic, self._on_image, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 20)

        self._timer = self.create_timer(1.0 / 20.0, self._tick)
        self.get_logger().info('target_detector ready (camera=%s)' % self._camera_topic)

    def _on_image(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge: %s' % str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._hsv_lo, self._hsv_hi)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _h, w = cv_image.shape[:2]
        visible = False
        bearing = 0.0
        range_est = self._range_max
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area >= self._min_area:
                M = cv2.moments(c)
                if M['m00'] > 1e-6:
                    cx = float(M['m10'] / M['m00'])
                    x_norm = (cx - (w / 2.0)) / (w / 2.0)
                    bearing = x_norm * (self._hfov / 2.0)
                    range_est = self._range_gain / math.sqrt(max(area, 1.0))
                    range_est = max(self._range_min, min(self._range_max, range_est))
                    visible = True

        with self._lock:
            self._visible_cam = visible
            self._bearing = bearing
            self._range_est = range_est

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self._has_odom = True

    def _tick(self):
        with self._lock:
            visible = self._visible_cam
            bearing = self._bearing
            range_est = self._range_est
            has_odom = self._has_odom
            rx = self._robot_x
            ry = self._robot_y
            ryaw = self._robot_yaw

        vis_msg = Bool()
        vis_msg.data = visible
        self._pub_vis.publish(vis_msg)

        if visible != self._prev_visible:
            if visible:
                self.get_logger().info('target acquired')
            else:
                self.get_logger().info('target lost')
        self._prev_visible = visible

        if not visible or not has_odom:
            if visible and not has_odom:
                self.get_logger().warn('target visible but odom not received yet', throttle_duration_sec=5.0)
            return

        range_msg = Float32()
        range_msg.data = range_est
        self._pub_range.publish(range_msg)
        bearing_msg = Float32()
        bearing_msg.data = bearing
        self._pub_horizontal_err.publish(bearing_msg)

        target_yaw = ryaw + bearing
        target_x = rx + range_est * math.cos(target_yaw)
        target_y = ry + range_est * math.sin(target_yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._output_frame
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
