#!/usr/bin/env python3
"""Blue target detection with lidar-assisted map-frame localization."""

import math
import threading

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers geometry message transforms)
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')

        self.declare_parameter('camera_topic', '/omni_robot/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('target_visible_topic', '/target_visible')
        self.declare_parameter('target_marker_topic', '/target_marker')
        self.declare_parameter('annotated_image_topic', '/omni_robot/camera/image_raw/target_status')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('camera_frame', 'base_link')
        self.declare_parameter('min_blue_area_px', 220)
        self.declare_parameter('min_visible_blue_pixels', 100)
        self.declare_parameter('hsv_blue_lower', [112, 170, 70])
        self.declare_parameter('hsv_blue_upper', [128, 255, 255])
        self.declare_parameter('blue_channel_min', 120)
        self.declare_parameter('blue_channel_dominance_min', 60)
        self.declare_parameter('mask_open_kernel_px', 3)
        self.declare_parameter('image_timeout_sec', 0.4)
        self.declare_parameter('camera_hfov_rad', 1.047)
        self.declare_parameter('range_min_m', 0.4)
        self.declare_parameter('range_max_m', 10.0)
        self.declare_parameter('scan_angle_offset_rad', 0.0)
        self.declare_parameter('scan_index_window', 10)
        self.declare_parameter('scan_fallback_window', 80)
        self.declare_parameter('marker_scale_m', 0.22)

        self._camera_topic = self.get_parameter('camera_topic').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._target_pose_topic = self.get_parameter('target_pose_topic').value
        self._target_visible_topic = self.get_parameter('target_visible_topic').value
        self._target_marker_topic = self.get_parameter('target_marker_topic').value
        self._annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self._output_frame = self.get_parameter('output_frame').value
        self._default_camera_frame = self.get_parameter('camera_frame').value
        self._min_area = int(self.get_parameter('min_blue_area_px').value)
        self._min_visible_blue_pixels = max(1, int(self.get_parameter('min_visible_blue_pixels').value))
        self._mask_open_kernel_px = max(1, int(self.get_parameter('mask_open_kernel_px').value))
        if self._mask_open_kernel_px % 2 == 0:
            self._mask_open_kernel_px += 1
        self._blue_channel_min = int(self.get_parameter('blue_channel_min').value)
        self._blue_dominance_min = int(self.get_parameter('blue_channel_dominance_min').value)
        self._image_timeout_ns = int(float(self.get_parameter('image_timeout_sec').value) * 1e9)
        self._hfov = float(self.get_parameter('camera_hfov_rad').value)
        self._range_min = float(self.get_parameter('range_min_m').value)
        self._range_max = float(self.get_parameter('range_max_m').value)
        self._scan_angle_offset = float(self.get_parameter('scan_angle_offset_rad').value)
        self._scan_window = max(0, int(self.get_parameter('scan_index_window').value))
        self._scan_fallback_window = max(self._scan_window, int(self.get_parameter('scan_fallback_window').value))
        self._marker_scale = float(self.get_parameter('marker_scale_m').value)

        lo = self.get_parameter('hsv_blue_lower').value
        hi = self.get_parameter('hsv_blue_upper').value
        self._hsv_lo = np.array(lo, dtype=np.uint8)
        self._hsv_hi = np.array(hi, dtype=np.uint8)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lock = threading.Lock()

        self._visible_cam = False
        self._bearing = 0.0
        self._camera_frame = self._default_camera_frame
        self._last_scan = None
        self._last_image_ns = 0
        self._prev_visible = False

        self._pub_pose = self.create_publisher(PoseStamped, self._target_pose_topic, 10)
        self._pub_vis = self.create_publisher(Bool, self._target_visible_topic, 10)
        self._pub_marker = self.create_publisher(Marker, self._target_marker_topic, 10)
        self._pub_annotated = self.create_publisher(Image, self._annotated_image_topic, 10)

        self.create_subscription(Image, self._camera_topic, self._on_image, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 20)
        self.create_timer(1.0 / 20.0, self._tick)
        self.get_logger().info(
            'target_detector ready (camera=%s scan=%s frame=%s)'
            % (self._camera_topic, self._scan_topic, self._output_frame)
        )

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._last_scan = msg

    def _on_image(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn('cv_bridge: %s' % str(exc))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, self._hsv_lo, self._hsv_hi)
        b, g, r = cv2.split(cv_image)
        dominant_rg = cv2.max(r, g)
        dominant_blue = cv2.subtract(b, dominant_rg)
        blue_min_mask = cv2.inRange(b, self._blue_channel_min, 255)
        blue_dom_mask = cv2.inRange(dominant_blue, self._blue_dominance_min, 255)
        mask = cv2.bitwise_and(hsv_mask, blue_min_mask)
        mask = cv2.bitwise_and(mask, blue_dom_mask)
        kernel = np.ones((self._mask_open_kernel_px, self._mask_open_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _h, width = cv_image.shape[:2]
        visible = False
        bearing = 0.0
        bbox = None

        # Visibility criterion is intentionally simple:
        # if this frame contains blue pixels above a tiny threshold, target is visible.
        blue_pixels = int(cv2.countNonZero(mask))
        visible = blue_pixels >= self._min_visible_blue_pixels and len(contours) > 0

        if visible:
            target_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(target_contour)
            bbox = (x, y, w, h)
            moments = cv2.moments(target_contour)
            if moments['m00'] > 1e-6:
                cx = float(moments['m10'] / moments['m00'])
                x_norm = (cx - (width / 2.0)) / (width / 2.0)
                bearing = x_norm * (self._hfov / 2.0)
            else:
                visible = False

        indicator_color = (0, 255, 0) if visible else (0, 0, 255)
        cv2.circle(cv_image, (18, 18), 10, indicator_color, -1)
        if visible and bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        out_img = self._bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_img.header = msg.header
        self._pub_annotated.publish(out_img)

        with self._lock:
            self._visible_cam = visible
            self._bearing = bearing
            self._last_image_ns = self.get_clock().now().nanoseconds
            if msg.header.frame_id:
                self._camera_frame = msg.header.frame_id

    def _estimate_range_from_scan(self, bearing: float, scan: LaserScan):
        if scan is None or not scan.ranges:
            return None
        angle = bearing + self._scan_angle_offset
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        candidates = self._scan_window_candidates(scan, idx, self._scan_window)
        if not candidates:
            candidates = self._scan_window_candidates(scan, idx, self._scan_fallback_window)
        if not candidates:
            return None
        return float(np.median(np.array(candidates, dtype=float)))

    def _scan_window_candidates(self, scan: LaserScan, idx: int, window: int):
        start = max(0, idx - window)
        stop = min(len(scan.ranges) - 1, idx + window)
        if start > stop:
            return []
        candidates = []
        for i in range(start, stop + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and self._range_min <= r <= self._range_max:
                candidates.append(r)
        return candidates

    def _publish_marker(self, pose: PoseStamped):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self._marker_scale
        marker.scale.y = self._marker_scale
        marker.scale.z = self._marker_scale
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        self._pub_marker.publish(marker)

    def _tick(self):
        with self._lock:
            visible = self._visible_cam
            bearing = self._bearing
            camera_frame = self._camera_frame
            scan = self._last_scan
            last_image_ns = self._last_image_ns

        now_ns = self.get_clock().now().nanoseconds
        image_is_fresh = (now_ns - last_image_ns) <= self._image_timeout_ns if last_image_ns > 0 else False
        visible = visible and image_is_fresh

        self._pub_vis.publish(Bool(data=visible))

        if visible != self._prev_visible:
            self.get_logger().info('target acquired' if visible else 'target lost')
        self._prev_visible = visible

        if not visible:
            return

        range_m = self._estimate_range_from_scan(bearing, scan)
        if range_m is None:
            self.get_logger().warn('target visible but no valid lidar range on bearing', throttle_duration_sec=2.0)
            return

        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = camera_frame
        point_cam.point.x = range_m * math.cos(bearing)
        point_cam.point.y = range_m * math.sin(bearing)
        point_cam.point.z = 0.0

        try:
            point_map = self._tf_buffer.transform(
                point_cam, self._output_frame, timeout=Duration(seconds=0.15)
            )
        except TransformException as exc:
            self.get_logger().warn(
                'failed to transform target point %s -> %s: %s'
                % (camera_frame, self._output_frame, str(exc)),
                throttle_duration_sec=2.0,
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._output_frame
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)
        self._publish_marker(pose)


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
