#!/usr/bin/env python3
"""Drive NavigateToPose using nearest free approach-point around the target."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import Spin
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker


class TargetNavBridge(Node):
    def __init__(self):
        super().__init__('target_nav_bridge')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('spin_action_name', '/spin')
        self.declare_parameter('goal_period_sec', 0.5)
        self.declare_parameter('goal_update_distance_m', 0.15)
        self.declare_parameter('last_pose_goal_tolerance_m', 0.5)
        self.declare_parameter('lost_visibility_timeout_sec', 0.8)
        self.declare_parameter('goal_pose_topic', '/target_pose')
        self.declare_parameter('target_visible_topic', '/target_visible')
        self.declare_parameter('nav_mode_topic', '/target_nav_mode')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('target_tolerance_m', 1)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('selected_goal_topic', '/target_nav_goal')
        self.declare_parameter('selected_goal_marker_topic', '/target_nav_goal_marker')
        self.declare_parameter('selected_goal_marker_scale_m', 0.20)
        self.declare_parameter('search_spin_distance_rad', 6.283)
        self.declare_parameter('search_spin_time_allowance_sec', 15.0)
        self._action = self.get_parameter('action_name').value
        self._spin_action = self.get_parameter('spin_action_name').value
        self._period = float(self.get_parameter('goal_period_sec').value)
        self._goal_update_distance = float(
            self.get_parameter('goal_update_distance_m').value)
        self._last_pose_goal_tolerance = float(
            self.get_parameter('last_pose_goal_tolerance_m').value)
        self._lost_visibility_timeout_ns = int(
            float(self.get_parameter('lost_visibility_timeout_sec').value) * 1e9)
        self._goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self._visible_topic = self.get_parameter('target_visible_topic').value
        self._nav_mode_topic = self.get_parameter('nav_mode_topic').value
        self._map_topic = self.get_parameter('map_topic').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        self._target_tolerance = float(self.get_parameter('target_tolerance_m').value)
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self._allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self._selected_goal_topic = self.get_parameter('selected_goal_topic').value
        self._selected_goal_marker_topic = self.get_parameter('selected_goal_marker_topic').value
        self._selected_goal_marker_scale = float(
            self.get_parameter('selected_goal_marker_scale_m').value)
        self._spin_dist = float(self.get_parameter('search_spin_distance_rad').value)
        self._spin_time_allowance = float(self.get_parameter('search_spin_time_allowance_sec').value)

        self._client = ActionClient(self, NavigateToPose, self._action)
        self._spin_client = ActionClient(self, Spin, self._spin_action)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._visible_raw = False
        self._last_seen_ns = 0
        self._last_pose = None
        self._last_map = None
        self._last_sent_pose = None
        self._goal_handle = None
        self._goal_pending = False
        self._active_goal_mode = ''
        self._spin_handle = None
        self._spin_pending = False
        self._mode = ''

        self.create_subscription(Bool, self._visible_topic, self._on_vis, 10)
        self.create_subscription(PoseStamped, self._goal_pose_topic, self._on_pose, 10)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)
        self._mode_pub = self.create_publisher(String, self._nav_mode_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, self._selected_goal_topic, 10)
        self._goal_marker_pub = self.create_publisher(Marker, self._selected_goal_marker_topic, 10)
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            'target_nav_bridge: %s target=%s visible=%s map=%s'
            % (self._action, self._goal_pose_topic, self._visible_topic, self._map_topic)
        )

    def _on_vis(self, msg: Bool):
        was_effective_visible = self._is_effectively_visible()
        self._visible_raw = msg.data
        if self._visible_raw:
            self._last_seen_ns = self.get_clock().now().nanoseconds
        if self._is_effectively_visible() and not was_effective_visible:
            self._cancel_spin()
            self.get_logger().info('target visible -> chase mode')
        if (not self._is_effectively_visible()) and was_effective_visible:
            self.get_logger().info('target lost -> continue to last known pose')

    def _is_effectively_visible(self) -> bool:
        if self._visible_raw:
            return True
        if self._last_seen_ns <= 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self._last_seen_ns) <= self._lost_visibility_timeout_ns

    def _set_mode(self, mode: str):
        if mode == self._mode:
            return
        self._mode = mode
        self._mode_pub.publish(String(data=mode))

    def _resolve_mode(self) -> str:
        if self._spin_handle is not None or self._spin_pending:
            return "Spining"
        if self._is_effectively_visible():
            return "Chase target"
        if self._goal_handle is not None or self._goal_pending or self._last_pose is not None:
            return "Go to last target's pose"
        return "Spining"

    def _on_pose(self, msg: PoseStamped):
        self._last_pose = msg

    def _on_map(self, msg: OccupancyGrid):
        self._last_map = msg

    def _distance(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _get_robot_xy(self):
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        return (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
        )

    def _world_to_cell(self, map_msg: OccupancyGrid, x: float, y: float):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)
        if mx < 0 or my < 0 or mx >= map_msg.info.width or my >= map_msg.info.height:
            return None
        return (mx, my)

    def _is_free_cell(self, map_msg: OccupancyGrid, mx: int, my: int) -> bool:
        idx = my * map_msg.info.width + mx
        if idx < 0 or idx >= len(map_msg.data):
            return False
        occ = int(map_msg.data[idx])
        if occ < 0:
            return self._allow_unknown
        return occ < self._occupied_threshold

    def _cell_center_world(self, map_msg: OccupancyGrid, mx: int, my: int):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        x = origin_x + (mx + 0.5) * res
        y = origin_y + (my + 0.5) * res
        return (x, y)

    def _select_approach_goal(self, target_pose: PoseStamped):
        map_msg = self._last_map
        if map_msg is None:
            return target_pose
        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return target_pose

        tx = float(target_pose.pose.position.x)
        ty = float(target_pose.pose.position.y)
        tol = max(0.0, self._target_tolerance)
        res = map_msg.info.resolution
        radius_cells = int(math.ceil(tol / res))
        center_cell = self._world_to_cell(map_msg, tx, ty)
        if center_cell is None:
            return target_pose

        best = None
        tol2 = tol * tol
        cx, cy = center_cell
        for my in range(cy - radius_cells, cy + radius_cells + 1):
            if my < 0 or my >= map_msg.info.height:
                continue
            for mx in range(cx - radius_cells, cx + radius_cells + 1):
                if mx < 0 or mx >= map_msg.info.width:
                    continue
                wx, wy = self._cell_center_world(map_msg, mx, my)
                if ((wx - tx) * (wx - tx) + (wy - ty) * (wy - ty)) > tol2:
                    continue
                if not self._is_free_cell(map_msg, mx, my):
                    continue
                robot_dist2 = (wx - robot_xy[0]) * (wx - robot_xy[0]) + (wy - robot_xy[1]) * (wy - robot_xy[1])
                if best is None or robot_dist2 < best[0]:
                    best = (robot_dist2, wx, wy)

        if best is None:
            return target_pose

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._map_frame
        goal.pose.position.x = best[1]
        goal.pose.position.y = best[2]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def _publish_selected_goal_viz(self, pose: PoseStamped):
        self._goal_pub.publish(pose)
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'target_nav_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self._selected_goal_marker_scale
        marker.scale.y = self._selected_goal_marker_scale
        marker.scale.z = self._selected_goal_marker_scale
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 1.0
        self._goal_marker_pub.publish(marker)

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_pending = False

    def _on_goal_response(self, future):
        self._goal_pending = False
        goal_handle = future.result() if future is not None else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            if not self._is_effectively_visible() and self._last_pose is None:
                self._start_search_spin()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, _future):
        self._goal_handle = None
        self._goal_pending = False
        # Spin only when last-known-pose navigation is completed and target is still absent.
        if (
            self._active_goal_mode == "Go to last target's pose" and
            not self._is_effectively_visible()
        ):
            self._start_search_spin()

    def _cancel_spin(self):
        if self._spin_handle is not None:
            self._spin_handle.cancel_goal_async()
            self._spin_handle = None
        self._spin_pending = False

    def _on_spin_response(self, future):
        self._spin_pending = False
        goal_handle = future.result() if future is not None else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected')
            self._spin_handle = None
            return
        self._spin_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, _future):
        self._spin_handle = None
        if not self._is_effectively_visible() and self._goal_handle is None and not self._goal_pending:
            self._start_search_spin()

    def _start_search_spin(self):
        if self._spin_handle is not None or self._spin_pending:
            return
        if not self._spin_client.wait_for_server(timeout_sec=0.5):
            return
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = self._spin_dist
        spin_goal.time_allowance.sec = int(self._spin_time_allowance)
        spin_goal.time_allowance.nanosec = int((self._spin_time_allowance % 1.0) * 1e9)
        self._spin_pending = True
        send_future = self._spin_client.send_goal_async(spin_goal)
        send_future.add_done_callback(self._on_spin_response)

    def _tick(self):
        if self._spin_handle is not None or self._spin_pending:
            pass
        elif self._is_effectively_visible():
            self._tick_chase()
        elif self._last_pose is not None:
            self._tick_last_known_pose()
        elif self._goal_handle is None and not self._goal_pending:
            self._start_search_spin()
        self._set_mode(self._resolve_mode())

    def _tick_chase(self):
        if self._last_pose is None:
            return
        self._cancel_spin()
        selected_goal = self._select_approach_goal(self._last_pose)
        self._publish_selected_goal_viz(selected_goal)
        if not self._client.wait_for_server(timeout_sec=0.5):
            return
        if self._last_sent_pose is not None:
            if self._distance(selected_goal, self._last_sent_pose) < self._goal_update_distance:
                return

        if self._goal_handle is not None and self._goal_handle.accepted:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = selected_goal
        self._last_sent_pose = selected_goal
        self._goal_pending = True
        self._active_goal_mode = "Chase target"
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            'goal sent: (%.2f, %.2f), target=(%.2f, %.2f)'
            % (
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
                self._last_pose.pose.position.x,
                self._last_pose.pose.position.y,
            )
        )

    def _tick_last_known_pose(self):
        if self._last_pose is None:
            return
        self._cancel_spin()
        selected_goal = self._select_approach_goal(self._last_pose)
        self._publish_selected_goal_viz(selected_goal)
        robot_xy = self._get_robot_xy()
        if robot_xy is not None:
            goal_dx = selected_goal.pose.position.x - robot_xy[0]
            goal_dy = selected_goal.pose.position.y - robot_xy[1]
            goal_dist = math.sqrt(goal_dx * goal_dx + goal_dy * goal_dy)
            if goal_dist <= self._last_pose_goal_tolerance:
                self._cancel_current_goal()
                self._start_search_spin()
                return
        if not self._client.wait_for_server(timeout_sec=0.5):
            return
        if self._last_sent_pose is not None:
            if self._distance(selected_goal, self._last_sent_pose) < self._goal_update_distance:
                return
        if self._goal_handle is not None and self._goal_handle.accepted:
            return

        goal = NavigateToPose.Goal()
        goal.pose = selected_goal
        self._last_sent_pose = selected_goal
        self._goal_pending = True
        self._active_goal_mode = "Go to last target's pose"
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)


def main(args=None):
    rclpy.init(args=args)
    node = TargetNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
