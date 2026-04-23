#!/usr/bin/env python3
"""Drive NavigateToPose updates from target tracking and run search spin on loss."""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import Spin
from std_msgs.msg import Bool


class TargetNavBridge(Node):
    def __init__(self):
        super().__init__('target_nav_bridge')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('spin_action_name', '/spin')
        self.declare_parameter('goal_period_sec', 0.5)
        self.declare_parameter('goal_update_distance_m', 0.15)
        self.declare_parameter('goal_pose_topic', '/target_pose')
        self.declare_parameter('target_visible_topic', '/target_visible')
        self.declare_parameter('search_spin_distance_rad', 6.283)
        self.declare_parameter('search_spin_time_allowance_sec', 15.0)
        self._action = self.get_parameter('action_name').value
        self._spin_action = self.get_parameter('spin_action_name').value
        self._period = float(self.get_parameter('goal_period_sec').value)
        self._goal_update_distance = float(
            self.get_parameter('goal_update_distance_m').value)
        self._goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self._visible_topic = self.get_parameter('target_visible_topic').value
        self._spin_dist = float(self.get_parameter('search_spin_distance_rad').value)
        self._spin_time_allowance = float(self.get_parameter('search_spin_time_allowance_sec').value)

        self._client = ActionClient(self, NavigateToPose, self._action)
        self._spin_client = ActionClient(self, Spin, self._spin_action)
        self._visible = False
        self._last_pose = None
        self._last_sent_pose = None
        self._goal_handle = None
        self._goal_pending = False
        self._spin_handle = None

        self.create_subscription(Bool, self._visible_topic, self._on_vis, 10)
        self.create_subscription(PoseStamped, self._goal_pose_topic, self._on_pose, 10)
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            'target_nav_bridge: %s from %s (%s)'
            % (self._action, self._goal_pose_topic, self._visible_topic)
        )

    def _on_vis(self, msg: Bool):
        was_visible = self._visible
        self._visible = msg.data
        if self._visible and not was_visible:
            self._cancel_spin()
            self.get_logger().info('target visible -> chase mode')
        if (not self._visible) and was_visible:
            self.get_logger().info('target lost -> continue to last known pose')
            if self._goal_handle is None and not self._goal_pending:
                self._start_search_spin()

    def _on_pose(self, msg: PoseStamped):
        self._last_pose = msg

    def _distance(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

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
            if not self._visible:
                self._start_search_spin()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, _future):
        self._goal_handle = None
        self._goal_pending = False
        if not self._visible:
            self._start_search_spin()

    def _cancel_spin(self):
        if self._spin_handle is not None:
            self._spin_handle.cancel_goal_async()
            self._spin_handle = None

    def _on_spin_response(self, future):
        goal_handle = future.result() if future is not None else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected')
            self._spin_handle = None
            return
        self._spin_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, _future):
        self._spin_handle = None
        if not self._visible and self._goal_handle is None and not self._goal_pending:
            self._start_search_spin()

    def _start_search_spin(self):
        if self._spin_handle is not None:
            return
        if not self._spin_client.wait_for_server(timeout_sec=0.5):
            return
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = self._spin_dist
        spin_goal.time_allowance.sec = int(self._spin_time_allowance)
        spin_goal.time_allowance.nanosec = int((self._spin_time_allowance % 1.0) * 1e9)
        send_future = self._spin_client.send_goal_async(spin_goal)
        send_future.add_done_callback(self._on_spin_response)

    def _tick(self):
        if self._visible:
            self._tick_chase()
            return

        if self._goal_handle is None and not self._goal_pending:
            self._start_search_spin()

    def _tick_chase(self):
        if self._last_pose is None:
            return
        self._cancel_spin()
        if not self._client.wait_for_server(timeout_sec=0.5):
            return
        if self._last_sent_pose is not None:
            if self._distance(self._last_pose, self._last_sent_pose) < self._goal_update_distance:
                return

        if self._goal_handle is not None and self._goal_handle.accepted:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = self._last_pose
        self._last_sent_pose = self._last_pose
        self._goal_pending = True
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            'goal sent: (%.2f, %.2f)'
            % (goal.pose.pose.position.x, goal.pose.pose.position.y))


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
