#!/usr/bin/env python3
"""Send NavigateToPose goals to Nav2 from target pose while visible; cancel when lost."""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


class TargetNavBridge(Node):
    def __init__(self):
        super().__init__('target_nav_bridge')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('goal_period_sec', 0.5)
        self.declare_parameter('goal_update_distance_m', 0.15)
        self.declare_parameter('goal_pose_topic', '/target_tracker/target_pose')
        self._action = self.get_parameter('action_name').value
        self._period = float(self.get_parameter('goal_period_sec').value)
        self._goal_update_distance = float(
            self.get_parameter('goal_update_distance_m').value)
        self._goal_pose_topic = self.get_parameter('goal_pose_topic').value

        self._client = ActionClient(self, NavigateToPose, self._action)
        self._visible = False
        self._last_pose = None
        self._last_sent_pose = None
        self._goal_handle = None

        self.create_subscription(Bool, '/target_tracker/target_visible', self._on_vis, 10)
        self.create_subscription(PoseStamped, self._goal_pose_topic, self._on_pose, 10)
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            'target_nav_bridge: %s from %s'
            % (self._action, self._goal_pose_topic))

    def _on_vis(self, msg: Bool):
        self._visible = msg.data
        if not msg.data:
            self._cancel_current_goal()
            self.get_logger().info('target lost -> cancel goal')

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

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        self._goal_handle = goal_handle

    def _tick(self):
        if not self._visible or self._last_pose is None:
            return
        if not self._client.wait_for_server(timeout_sec=0.5):
            return

        if self._last_sent_pose is not None:
            if self._distance(self._last_pose, self._last_sent_pose) < self._goal_update_distance:
                return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = self._last_pose
        self._last_sent_pose = self._last_pose
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
