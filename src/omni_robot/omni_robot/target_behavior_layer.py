#!/usr/bin/env python3
"""Compute tracking goals with 1m standoff and heading centering."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class TargetBehaviorLayer(Node):
    def __init__(self):
        super().__init__('target_behavior_layer')
        self.declare_parameter('desired_standoff_m', 1.0)
        self.declare_parameter('distance_deadband_m', 0.1)
        self.declare_parameter('max_forward_step_m', 1.5)
        self.declare_parameter('max_backoff_m', 0.7)
        self.declare_parameter('target_pose_topic', '/target_tracker/target_pose')
        self.declare_parameter('visible_topic', '/target_tracker/target_visible')
        self.declare_parameter('odom_topic', '/omni_robot/odom')
        self.declare_parameter('goal_topic', '/target_tracker/goal_pose')
        self.declare_parameter('output_frame', 'odom')

        self._desired = float(self.get_parameter('desired_standoff_m').value)
        self._deadband = float(self.get_parameter('distance_deadband_m').value)
        self._max_forward = float(self.get_parameter('max_forward_step_m').value)
        self._max_backoff = float(self.get_parameter('max_backoff_m').value)
        self._output_frame = self.get_parameter('output_frame').value

        self._visible = False
        self._target_pose = None
        self._robot_pose = None

        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)

        self.create_subscription(
            Bool, self.get_parameter('visible_topic').value, self._on_visible, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('target_pose_topic').value, self._on_target_pose, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 20)

        self.create_timer(0.2, self._tick)
        self.get_logger().info('target_behavior_layer active')

    def _on_visible(self, msg: Bool):
        self._visible = msg.data

    def _on_target_pose(self, msg: PoseStamped):
        self._target_pose = msg

    def _on_odom(self, msg: Odometry):
        self._robot_pose = msg.pose.pose

    def _tick(self):
        if not self._visible or self._target_pose is None or self._robot_pose is None:
            return

        rx = self._robot_pose.position.x
        ry = self._robot_pose.position.y
        tx = self._target_pose.pose.position.x
        ty = self._target_pose.pose.position.y

        dx = tx - rx
        dy = ty - ry
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 1e-4:
            return

        # Keep a distance band around desired standoff.
        error = dist - self._desired
        if abs(error) < self._deadband:
            step = 0.0
        else:
            step = max(-self._max_backoff, min(self._max_forward, error))

        ux = dx / dist
        uy = dy / dist

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._output_frame
        goal.pose.position.x = rx + ux * step
        goal.pose.position.y = ry + uy * step
        goal.pose.position.z = 0.0

        # Face target to keep it centered in the horizontal FOV.
        yaw = math.atan2(dy, dx)
        goal.pose.orientation = yaw_to_quaternion(yaw)
        self._goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = TargetBehaviorLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
