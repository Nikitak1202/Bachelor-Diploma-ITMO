// Copyright (c) 2026 omni_robot authors

#include "nav2_mppi_controller/critics/target_camera_critic.hpp"

#include <cmath>
#include <memory>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace mppi::critics
{

void TargetCameraCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 100.0f);
  getParam(target_pose_topic_, "target_pose_topic", std::string("/target_pose"));
  getParam(target_timeout_, "target_timeout", 0.4);
  getParam(use_stale_target_, "use_stale_target", true);
  getParam(camera_yaw_offset_, "camera_yaw_offset", 3.14f);
  getParam(angular_deadband_, "angular_deadband", 0.01f);
  getParam(terminal_weight_, "terminal_weight", 0.7f);
  getParam(max_cost_, "max_cost", 25.0f);
  getParam(publish_debug_markers_, "publish_debug_markers", true);
  getParam(debug_markers_topic_, "debug_markers_topic", std::string("/target_camera_critic/markers"));
  getParam(debug_arrow_length_, "debug_arrow_length", 0.3f);
  getParam(debug_publish_rate_hz_, "debug_publish_rate_hz", 10.0);
  terminal_weight_ = std::clamp(terminal_weight_, 0.0f, 1.0f);
  max_cost_ = std::max(1.0f, max_cost_);
  if (power_ > 4) {
    RCLCPP_WARN(logger_, "TargetCameraCritic: cost_power=%u is too aggressive, capping to 4", power_);
    power_ = 4;
  }

  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "TargetCameraCritic: parent node is gone, cannot subscribe");
    return;
  }

  sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic_, rclcpp::QoS(10),
    std::bind(&TargetCameraCritic::onTargetPose, this, std::placeholders::_1));
  if (publish_debug_markers_) {
    debug_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_markers_topic_, rclcpp::QoS(10));
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, debug_publish_rate_hz_));
    debug_timer_ = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TargetCameraCritic::onDebugTimer, this));
  }

  RCLCPP_INFO(
    logger_,
    "TargetCameraCritic instantiated with %d power, %f weight, topic=%s, timeout=%.2fs, stale=%s, yaw_offset=%.3f, deadband=%.3f, terminal_w=%.2f, max_cost=%.2f, debug=%s",
    power_, weight_, target_pose_topic_.c_str(), target_timeout_, use_stale_target_ ? "on" : "off",
    camera_yaw_offset_, angular_deadband_, terminal_weight_, max_cost_,
    publish_debug_markers_ ? "on" : "off");
}

void TargetCameraCritic::onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_target_ = *msg;
}

void TargetCameraCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const std::string & traj_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped target_traj;
  if (!tryGetTargetInFrame(traj_frame, target_traj, true)) {
    return;
  }

  const float tx = static_cast<float>(target_traj.pose.position.x);
  const float ty = static_cast<float>(target_traj.pose.position.y);

  // Blend horizon and terminal bearing errors so robot rotates while continuing path progress.
  auto desired_all = xt::eval(
    xt::atan2(ty - data.trajectories.y, tx - data.trajectories.x) + camera_yaw_offset_);
  auto angular_err_all = xt::eval(
    xt::fabs(utils::shortest_angular_distance(data.trajectories.yaws, desired_all)));
  auto mean_err = xt::eval(xt::mean(angular_err_all, {1}));

  const auto last = data.trajectories.x.shape(1) - 1;
  auto desired_last = xt::eval(
    xt::atan2(ty - xt::view(data.trajectories.y, xt::all(), last),
    tx - xt::view(data.trajectories.x, xt::all(), last)) + camera_yaw_offset_);
  auto angular_err_last = xt::eval(
    xt::fabs(
      utils::shortest_angular_distance(
        xt::view(data.trajectories.yaws, xt::all(), last), desired_last)));

  auto combined_err = xt::eval((1.0f - terminal_weight_) * mean_err + terminal_weight_ * angular_err_last);
  auto effective_err = xt::eval(xt::maximum(0.0f, combined_err - angular_deadband_));
  auto raw_cost = xt::eval(effective_err * weight_);
  auto bounded_cost = xt::eval(xt::minimum(raw_cost, max_cost_));
  data.costs += xt::pow(bounded_cost, power_);
}

void TargetCameraCritic::publishDebugMarkers(
  const std::string & frame_id, float robot_x, float robot_y, float current_yaw, float desired_yaw,
  bool has_desired_yaw)
{
  visualization_msgs::msg::MarkerArray arr;
  const auto stamp = costmap_ros_->get_clock()->now();

  auto make_arrow = [&](int id, float yaw, float r, float g, float b) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "target_camera_critic";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.06;
    m.scale.y = 0.10;
    m.scale.z = 0.12;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    geometry_msgs::msg::Point p0;
    p0.x = robot_x;
    p0.y = robot_y;
    p0.z = 0.08;
    geometry_msgs::msg::Point p1;
    p1.x = robot_x + debug_arrow_length_ * std::cos(yaw);
    p1.y = robot_y + debug_arrow_length_ * std::sin(yaw);
    p1.z = 0.08;
    m.points.push_back(p0);
    m.points.push_back(p1);
    return m;
  };

  arr.markers.push_back(make_arrow(0, current_yaw, 0.0f, 1.0f, 0.0f));
  if (has_desired_yaw) {
    arr.markers.push_back(make_arrow(1, desired_yaw, 1.0f, 0.45f, 0.0f));
  } else {
    auto del = make_arrow(1, current_yaw, 1.0f, 0.45f, 0.0f);
    del.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(del);
  }
  debug_markers_pub_->publish(arr);
}

bool TargetCameraCritic::tryGetTargetInFrame(
  const std::string & frame_id, geometry_msgs::msg::PoseStamped & target_traj, bool for_scoring)
{
  geometry_msgs::msg::PoseStamped target_in;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!latest_target_.has_value()) {
      return false;
    }
    target_in = latest_target_.value();
  }

  auto node = parent_.lock();
  if (!node) {
    return false;
  }

  if (!use_stale_target_) {
    const auto now = node->now();
    const rclcpp::Time stamp(target_in.header.stamp, RCL_ROS_TIME);
    if ((now - stamp).seconds() > target_timeout_) {
      return false;
    }
  } else if (for_scoring) {
    const auto now = node->now();
    const rclcpp::Time stamp(target_in.header.stamp, RCL_ROS_TIME);
    if ((now - stamp).seconds() > target_timeout_) {
      // Keep old target for marker continuity and last-known-pose chase, but don't score forever.
      return false;
    }
  }

  try {
    costmap_ros_->getTfBuffer()->transform(
      target_in, target_traj, frame_id, tf2::durationFromSec(0.1));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "TargetCameraCritic: failed to transform target from %s to %s: %s",
      target_in.header.frame_id.c_str(), frame_id.c_str(), ex.what());
    return false;
  }
}

void TargetCameraCritic::onDebugTimer()
{
  if (!publish_debug_markers_ || !debug_markers_pub_ || !costmap_ros_) {
    return;
  }
  const std::string frame_id = costmap_ros_->getGlobalFrameID();
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = costmap_ros_->getTfBuffer()->lookupTransform(
      frame_id, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException &) {
    return;
  }

  const float robot_x = static_cast<float>(tf_msg.transform.translation.x);
  const float robot_y = static_cast<float>(tf_msg.transform.translation.y);
  const float current_yaw = static_cast<float>(tf2::getYaw(tf_msg.transform.rotation));

  geometry_msgs::msg::PoseStamped target_traj;
  if (!tryGetTargetInFrame(frame_id, target_traj, false)) {
    publishDebugMarkers(frame_id, robot_x, robot_y, current_yaw, current_yaw, false);
    return;
  }

  const float tx = static_cast<float>(target_traj.pose.position.x);
  const float ty = static_cast<float>(target_traj.pose.position.y);
  const float desired_yaw = std::atan2(ty - robot_y, tx - robot_x) + camera_yaw_offset_;
  publishDebugMarkers(frame_id, robot_x, robot_y, current_yaw, desired_yaw, true);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TargetCameraCritic,
  mppi::critics::CriticFunction)
