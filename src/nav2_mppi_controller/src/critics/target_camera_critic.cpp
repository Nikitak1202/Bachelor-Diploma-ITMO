// Copyright (c) 2026 omni_robot authors

#include "nav2_mppi_controller/critics/target_camera_critic.hpp"

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::critics
{

void TargetCameraCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 6.0);
  getParam(target_pose_topic_, "target_pose_topic", std::string("/target_pose"));
  getParam(target_timeout_, "target_timeout", 2.0);

  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "TargetCameraCritic: parent node is gone, cannot subscribe");
    return;
  }

  sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic_, rclcpp::QoS(10),
    std::bind(&TargetCameraCritic::onTargetPose, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "TargetCameraCritic instantiated with %d power, %f weight, topic=%s, timeout=%.2fs",
    power_, weight_, target_pose_topic_.c_str(), target_timeout_);
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

  geometry_msgs::msg::PoseStamped target_in;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!latest_target_.has_value()) {
      return;
    }
    target_in = latest_target_.value();
  }

  auto node = parent_.lock();
  if (!node) {
    return;
  }

  // Drop stale target updates so the critic disengages when target is lost.
  const auto now = node->now();
  const rclcpp::Time stamp(target_in.header.stamp, RCL_ROS_TIME);
  if ((now - stamp).seconds() > target_timeout_) {
    return;
  }

  // Project the target into the same frame as the candidate trajectories.
  const std::string & traj_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped target_traj;
  try {
    costmap_ros_->getTfBuffer()->transform(
      target_in, target_traj, traj_frame, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "TargetCameraCritic: failed to transform target from %s to %s: %s",
      target_in.header.frame_id.c_str(), traj_frame.c_str(), ex.what());
    return;
  }

  const float tx = static_cast<float>(target_traj.pose.position.x);
  const float ty = static_cast<float>(target_traj.pose.position.y);

  // Desired bearing per (batch, timestep) toward target, in the trajectory frame.
  auto desired = xt::eval(xt::atan2(ty - data.trajectories.y, tx - data.trajectories.x));
  auto angular_err = xt::eval(
    xt::fabs(utils::shortest_angular_distance(data.trajectories.yaws, desired)));

  data.costs += xt::pow(xt::mean(angular_err, {1}) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TargetCameraCritic,
  mppi::critics::CriticFunction)
