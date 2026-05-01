// Copyright (c) 2026 omni_robot authors

#include "nav2_mppi_controller/critics/potential_field_critic.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"

#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

void PotentialFieldCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 8.0f);
  getParam(influence_distance_, "influence_distance", 0.9f);
  getParam(gradient_sample_distance_, "gradient_sample_distance", 0.12f);
  getParam(min_obstacle_cost_, "min_obstacle_cost", 5.0f);
  getParam(preferred_distance_, "preferred_distance", 0.35f);
  getParam(proximity_weight_, "proximity_weight", 40.0f);
  getParam(direction_weight_, "direction_weight", 1.0f);
  getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0f);
  getParam(inflation_radius_, "inflation_radius", 0.55f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(max_force_cost_, "max_force_cost", 50.0f);
  getParam(publish_debug_markers_, "publish_debug_markers", true);
  getParam(
    debug_markers_topic_, "debug_markers_topic",
    std::string("/potential_field_critic/markers"));
  getParam(debug_publish_rate_hz_, "debug_publish_rate_hz", 10.0);
  getParam(debug_arrow_length_, "debug_arrow_length", 0.45f);

  power_ = std::max(1u, power_);
  weight_ = std::max(0.0f, weight_);
  influence_distance_ = std::max(0.1f, influence_distance_);
  gradient_sample_distance_ = std::max(0.02f, gradient_sample_distance_);
  min_obstacle_cost_ = std::clamp(min_obstacle_cost_, 0.0f, 252.0f);
  preferred_distance_ = std::max(0.01f, preferred_distance_);
  proximity_weight_ = std::max(0.0f, proximity_weight_);
  direction_weight_ = std::max(0.0f, direction_weight_);
  inflation_scale_factor_ = std::max(1e-4f, inflation_scale_factor_);
  inflation_radius_ = std::max(0.05f, inflation_radius_);
  max_force_cost_ = std::max(1.0f, max_force_cost_);
  debug_arrow_length_ = std::max(0.1f, debug_arrow_length_);

  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "PotentialFieldCritic: parent node is gone");
    return;
  }

  if (publish_debug_markers_) {
    debug_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_markers_topic_, rclcpp::QoS(10));
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, debug_publish_rate_hz_));
    debug_timer_ = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PotentialFieldCritic::onDebugTimer, this));
  }

  RCLCPP_INFO(
    logger_,
    "PotentialFieldCritic instantiated with power=%u, weight=%.3f, influence=%.2f m, debug=%s",
    power_, weight_, influence_distance_, publish_debug_markers_ ? "on" : "off");
}

float PotentialFieldCritic::normalizedCostAt(float x, float y) const
{
  const float raw = rawCostAt(x, y);
  const float lethal = static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE);
  return std::clamp(raw / lethal, 0.0f, 1.0f);
}

float PotentialFieldCritic::rawCostAt(float x, float y) const
{
  unsigned int mx = 0u;
  unsigned int my = 0u;
  if (!costmap_ || !costmap_->worldToMap(x, y, mx, my)) {
    return static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  const auto raw = costmap_->getCost(mx, my);
  if (raw == nav2_costmap_2d::NO_INFORMATION) {
    return static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  return static_cast<float>(raw);
}

float PotentialFieldCritic::obstacleDistanceFromCost(float cost) const
{
  if (cost >= static_cast<float>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
    return 0.0f;
  }
  if (cost < 1.0f) {
    return inflation_radius_;
  }
  const float inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  const float dist = (inflation_scale_factor_ * inscribed_radius - std::log(cost) + std::log(253.0f)) /
    inflation_scale_factor_ - inscribed_radius;
  return std::max(0.0f, dist);
}

bool PotentialFieldCritic::computeRepulsionForce(float x, float y, float & fx, float & fy) const
{
  fx = 0.0f;
  fy = 0.0f;

  if (!costmap_) {
    return false;
  }

  const float center_cost = normalizedCostAt(x, y) * 252.0f;
  if (center_cost < min_obstacle_cost_) {
    return false;
  }

  const float d = gradient_sample_distance_;
  const float d2 = std::min(influence_distance_, 2.0f * d);
  const float gx =
    0.7f * (normalizedCostAt(x + d, y) - normalizedCostAt(x - d, y)) / (2.0f * d) +
    0.3f * (normalizedCostAt(x + d2, y) - normalizedCostAt(x - d2, y)) / (2.0f * d2);
  const float gy =
    0.7f * (normalizedCostAt(x, y + d) - normalizedCostAt(x, y - d)) / (2.0f * d) +
    0.3f * (normalizedCostAt(x, y + d2) - normalizedCostAt(x, y - d2)) / (2.0f * d2);

  const float mag = std::hypot(gx, gy);
  if (mag < 1e-4f) {
    return false;
  }

  // Gradient points toward higher obstacle potential, so repulsion is opposite.
  fx = -gx / mag;
  fy = -gy / mag;
  return true;
}

void PotentialFieldCritic::score(CriticData & data)
{
  if (!enabled_ || weight_ <= 0.0f) {
    return;
  }

  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    return;
  }

  const size_t batch = data.trajectories.x.shape(0);
  const size_t traj_len = data.trajectories.x.shape(1);
  if (traj_len < 2) {
    return;
  }

  auto && dir_cost = xt::xtensor<float, 1>::from_shape({batch});
  dir_cost.fill(0.0f);

  for (size_t i = 0; i < batch; ++i) {
    float traj_cost = 0.0f;
    float traversed_distance = 0.0f;
    for (size_t j = 0; j + 1 < traj_len; ++j) {
      const float x = data.trajectories.x(i, j);
      const float y = data.trajectories.y(i, j);
      const float nx = data.trajectories.x(i, j + 1) - x;
      const float ny = data.trajectories.y(i, j + 1) - y;
      const float nmag = std::hypot(nx, ny);
      traversed_distance += nmag;
      if (traversed_distance > influence_distance_) {
        break;
      }

      const float raw_cost = rawCostAt(x, y);
      if (raw_cost >= min_obstacle_cost_) {
        const float obstacle_dist = obstacleDistanceFromCost(raw_cost);
        const float proximity = std::max(0.0f, preferred_distance_ - obstacle_dist);
        traj_cost += proximity_weight_ * proximity;
      }

      float fx = 0.0f;
      float fy = 0.0f;
      if (!computeRepulsionForce(x, y, fx, fy)) {
        continue;
      }

      if (nmag < 1e-4f) {
        continue;
      }

      const float vx = nx / nmag;
      const float vy = ny / nmag;

      const float aligned = vx * fx + vy * fy;
      const float toward_obstacle = std::max(0.0f, -aligned);
      const float nearby_gain = 1.0f + std::max(0.0f, preferred_distance_ - obstacleDistanceFromCost(raw_cost));
      traj_cost += direction_weight_ * nearby_gain * toward_obstacle;
    }
    dir_cost[i] = std::min(traj_cost * weight_, max_force_cost_);
  }

  data.costs += xt::pow(dir_cost / static_cast<float>(traj_len), power_);
}

void PotentialFieldCritic::publishDebugArrow(
  const std::string & frame_id, float x, float y, float fx, float fy, bool has_force)
{
  if (!debug_markers_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = costmap_ros_->get_clock()->now();
  marker.ns = "potential_field_critic";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = has_force ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  marker.scale.x = 0.06;
  marker.scale.y = 0.10;
  marker.scale.z = 0.12;
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = 1.0;

  if (has_force) {
    geometry_msgs::msg::Point p0;
    p0.x = x;
    p0.y = y;
    p0.z = 0.10;
    geometry_msgs::msg::Point p1;
    p1.x = x + debug_arrow_length_ * fx;
    p1.y = y + debug_arrow_length_ * fy;
    p1.z = 0.10;
    marker.points.push_back(p0);
    marker.points.push_back(p1);
  }

  arr.markers.push_back(marker);
  debug_markers_pub_->publish(arr);
}

void PotentialFieldCritic::onDebugTimer()
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

  const float x = static_cast<float>(tf_msg.transform.translation.x);
  const float y = static_cast<float>(tf_msg.transform.translation.y);
  float fx = 0.0f;
  float fy = 0.0f;
  const bool has_force = computeRepulsionForce(x, y, fx, fy);
  publishDebugArrow(frame_id, x, y, fx, fy, has_force);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PotentialFieldCritic,
  mppi::critics::CriticFunction)
