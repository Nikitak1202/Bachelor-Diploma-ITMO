// Copyright (c) 2026 omni_robot authors
//
// MPPI critic adding a local potential-field directional bias away from
// nearby obstacles to reduce bump-like contacts for omni motions.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__POTENTIAL_FIELD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__POTENTIAL_FIELD_CRITIC_HPP_

#include <algorithm>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi::critics
{

class PotentialFieldCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  inline float normalizedCostAt(float x, float y) const;
  inline float rawCostAt(float x, float y) const;
  inline bool computeRepulsionForce(float x, float y, float & fx, float & fy) const;
  inline float obstacleDistanceFromCost(float cost) const;
  float maxSampledRawCostAt(float x, float y) const;
  void onDebugTimer();
  void publishDebugArrow(
    const std::string & frame_id, float x, float y, float fx, float fy, bool has_force);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  unsigned int power_{1};
  float weight_{8.0f};
  float influence_distance_{0.9f};
  float gradient_sample_distance_{0.12f};
  float min_obstacle_cost_{5.0f};
  float preferred_distance_{0.35f};
  float proximity_weight_{40.0f};
  float direction_weight_{1.0f};
  float inflation_scale_factor_{10.0f};
  float inflation_radius_{0.55f};
  float near_goal_distance_{0.5f};
  float max_force_cost_{50.0f};
  float activation_cost_threshold_{30.0f};
  float activation_sample_radius_{0.0f};
  float dominance_scale_{8.0f};
  bool publish_debug_markers_{true};
  std::string debug_markers_topic_{"/potential_field_critic/markers"};
  double debug_publish_rate_hz_{10.0};
  float debug_arrow_length_{0.45f};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__POTENTIAL_FIELD_CRITIC_HPP_
