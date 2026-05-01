// Copyright (c) 2026 omni_robot authors
//
// MPPI critic penalizing trajectory endpoints whose heading deviates from
// the bearing toward the tracked target. Keeps the target roughly in the
// center of the camera frame while chasing.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__TARGET_CAMERA_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__TARGET_CAMERA_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

class TargetCameraCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

protected:
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onDebugTimer();
  bool tryGetTargetInFrame(
    const std::string & frame_id, geometry_msgs::msg::PoseStamped & target_traj, bool for_scoring);
  void publishDebugMarkers(
    const std::string & frame_id, float robot_x, float robot_y, float current_yaw, float desired_yaw,
    bool has_desired_yaw);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  std::mutex pose_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_;

  std::string target_pose_topic_{"/target_pose"};
  std::string debug_markers_topic_{"/target_camera_critic/markers"};
  double target_timeout_{0.4};
  double debug_publish_rate_hz_{10.0};
  float camera_yaw_offset_{0.0f};
  float angular_deadband_{0.05f};
  float terminal_weight_{0.7f};
  float max_cost_{25.0f};
  float debug_arrow_length_{0.7f};
  bool use_stale_target_{true};
  bool publish_debug_markers_{true};
  unsigned int power_{1};
  float weight_{10.0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__TARGET_CAMERA_CRITIC_HPP_
