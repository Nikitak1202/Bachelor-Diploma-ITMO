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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

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

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::mutex pose_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_;

  std::string target_pose_topic_{"/target_pose"};
  double target_timeout_{2.0};
  unsigned int power_{1};
  float weight_{6.0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__TARGET_CAMERA_CRITIC_HPP_
