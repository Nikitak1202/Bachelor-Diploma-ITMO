// Copyright (c) 2026 omni_robot authors
//
// BT condition returning SUCCESS while the blue target is visible in camera.
// Subscribes to a latched std_msgs/Bool topic published by target_detector.

#ifndef OMNI_BEHAVIOR__TARGET_VISIBLE_CONDITION_HPP_
#define OMNI_BEHAVIOR__TARGET_VISIBLE_CONDITION_HPP_

#include <atomic>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace omni_behavior
{

class TargetVisibleCondition : public BT::ConditionNode
{
public:
  TargetVisibleCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TargetVisibleCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", std::string("/target_visible"),
        "Topic providing the visibility flag"),
    };
  }

private:
  void onVisible(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> visible_{false};
  std::string topic_;
};

}  // namespace omni_behavior

#endif  // OMNI_BEHAVIOR__TARGET_VISIBLE_CONDITION_HPP_
