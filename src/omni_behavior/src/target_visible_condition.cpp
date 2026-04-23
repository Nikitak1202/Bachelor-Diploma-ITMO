// Copyright (c) 2026 omni_robot authors

#include "omni_behavior/target_visible_condition.hpp"

#include <memory>
#include <string>

#include "rclcpp/qos.hpp"

namespace omni_behavior
{

TargetVisibleCondition::TargetVisibleCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  getInput("topic", topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // Latched publisher on /target_visible -> TRANSIENT_LOCAL durability here too.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_, qos,
    std::bind(&TargetVisibleCondition::onVisible, this, std::placeholders::_1),
    sub_options);
}

void TargetVisibleCondition::onVisible(const std_msgs::msg::Bool::SharedPtr msg)
{
  visible_.store(msg->data);
}

BT::NodeStatus TargetVisibleCondition::tick()
{
  callback_group_executor_.spin_some();
  return visible_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace omni_behavior

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<omni_behavior::TargetVisibleCondition>("TargetVisibleCondition");
}
