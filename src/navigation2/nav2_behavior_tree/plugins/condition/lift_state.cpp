#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class LiftState : public BT::ConditionNode
{
public:
  LiftState(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("lift_state_bt_node");

    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/yolo/result",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        last_state_ = msg->data;
      });
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("expected") };
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);

    std::string expected;
    if (!getInput("expected", expected)) {
      RCLCPP_ERROR(node_->get_logger(),
        "Input port [expected] not found!");
      return BT::NodeStatus::FAILURE;
    }

    // RCLCPP_INFO(node_->get_logger(),
    //   "YOLO state: '%s', expecting: '%s'",
    //   last_state_.c_str(), expected.c_str());

    return (last_state_ == expected) ?
      BT::NodeStatus::SUCCESS :
      BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string last_state_ {"unknown"};
};
#include "behaviortree_cpp_v3/bt_factory.h"
// 必须放在全局作用域，且必须这样写
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LiftState>("LiftState");
}
