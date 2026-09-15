#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class SetGoal : public BT::SyncActionNode
{
public:
  SetGoal(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
  {
    // 从黑板获取已有的 rclcpp::Node::SharedPtr
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw"),   // rad
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus tick() override
  {
    double x, y, yaw;
    if (!getInput("x", x)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: x"); return BT::NodeStatus::FAILURE; }
    if (!getInput("y", y)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: y"); return BT::NodeStatus::FAILURE; }
    if (!getInput("yaw", yaw)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: yaw"); return BT::NodeStatus::FAILURE; }

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();
    goal_msg.pose.orientation.w = q.w();

    setOutput("goal", goal_msg);

    RCLCPP_INFO(node_->get_logger(), "Set goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

// 注册节点
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetGoal>("SetGoal");
}
