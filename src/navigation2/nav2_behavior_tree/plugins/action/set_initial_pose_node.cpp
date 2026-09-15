#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class SetInitialPose : public BT::SyncActionNode
{
public:
  SetInitialPose(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("set_initial_pose_bt_node");
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw")   // rad
    };
  }

  BT::NodeStatus tick() override
  {
    double x, y, yaw;
    if (!getInput("x", x)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: x"); return BT::NodeStatus::FAILURE; }
    if (!getInput("y", y)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: y"); return BT::NodeStatus::FAILURE; }
    if (!getInput("yaw", yaw)) { RCLCPP_ERROR(node_->get_logger(), "Missing input port: yaw"); return BT::NodeStatus::FAILURE; }

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = node_->now();
    init_pose.header.frame_id = "map";

    init_pose.pose.pose.position.x = x;
    init_pose.pose.pose.position.y = y;
    init_pose.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    init_pose.pose.pose.orientation.x = q.x();
    init_pose.pose.pose.orientation.y = q.y();
    init_pose.pose.pose.orientation.z = q.z();
    init_pose.pose.pose.orientation.w = q.w();

    // 典型 AMCL 默认协方差（你可以按需修改）
    for (int i = 0; i < 36; i++) {
      init_pose.pose.covariance[i] = 0.0;
    }
    init_pose.pose.covariance[0] = 0.25;   // x
    init_pose.pose.covariance[7] = 0.25;   // y
    init_pose.pose.covariance[35] = 0.0685; // yaw

    pub_->publish(init_pose);

    RCLCPP_INFO(node_->get_logger(), 
        "Set initial pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

// 注册节点
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetInitialPose>("SetInitialPose");
}
