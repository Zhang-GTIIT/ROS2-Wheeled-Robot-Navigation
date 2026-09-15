#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <string>

class RunShell : public BT::SyncActionNode
{
public:
  RunShell(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("run_shell_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("script")   // 传入脚本路径
    };
  }

  BT::NodeStatus tick() override
  {
    std::string script_path;
    if (!getInput("script", script_path))
    {
      RCLCPP_ERROR(node_->get_logger(), "Missing input port: script");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing shell script: %s", script_path.c_str());

    // 统一使用 bash 执行
    std::string cmd = "bash " + script_path;

    int ret = system(cmd.c_str());

    if (ret == 0)
    {
      RCLCPP_INFO(node_->get_logger(), "Shell script executed SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Shell script FAILED, return code = %d", ret);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
};


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RunShell>("RunShell");
}
