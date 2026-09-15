#pragma once  
  
#include <behaviortree_cpp_v3/condition_node.h>  
#include <rclcpp/rclcpp.hpp>  
#include <tf2_ros/buffer.h>  
#include <nav2_util/robot_utils.hpp>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
  
namespace nav2_behavior_tree  
{  
  
class IsNearGoal : public BT::ConditionNode  
{  
public:  
  IsNearGoal(  
    const std::string & name,  
    const BT::NodeConfiguration & conf);  
  
  BT::NodeStatus tick() override;  
  
  static BT::PortsList providedPorts()  
  {  
    return {  
      BT::InputPort<double>("xy_tolerance", 0.5, "XY distance tolerance"),  
      BT::InputPort<double>("yaw_tolerance", 1.57, "Yaw angle tolerance in radians")  
    };  
  }  
  
private:  
  rclcpp::Node::SharedPtr node_;  
  std::shared_ptr<tf2_ros::Buffer> tf_;  
    
  // Helper function to normalize angle to [-pi, pi]  
  double normalizeAngle(double angle);  
};  
  
}  // namespace nav2_behavior_tree