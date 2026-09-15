#include "nav2_behavior_tree/plugins/condition/is_near_goal_condition.hpp"  
#include <nav2_util/geometry_utils.hpp>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>  
  
namespace nav2_behavior_tree  
{  
  
IsNearGoal::IsNearGoal(  
  const std::string & name,  
  const BT::NodeConfiguration & conf)  
: BT::ConditionNode(name, conf)  
{  
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");  
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");  
}  
  
double IsNearGoal::normalizeAngle(double angle)  
{  
  while (angle > M_PI) angle -= 2 * M_PI;  
  while (angle < -M_PI) angle += 2 * M_PI;  
  return angle;  
}  
  
BT::NodeStatus IsNearGoal::tick()  
{  
  double xy_tol, yaw_tol;  
  getInput("xy_tolerance", xy_tol);  
  getInput("yaw_tolerance", yaw_tol);  
  
  geometry_msgs::msg::PoseStamped robot_pose;  
  geometry_msgs::msg::PoseStamped goal_pose;  
  
  if (!nav2_util::getCurrentPose(  
        robot_pose, *tf_, "map", "base_link", 0.1))  
  {  
    return BT::NodeStatus::FAILURE;  
  }  
  
  if (!config().blackboard->get("goal", goal_pose))  
  {  
    return BT::NodeStatus::FAILURE;  
  }  
  
  double dx = robot_pose.pose.position.x - goal_pose.pose.position.x;  
  double dy = robot_pose.pose.position.y - goal_pose.pose.position.y;  
  double dist = std::hypot(dx, dy);  
  
  // 使用 tf2::getYaw 从四元数获取偏航角  
  tf2::Quaternion q_r, q_g;
  tf2::fromMsg(robot_pose.pose.orientation, q_r);
  tf2::fromMsg(goal_pose.pose.orientation, q_g);

  double roll, pitch, yaw_r, yaw_g;
  tf2::Matrix3x3(q_r).getRPY(roll, pitch, yaw_r);
  tf2::Matrix3x3(q_g).getRPY(roll, pitch, yaw_g); 
  double dyaw = std::fabs(normalizeAngle(yaw_r - yaw_g));  
  
  if (dist < xy_tol && dyaw < yaw_tol) {  
    return BT::NodeStatus::SUCCESS;  
  }  
  
  return BT::NodeStatus::FAILURE;  
}  
  
}  // namespace nav2_behavior_tree  
  
#include "behaviortree_cpp_v3/bt_factory.h"  
BT_REGISTER_NODES(factory)  
{  
  factory.registerNodeType<nav2_behavior_tree::IsNearGoal>("IsNearGoal");  
}