#include <behaviortree_cpp_v3/bt_factory.h>

#include "nav2_behavior_tree/set_robot_state.hpp"
#include "nav2_behavior_tree/plugins/condition/is_robot_state.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<elevator_bt::SetRobotState>("SetRobotState");
  factory.registerNodeType<elevator_bt::IsRobotState>("IsRobotState");
}