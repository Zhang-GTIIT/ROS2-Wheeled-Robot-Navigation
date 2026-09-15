// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// You may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/plugins/action/spin_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/utils.h>   // for tf2::getYaw
#include "rclcpp/logger.hpp"

namespace nav2_behavior_tree
{

SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf),
  tf_buffer_(this->node_->get_clock()),
  tf_listener_(tf_buffer_)
{


  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  getInput("is_recovery", is_recovery_);


}

void SpinAction::on_tick()
{
  if (is_recovery_) {
    increment_recovery_count();
  }
  double dist;
  getInput("spin_dist", dist);
  // 获取当前位置并计算目标旋转角度

  geometry_msgs::msg::TransformStamped tf_msg =
    tf_buffer_.lookupTransform("map", "base_laser", tf2::TimePointZero);

  double current_yaw = tf2::getYaw(tf_msg.transform.rotation);

  // 计算最短旋转角度
  double diff = std::fmod(dist - current_yaw + M_PI, 2.0 * M_PI) - M_PI;
  goal_.target_yaw = diff;

  
  RCLCPP_INFO(this->node_->get_logger(),
              "SpinAction: target yaw = %.3f rad, angle diff = %.3f rad, currentyaw = %.3f rad",
              dist, diff, current_yaw);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
}
