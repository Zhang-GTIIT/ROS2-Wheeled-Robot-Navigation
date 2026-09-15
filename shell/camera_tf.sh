#!/bin/bash
 
# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash

# 运行节点
ros2 run tf2_ros static_transform_publisher 38 19 10 0 0 0 base_link camera_link
