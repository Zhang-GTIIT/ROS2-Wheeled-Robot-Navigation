#!/bin/bash

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash

pkill -f /home/agv/wzb/install/icp_localization_ros2

ros2 launch icp_localization_ros2 bringup.launch.py node_params:=/home/agv/wzb/src/icp_localization_ros2/config/node_params4f.yaml &
