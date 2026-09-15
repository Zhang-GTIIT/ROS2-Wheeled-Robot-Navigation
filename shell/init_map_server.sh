#!/bin/bash
 
# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash

ros2 lifecycle set /map_server configure 
ros2 lifecycle set /map_server activate 