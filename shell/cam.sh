#!/bin/bash
 
# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash

# 运行节点
ros2 launch orbbec_camera gemini_330_series.launch.py &
