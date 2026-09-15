#!/bin/bash

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/agv/wzb/map/scans1f4.yaml}" &
