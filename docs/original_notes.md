# cam launch
cd orb_ws
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py

# 停止CAN接口（如果正在运行）
sudo ip link set can0 down
# 设置CAN参数（比特率500000）
sudo ip link set can0 type can bitrate 500000
# 启动CAN接口
sudo ip link set can0 up
# 验证接口状态
ip link show can0



ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 launch ranger_bringup ranger_mini_v3.launch.py
# rviz2
ros2 run rviz2 rviz2 -d /home/agv/wzb/nav2_default_view.rviz
# nav2 w/o amcl
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/home/agv/wzb/map/scans5f.yaml params_file:=/home/agv/wzb/nav2_params.yaml
# icp
ros2 launch icp_localization_ros2 bringup.launch.py node_params:=/home/agv/wzb/src/icp_localization_ros2/config/node_params5f.yaml

# mapservice
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/agv/wzb/map/scans5f.yaml}"
ros2 lifecycle set /map_server configure 
ros2 lifecycle set /map_server activate 
