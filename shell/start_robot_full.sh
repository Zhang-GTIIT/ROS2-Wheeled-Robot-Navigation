#!/bin/bash

# ========== 路径统一前缀 ==========
ROOT_DIR="/home/agv/wzb"

# ========== 可配置参数 ==========
MAP_YAML="$ROOT_DIR/map/scans5f2.yaml"
NAV2_PARAMS="$ROOT_DIR/nav2_params/nav2_params.yaml"
RVIV_CONFIG="$ROOT_DIR/nav2_default_view.rviz"
ICP_PARAMS="$ROOT_DIR/src/icp_localization_ros2/config/node_params5f2.yaml"

# CAN接口参数
CAN_INTERFACE="c"
CAN_BITRATE="c"

# 启动延迟
STARTUP_DELAY=4

# ROS2相关
USE_SIM_TIME="False"

# ========== 日志配置（核心修改点） ==========
LOG_BASE="$ROOT_DIR/ros2_logs"

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_RUN_DIR="$LOG_BASE/$TIMESTAMP" 

mkdir -p "$LOG_RUN_DIR"

LOG_RVIZ="$LOG_RUN_DIR/rviz.log"
LOG_NAV2="$LOG_RUN_DIR/nav2.log"
LOG_CAN="$LOG_RUN_DIR/can_core.log"
LOG_LIDAR="$LOG_RUN_DIR/lidar.log"
LOG_ICP="$LOG_RUN_DIR/icp.log"
LOG_MAP_LOADER="$LOG_RUN_DIR/map_loader.log"
LOG_MAPSERVER="$LOG_RUN_DIR/mapserver_lifecycle.log"
LOG_CAMERA="$LOG_RUN_DIR/camera.log"

echo "=== 机器人系统启动流程 ==="
echo "日志目录：$LOG_RUN_DIR"

# ================== 终端0：RViz ==================
gnome-terminal --title="RViz" -- bash -c "
echo '=== 终端0: RViz ===';
echo 'Log: $LOG_RVIZ';
ros2 run rviz2 rviz2 -d $RVIV_CONFIG \
  2>&1 | tee -a $LOG_RVIZ;
exec bash"

# ================== 终端1：Nav2 ==================
gnome-terminal --title="Nav2" -- bash -c "
echo '=== 终端1: Nav2 ===';
echo 'Log: $LOG_NAV2';
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=$USE_SIM_TIME \
  map:=$MAP_YAML \
  params_file:=$NAV2_PARAMS \
  2>&1 | tee -a $LOG_NAV2;
exec bash"

# ================== 终端2：CAN & ROS2 Core ==================
  # sudo ip link set $CAN_INTERFACE down 2>/dev/null;
  # sudo ip link set $CAN_INTERFACE type can bitrate $CAN_BITRATE;
  # sudo ip link set $CAN_INTERFACE up;
  # sudo ip link show $CAN_INTERFACE;
gnome-terminal --title="CAN & ROS2 Core" -- bash -c "
echo '=== 终端2: CAN & ROS2 Core ===';
echo 'Log: $LOG_CAN';
{
  sudo ip link set $CAN_INTERFACE down 2>/dev/null;
  sudo ip link set $CAN_INTERFACE type can bitrate $CAN_BITRATE;
  sudo ip link set $CAN_INTERFACE up;
  sudo ip link show $CAN_INTERFACE;
  echo '';
  ros2 launch ranger_bringup ranger_mini_v3.launch.py;
} 2>&1 | tee -a $LOG_CAN;
exec bash"

# ================== 终端3：激光雷达 ==================
gnome-terminal --title="Lidar Node" -- bash -c "
echo '=== 终端3: Lidar ===';
echo 'Log: $LOG_LIDAR';
ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
  2>&1 | tee -a $LOG_LIDAR;
exec bash"

# ================== 终端4：ICP ==================
gnome-terminal --title="ICP Localization" -- bash -c "
echo '=== 终端4: ICP ===';
echo 'Log: $LOG_ICP';
ros2 launch icp_localization_ros2 bringup.launch.py \
  node_params:=$ICP_PARAMS \
  2>&1 | tee -a $LOG_ICP;
exec bash"

# ================== 终端5：Map Loader ==================
gnome-terminal --title="Map Loader" -- bash -c "
echo '=== 终端5: Map Loader ===';
echo 'Log: $LOG_MAP_LOADER';
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  \"{map_url: $MAP_YAML}\" \
  2>&1 | tee -a $LOG_MAP_LOADER;
exec bash"

sleep $STARTUP_DELAY

# ================== 终端6：MapServer Lifecycle ==================
gnome-terminal --title="MapServer Lifecycle" -- bash -c "
echo '=== 终端6: MapServer Lifecycle ===';
echo 'Log: $LOG_MAPSERVER';
{
  ros2 lifecycle set /map_server configure;
  ros2 lifecycle set /map_server activate;
} 2>&1 | tee -a $LOG_MAPSERVER;
exec bash"

# ================== 终端7：Camera ==================
gnome-terminal --title="Camera Config" -- bash -c "
echo '=== 终端7: Camera ===';
echo 'Log: $LOG_CAMERA';
ros2 launch orbbec_camera gemini_330_series.launch.py \
  2>&1 | tee -a $LOG_CAMERA;
exec bash"

# ================== 终端7：Camera ==================
gnome-terminal --title="Camera Config" -- bash -c "
echo '=== 终端7: Camera ===';
echo 'Log: $LOG_CAMERA';
python3 /home/agv/wzb/src/image_saver/image_saver/pc_relay.py;
exec bash"

# ================== 终端8：发布 Camera TF ==================
gnome-terminal --title="Camera TF" -- bash -c "
echo '=== 终端8: Camera TF ===';
echo 'Log: $LOG_CAMERA';
ros2 run tf2_ros static_transform_publisher 0.28 0.0 0.05 0 0 0 base_link camera_link \
  2>&1 | tee -a $LOG_CAMERA;
exec bash"