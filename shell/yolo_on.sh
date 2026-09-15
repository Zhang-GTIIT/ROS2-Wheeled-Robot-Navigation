#!/bin/bash
 
# 激活conda环境
source /home/agv/anaconda3/etc/profile.d/conda.sh  # 根据您的conda安装路径调整
conda activate yolo  # 替换为您的conda环境名
 
# 设置ROS2环境
source /opt/ros/humble/setup.bash
source /home/agv/wzb/install/setup.bash


export PYTHON_EXECUTABLE=$(which python)
# 设置PYTHONPATH包含conda环境的库
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
 
# 运行节点
ros2 run image_saver detect &
