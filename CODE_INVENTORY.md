# 代码清单

本清单对应 Ranger Mini V3 轮式移动机器人自主巡航项目。

## 自写/项目层代码

| 位置 | 内容 |
|---|---|
| `src/elevator_manager/` | ROS 2 电梯管理节点、等待区判断、启动文件与区域配置 |
| `src/image_saver/` | 相机图像保存、YOLO 分类检测、点云话题转发 |
| `src/FAST_LIO/` | FAST-LIO2 ROS 2 建图算法、IMU 预积分、迭代卡尔曼滤波、点云预处理和 ikd-Tree 增量地图 |
| `python_code/send_goal.py` | 通过 `NavigateThroughPoses` 发送多点巡航任务，并转换 Unity 坐标 |
| `python_code/all_task.py` | 从 JSON 读取并执行 Nav2 单目标任务 |
| `python_code/monitor.py` | Socket 接收任务并阻塞执行 Nav2 导航 |
| `python_code/nav2_action/set_goal.py` | 从任务 JSON 发送 `NavigateToPose` |
| `python_code/remote_api.py` | ROS 图像订阅与远程 VLA HTTP 接口调用 |
| `python_code/send_email.py` | 巡航通知邮件；整理版已去除硬编码凭据 |
| `python_code/openvla_ft_data/` | OpenVLA 数据采集脚本 |
| `python_code/piper_dataset_builder/` | 从 ROS bag 构建 Piper/TFDS 数据集（bag 本体未复制） |
| `python_code/convert_to_openvla_rlds.py` | 转换为 OpenVLA/RLDS 数据格式 |
| `bt/` | 多楼层巡航、地图切换、YOLO 与机械臂动作编排的行为树 |
| `shell/` | Ranger Mini V3 底盘启动、相机 TF、地图/ICP 切换、YOLO 开关 |
| `nav2_params/` | 多组 Nav2 参数配置 |
| `map/**/*.yaml` | 地图元数据；不含体积很大的 PGM/PCD 文件 |

## 基于上游修改的代码

| 位置 | 整理时检测到的变化 |
|---|---|
| `src/navigation2/` | 自定义行为树动作/条件节点、消息、Bringup 与导航树等；只复制涉及改动的 4 个 Nav2 包 |
| `src/icp_localization_ros2/` | ICP 参数、启动文件、质量消息、日志/TF 和定位实现改动 |
| `src/spatio_temporal_voxel_layer/` | 体素网格与图层实现改动 |

## 配置与样例数据

- `python_code/nav2_task/*.json`、`waypoints.json`、`tf_unity_to_real.json` 是任务/坐标配置样例。
- `nav2_default_view.rviz` 是 RViz 视图配置。
- `python_code/pos_tf.ipynb` 是坐标转换分析笔记本。

## 上传建议

当前整理文件夹可以作为 GitHub 仓库根目录。首次提交前建议重点复核硬编码的 `/home/agv/wzb` 路径、局域网地址、地图文件名与楼层坐标是否仍适用于部署机器。

FAST-LIO 源码不是从 `wzb.zip` 提取，而是根据日志确认所用组件后补充的上游 ROS 2 版本；若能从机器人取回 `/home/agv/fast_lio_ws/src/FAST_LIO/`，应再与此目录进行差异比较。
