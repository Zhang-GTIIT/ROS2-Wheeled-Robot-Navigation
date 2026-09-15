# 第三方代码来源

本整理版保留了项目实际修改过的第三方代码，并移除了嵌套 `.git` 历史。公开发布时应继续保留各包自带的版权与许可证文件。

| 代码 | 上游 | 原压缩包分支 / 基准提交 | 本整理版策略 |
|---|---|---|---|
| FAST-LIO ROS 2 | https://github.com/Ericsii/FAST_LIO_ROS2 | `ros2` / `2fffc570a25d0df172720bac034fbdb6a13d2162` | 原 ZIP 没有源码；根据运行日志补充轻量工作树，并将 ikd-Tree 子模块源码直接纳入 |
| Navigation2 | https://github.com/ros-planning/navigation2 | `humble` / `c8c8aba2b26f2383aedc438552b603b489b0512c` | 只保留检测到定制内容的 `nav2_behavior_tree`、`nav2_bringup`、`nav2_bt_navigator`、`nav2_msgs` |
| ICP Localization ROS 2 | https://github.com/baiyeweiguang/icp_localization_ros2 | `main` / `e5062828fe1be8912873565f35537517a52ecffa` | 保留工作树，移除嵌套 `.git` |
| Spatio-Temporal Voxel Layer | https://github.com/SteveMacenski/spatio_temporal_voxel_layer | `humble` / `92896fe9ea994260bf34f1fa7b7735f3804cfc6f` | 保留工作树，移除嵌套 `.git` |

原 `navigation2` 克隆中其余未定制包没有重复复制；部署时可通过 ROS 2 Humble 二进制包或对应上游源码提供这些依赖。

FAST-LIO 使用 GPL-2.0 许可证；对应许可证文件已保留在 `src/FAST_LIO/LICENSE`。其运行依赖 `livox_ros_driver2`、PCL、Eigen 和 ROS 2 Humble。内置 ikd-Tree 取自 FAST-LIO 原子模块提交 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。
