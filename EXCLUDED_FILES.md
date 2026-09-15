# 未复制内容

原压缩包约 1.83 GB，解压后约 5.22 GB、36,504 个文件。本整理版有意排除了以下内容；它们仍完整保留在原始 `wzb.zip` 中。

- `build/`、`install/`、`log/`：colcon/CMake 编译产物与构建日志。
- `ros2_logs/`、`my_log/`、`shell/Log/`：运行日志和相机 SDK 日志。
- `saved_images/`：约 580 MB 的采集图片。
- `map/**/*.pcd`、`map/**/*.pgm`：约 2.38 GB 的点云与栅格地图；只保留对应 YAML 元数据。
- `python_code/**/rosbag/`：约 140 MB 的 ROS 2 bag 数据库。
- `best.pt`：YOLO 模型权重。
- `raw_data/`、`received_tasks/`：运行生成的数据与接收任务记录。
- `navigation2/.git` 等嵌套 Git 历史，以及 Nav2 中未检测到项目修改的上游包。
- 第三方包中的演示 GIF、截图、样例点云和栅格图；这些资料可从对应上游仓库获取。
- 各类 `.o`、`.so`、`.a`、`.pyc` 和缓存文件。

如需共享地图、模型或 bag，建议使用 GitHub Releases、对象存储或 Git LFS，并在仓库中记录下载位置与校验值。
