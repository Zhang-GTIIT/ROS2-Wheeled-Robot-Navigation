# 源码来源说明

`wzb.zip` 中没有 FAST-LIO 工作区源码，但建图日志显示机器人运行的是：

```text
/home/agv/fast_lio_ws/src/FAST_LIO/
executable: fastlio_mapping
```

本目录因此补充了与日志结构匹配的 ROS 2 上游实现：

- 仓库：`https://github.com/Ericsii/FAST_LIO_ROS2`
- 分支：`ros2`
- 提交：`2fffc570a25d0df172720bac034fbdb6a13d2162`
- ikd-Tree：`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`

只保留编译和阅读算法所需的源码、配置、启动文件、RViz 配置、README 与许可证；没有复制上游 PCD、Log、文档媒体或 Git 历史。

注意：这不是从机器人原 `/home/agv/fast_lio_ws` 取回的工作树，无法证明其中是否还有项目自定义改动。若之后能访问机器人，请用该原目录与本目录做 `git diff --no-index` 或其他目录差异比较。
