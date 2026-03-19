## 此文件夹不是功能包，无需编译

# 文件说明

- `save_file_demo.cpp`：演示如何从 ROS2 消息中提取内容并保存到指定路径下的 CSV 文件。
  - 读取 CSV 文件中的目标点坐标（x, y, z），并逐点控制运动平台移动。
  - 每到一个目标点，订阅 `aimooe_coord` 话题，收集 2 秒内的测量数据。
  - 所有测量数据追加保存到 `/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/pf_control/save/1105_points_measurement.csv` 文件。
  - 代码结构清晰，适合参考如何实现消息提取与数据保存。

# 使用方法

1. 确保 `pf_control` 功能包已编译并可用。
2. 将 `save_file_demo.cpp` 作为参考或直接运行（需在有 ROS2 节点环境下）。
3. 修改 CSV 路径和保存路径以适配实际需求。
4. 运行后会在终端输出每个目标点的读取和数据保存情况。

# 典型流程

- 读取目标点 → 控制移动 → 订阅测量数据 → 保存到 CSV 文件

# 注意事项

- 本文件夹仅供代码示例和参考，不参与 colcon 编译。
- 如需集成到功能包，请将代码迁移到对应 ROS2 包下。
- 参考48-80行代码即可