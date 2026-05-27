# robot_trajectory_planner

ROS 2 Python package for:
- 机械臂运动控制的轨迹规划层
- 用于生成特定任务下的机械臂末端工具坐标轨迹，包括位姿、速度、加速度（和力）

## Nodes
- `teleoperation_trajectory_node`：用于机械臂遥操作映射，包括位姿、速度、加速度
- `teleoperation_feedback_node`：用于机械臂遥操作映射，包括力反馈
- `scan_cylindrical_surface_node`：用于机械臂扫查圆柱表面，包括位姿
- `scan_raster_trajectory_node`：用于机械臂扫查平面表面，包括位姿             