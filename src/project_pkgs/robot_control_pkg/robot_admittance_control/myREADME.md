# arm_admittance_controller

Admittance control law to generate desired motion of an end-effector (twist), given a desired control wrench and external wrench for a robotic arm that is not torque-controlled (i.e. velocity or position controlled). 

Such a controller is necessary to use impedance-control-like laws and provide compliant human-robot-interaction when the  velocity/position controlled robot arm is equipped with an external force/torque sensor.

* The current implementation has been tested on a UR10 **velocity-controlled robot** equipped with a robotiq FT 300 force torque sensor. The arm is part of the Robbie Yuri robot of the Interactive Robotics Group (IRG), MIT which is an older version of the [Care-O-Bot](http://www.care-o-bot.org) platform series.

* The current implementation is being tested on a Mitsubishi **position-controlled robot** equipped with a Mitsubishi force torque sensor. The robot is part of the MIT-Mitsubishi collaboration.

**Disclaimer**: Parts of this code was originally forked from [ridgeback_ur5_controller](https://github.com/epfl-lasa/ridgeback_ur5_controller) which is a repo used to control a Clearpath robotic platform with a UR5 from the LASA laboratory at EPFL. Plenty modifications have been made in order to work with a standalone robotic arm.

## Installation
...

## Usage
...

## Contact
**Maintainer**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT mit dot edu)

## License
This package is licensed under MIT license.

# admittance-controller_UR5
UR5 with FT-300 robotiq force sensor(platform) to implement admittance control

codes partly from https://github.com/nbfigueroa/robot_admittance_controller

---

# 项目说明（中文）

## 1. 项目简介

`robot_admittance_control` 是一个基于 ROS2 的机械臂导纳控制功能包，核心目标是：

- 在非力矩控制型机械臂（速度/位置控制接口）上实现柔顺交互；
- 将外部力/力矩传感器数据与控制期望力融合，计算末端期望速度（Twist）；
- 通过导纳动力学（质量-阻尼模型）生成平滑、受限、可控的运动指令。

本仓库已完成从 ROS1 到 ROS2 的迁移，当前使用 `ament_cmake`、`rclcpp`、`tf2_ros`、ROS2 Python Launch 体系。

---

## 2. 功能特性

- ROS2 C++ 导纳控制主节点（`admittance_controller_node`）
- ROS2 C++ 力传感器监听/调试节点（`read_force_node`）
- ROS2 Python TCP 力数据发布节点（`readForceSensor.py`）
- 多套参数文件（普通/UR10/实机）
- ROS2 启动文件（`.launch.py`）统一管理
- 内嵌 UR 描述资源（`ur_description`）与可视化启动

---

## 3. 目录结构说明

关键目录：

- `src/`：核心 C++ 源码
	- `admittance_controller_node.cpp`
	- `AdmittanceController.cpp`
	- `readForce.cpp`
- `include/robot_admittance_control/`：头文件
- `config/`：参数配置（ROS2 YAML）
- `launch/`：主功能包启动文件（ROS2）
- `Scripts/`：Python 节点（ROS2）
- `ur_description/`：UR 相关模型、xacro、可视化启动

---

## 4. 依赖环境

建议环境：

- Ubuntu + ROS2（Humble/Jazzy 同类发行版）
- colcon
- 常见依赖：
	- `rclcpp`
	- `rclpy`
	- `geometry_msgs`
	- `std_msgs`
	- `std_srvs`
	- `tf2`, `tf2_ros`, `tf2_eigen`
	- `xacro`
	- `robot_state_publisher`
	- `joint_state_publisher_gui`
	- `rviz2`

---

## 5. 编译方法

在工作空间根目录执行：

1. 构建
	 - `colcon build --packages-select robot_admittance_control --symlink-install`
2. 加载环境
	 - `source install/setup.bash`

---

## 6. 运行方式

### 6.1 导纳控制主节点（推荐）

- 启动默认导纳控制：
	- `ros2 launch robot_admittance_control admittance_controller.launch.py`

- 启动 UR10 参数版本：
	- `ros2 launch robot_admittance_control ur10_admittance_controller.launch.py`

- 启动 UR10 实机参数版本：
	- `ros2 launch robot_admittance_control ur10_admittance_controller_real.launch.py`

### 6.2 力数据相关节点

- 启动力相关节点组合：
	- `ros2 launch robot_admittance_control read_force.launch.py`

### 6.3 机器人模型与可视化

- 启动 UR5 FT 模型与 RViz2：
	- `ros2 launch robot_admittance_control view_ur5_ft.launch.py`

---

## 7. 参数说明

主要参数（在 `config/*.yaml`）：

- `topic_arm_pose`：末端位姿输入话题
- `topic_arm_twist`：末端速度输入话题
- `topic_external_wrench`：外部力/力矩输入话题
- `topic_control_wrench`：控制器给定力/力矩话题
- `topic_arm_command`：机械臂期望速度输出话题
- `mass_arm`：导纳质量矩阵（6x6，按行展开 36 元素）
- `damping_arm`：导纳阻尼矩阵（6x6，按行展开 36 元素）
- `workspace_limits`：工作空间约束 `[x_min, x_max, y_min, y_max, z_min, z_max]`
- `arm_max_vel`：线速度上限
- `arm_max_acc`：线加速度上限

说明：

- `mass_arm` 与 `damping_arm` 元素数量必须为 36；
- `workspace_limits` 元素数量必须为 6；
- 若参数长度不正确，节点会报错并退出。

---

## 8. 话题接口（核心）

输入（订阅）：

- 末端位姿：`geometry_msgs/msg/Pose`
- 末端速度：`geometry_msgs/msg/Twist`
- 外力数据：`geometry_msgs/msg/WrenchStamped`
- 控制力：`geometry_msgs/msg/WrenchStamped`

输出（发布）：

- 末端期望速度：`geometry_msgs/msg/Twist`

---

## 9. 算法简述

导纳动力学采用经典形式：

$$
\mathbf{M}\dot{\mathbf{v}} = -\mathbf{D}\mathbf{v} + \mathbf{F}_{ext} + \mathbf{F}_{ctrl}
$$

其中：

- $\mathbf{M}$：质量矩阵
- $\mathbf{D}$：阻尼矩阵
- $\mathbf{v}$：末端速度（Twist）
- $\mathbf{F}_{ext}$：传感器外力
- $\mathbf{F}_{ctrl}$：控制期望力

实现中还加入了：

- 加速度限幅
- 速度限幅
- 工作空间约束与越界回退策略

---

## 10. 从 ROS1 迁移状态

已完成：

- 构建系统：`catkin` → `ament_cmake`
- 通信 API：`roscpp/rospy` → `rclcpp/rclpy`
- TF：`tf` → `tf2_ros`
- 启动：`.launch` XML → `.launch.py`

已清理：

- 旧 ROS1 `.launch` 文件已删除。

---

## 11. 常见问题

### Q1：节点启动后无运动输出？

请检查：

- 位姿/速度/力话题是否有数据；
- 话题名是否与参数一致；
- TF 是否存在（如 `base_link`、`arm_base_link`、`robotiq_force_torque_frame_id`）。

### Q2：参数改了但不生效？

- 确认 launch 中使用的是目标参数文件；
- 重新 source：`source install/setup.bash`。

### Q3：模型显示异常或缺失？

- 确认 xacro 路径与 package 资源路径可解析；
- 确认在工作空间内重新构建后再启动 RViz2。

---

## 12. 后续建议

- 将控制器进一步组件化为 ROS2 Lifecycle Node；
- 增加单元测试与参数校验测试；
- 对接 ros2_control 统一控制接口；
- 在 README 中补充实机标定与调参流程（建议分章节记录）。

---

## 13. 维护说明

如需继续扩展，请优先基于以下文件：

- [src/robot_control_pkg/robot_admittance_control/CMakeLists.txt](src/robot_control_pkg/robot_admittance_control/CMakeLists.txt)
- [src/robot_control_pkg/robot_admittance_control/package.xml](src/robot_control_pkg/robot_admittance_control/package.xml)
- [src/robot_control_pkg/robot_admittance_control/launch](src/robot_control_pkg/robot_admittance_control/launch)
- [src/robot_control_pkg/robot_admittance_control/config](src/robot_control_pkg/robot_admittance_control/config)

---

## 新增功能：三轴力传感器消息转发节点

本包新增 three_axis_force_relay 节点，用于将 `force_sensor_msg::msg::ThreeAxisForce` 格式的三轴力传感器消息转为 `geometry_msgs::msg::WrenchStamped`，便于导纳控制模块直接复用。

### 启动方法

```bash
ros2 launch robot_admittance_control three_axis_force_relay.launch.py
```

- 默认订阅话题：`/forcesensor_pub`（类型：ThreeAxisForce）
- 默认发布话题：`/wrench_stamped`（类型：WrenchStamped）

如需自定义话题名，可通过 launch 文件 remapping。


