# asm_description

> 快速联调入口：见 QUICK_REF.md（现场排障一页清单）

## 1. 功能概述

asm_description 是一个面向 UR5 + 末端 ASM 自适应扫查机构的描述与仿真功能包，主要解决以下问题：

- 组合建模：将 UR5 与 ASM 机构在同一套 xacro/URDF 中完成装配。
- 可视化：支持在 RViz 中查看模型、TF 与关节姿态。
- 仿真：支持在 Gazebo 中加载组合机器人并通过 ros2_control 控制。
- 控制对接：提供 JointTrajectoryController 配置与键盘测试节点，便于快速验证控制链路。

---

## 2. 包内容说明

### 2.1 目录结构

- urdf/: 机器人模型与宏定义
  - ur5_with_asm.urdf.xacro：UR5 与 ASM 的组合模型入口
  - asm_description_mount.xacro：ASM 机构本体及安装关系
- launch/: 启动文件
  - ur5_with_asm.launch.py：组合模型 + RViz 显示
  - ur5_with_asm_gazebo.launch.py：Gazebo 仿真与控制器加载
  - gazebo.launch.py / load_urdf_into_gazebo.launch.py：Gazebo 相关加载流程
- config/: 控制器参数
  - ur5_with_asm_controllers.yaml：UR5 + ASM 联合控制配置
  - ros2_controllers.yaml：ASM 两关节控制配置示例
- scripts/: 测试控制脚本
  - ur5_with_asm_keyboard_teleop.py：键盘发送 FollowJointTrajectory 目标
  - arm_keyboard_teleop.py：ASM 控制脚本
- rviz/: RViz 配置
- worlds/: Gazebo 场景
- meshes/、textures/: 模型资源

### 2.2 关键能力

- 通过 xacro 参数实现安装位姿可配置：asm_parent_link、asm_mount_xyz、asm_mount_rpy。
- 在仿真中通过 joint_state_broadcaster + joint_trajectory_controller 驱动关节。
- 支持 UR 控制器候选动作接口自动选择：
  - /joint_trajectory_controller/follow_joint_trajectory
  - /scaled_joint_trajectory_controller/follow_joint_trajectory

---

## 3. 使用方式

## 3.1 编译

在工作空间根目录执行：

```bash
colcon build --packages-select asm_description
source install/setup.bash
```

## 3.2 RViz 可视化（不进 Gazebo）

```bash
ros2 launch asm_description ur5_with_asm.launch.py
```

可选参数示例：

```bash
ros2 launch asm_description ur5_with_asm.launch.py ur_type:=ur5 asm_parent_link:=tool0 asm_mount_xyz:="0 0 0" asm_mount_rpy:="0 0 0"
```

## 3.3 Gazebo 仿真

```bash
ros2 launch asm_description ur5_with_asm_gazebo.launch.py
```

常用参数示例：

```bash
ros2 launch asm_description ur5_with_asm_gazebo.launch.py ur_type:=ur5 controllers_file:=ur5_with_asm_controllers.yaml launch_rviz:=true gazebo_gui:=true
```

## 3.4 键盘测试控制

先启动 Gazebo，再开新终端执行：

```bash
source install/setup.bash
ros2 run asm_description ur5_with_asm_keyboard_teleop.py
```

脚本说明：

- 启动时会等待控制器动作服务可用。
- 启动后先从 /joint_states 同步当前姿态，再进行增量控制。
- 每次按键发送单点轨迹目标，默认轨迹时间为 0.8 s。

---

## 4. 开发经验沉淀（重点）

以下经验来自本包在 UR5 + ASM 联调中的实际问题复盘，建议作为后续新机器人开发的基线清单。

### 4.1 架构与开发顺序

推荐分阶段开发，避免一次性耦合过多问题：

1. 先做模型：只在 RViz 验证 URDF/xacro、TF、关节方向与限位。
2. 再接控制：接入 ros2_control 与 controllers.yaml，先在 fake/mock 或简化环境验证。
3. 最后进 Gazebo：逐步加入碰撞、惯量、摩擦，观察稳定性。

这样做可以把“显示问题”“控制问题”“物理问题”分离排查。

其中，注意，自行创建的URDF文件修改成.xacro格式时，需要更改meshes文件的加载路径：

<meshfilename="**file://$(find asm_description)/meshes/base_link.STL**"/>

### 4.2 控制器配置经验

- joints 名称必须与 URDF 完全一致（包括前缀）。
- 同一关节避免被多个 active 控制器抢占。
- 启动顺序建议：joint_state_broadcaster -> 轨迹控制器。
- 对组合机器人可采用分控制器策略：UR 主臂控制器 + ASM 末端控制器。

### 4.3 防止 Gazebo 抖动/倒地的关键措施

1. 目标初始化

- 不要把目标关节初值硬编码为全 0。
- 应在控制节点启动后先订阅 /joint_states，并把当前姿态作为初始目标。
- 这样可避免第一条指令发生大幅“跳到零位”。

2. 轨迹平滑

- 减小每次增量步长。
- 适当增大 time_from_start（例如 0.8 s），降低激励。

3. 物理参数

- 惯量参数必须合理且与几何一致。
- 关节 effort/velocity/limit 需与控制目标匹配。
- 初始姿态避免自碰撞或与地面/工件穿插。

4. 碰撞模型

- 早期建议使用简化 collision 几何体，确认稳定后再替换精细网格。

### 4.4 启动与资源管理经验

- Gazebo 中使用 world 文件时，要确保 worlds 已安装到 share 目录。
- spawn_entity 读取 robot_description 时，注意 XML 头处理与字符串格式。
- display 与 gazebo 两条启动链路建议参数对齐，减少行为差异。

### 4.5 调试清单（建议每次联调前快速过一遍）

1. 模型层

- TF 树是否连通。
- 关节轴方向、限位是否符合机械结构。

2. 控制层

- 控制器是否全部 active。
- action server 名称是否与脚本一致。
- joint_state 是否持续更新。

3. 物理层

- 是否出现初始穿模。
- 惯量、阻尼、摩擦是否异常。
- 首条控制命令是否过于激进。


### 4.6 力传感器配置经验

- 调试力传感器是否正常工作，可在rviz下进行，图像化直观且方便
- 注意实际控制时不要让关节运动到关节限位，否则力传感器会有异常值。
- 
---

## 5. 与本包相关的关键文件

- launch/ur5_with_asm.launch.py
- launch/ur5_with_asm_gazebo.launch.py
- config/ur5_with_asm_controllers.yaml
- config/ros2_controllers.yaml
- urdf/ur5_with_asm.urdf.xacro
- urdf/asm_description_mount.xacro
- scripts/ur5_with_asm_keyboard_teleop.py

---

## 6. 后续扩展建议

如果后续设计新机器人，建议在本包经验基础上复用如下模板：

- 新建 description 包：模型 + xacro 参数化。
- 新建 bringup 包：launch + controllers + 仿真启动编排。
- 保持统一关节命名规范，确保 RViz、控制器、脚本三处一致。
- 在控制脚本中默认加入“从 /joint_states 初始化目标”的保护逻辑。
