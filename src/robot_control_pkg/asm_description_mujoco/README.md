# asm_description_mujoco

一个基于 MuJoCo + ROS 2 的 UR5 + ASM 仿真功能包。

## 功能

- 加载 MuJoCo 模型并启动仿真节点
- 发布关节状态、末端位姿和传感器数据
- 支持关节控制
- 支持笛卡尔末端位姿控制
- 支持键盘测试节点

## 启动

先构建工作空间：

```bash
colcon build --packages-select asm_description_mujoco
source install/setup.bash
```

启动仿真节点：

```bash
ros2 launch asm_description_mujoco asm_ros2_node.launch.py
```

## 键盘测试

关节模式只控制 UR5 的 6 个关节，ASM tool 两个关节已改为被动关节。

关节模式：

```bash
ros2 run asm_description_mujoco asm_keyboard_teleop --mode joint
```

笛卡尔模式：

```bash
ros2 run asm_description_mujoco asm_keyboard_teleop --mode cartesian
```

## 常用话题

- `/joint_states`
- `/end_effector_pose`
- `/command_joint`
- `/cartesian_target_pose`
- `/sensors/*`

## 说明

- 关节模式默认使用当前 `/joint_states` 初始化目标。
- 笛卡尔模式默认使用当前 `/end_effector_pose` 初始化目标。
- 键盘节点只做最基础的测试控制，适合快速验证仿真通信链路。
