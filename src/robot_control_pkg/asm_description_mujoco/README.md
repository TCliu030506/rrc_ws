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


## 2026-05-16 至 2026-05-20 更新调整
- asm_ros2_node.py 为多线程版本
- asm_ros2_node copy.py 为原始单线程版本

- 差异对比：
有，而且有几个会直接改变仿真/控制结果。最关键的差异不是线程本身，而是**控制命令怎么作用到 MuJoCo 状态**。

最可疑的功能差异：（Codex生成）

1. **笛卡尔目标 IK 不再直接改 `self.data.qpos`**
   旧版在 IK 里会把解算结果写回主仿真状态：  
   [asm_ros2_node copy.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node%20copy.py:420)

   新版 IK 用单独的 `self.ik_data`，只返回解，然后通过 actuator `ctrl` 去追踪：  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:728)  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:1021)

   这会让行为从“每个控制周期近似瞬移到 IK 解”变成“通过位置执行器动力学慢慢跟踪 IK 解”。在 `ultra_scanning_sim_mujoco.launch.py` 这种导纳控制发 `/cartesian_target_pose` 的闭环里，这个差异非常容易导致扫查轨迹、接触力、末端误差明显不同。

2. **`JointState` 的 position 控制语义变了**
   旧版 position 模式是直接写 `qpos`，然后 `mj_forward()`：  
   [asm_ros2_node copy.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node%20copy.py:622)

   新版 position 模式只是更新 actuator 目标，不直接改关节位置：  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:982)

   如果你的系统里还有节点发 `command_joint`，新版会更“物理真实”，但不会和旧版结果一致。

3. **发布数据来自状态快照，时间戳也变了**
   旧版发布时直接读当前 `self.data`，stamp 是发布时刻：  
   [asm_ros2_node copy.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node%20copy.py:703)

   新版仿真线程每步写 `SimState` 快照，发布线程读快照，stamp 是快照时刻：  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:458)  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:1036)

   这会带来一点延迟/采样相位差，对导纳控制这种闭环也可能有影响。

4. **相机发布行为变了**
   旧版相机图像跟着 `publish_rate_hz` 渲染发布，默认 50 Hz。新版新增 `render_rate_hz=15`，并且默认 `render_only_with_subscribers=True`：  
   [asm_ros2_node.py](/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/asm_description_mujoco/asm_description_mujoco/asm_ros2_node.py:110)

   如果你的测试依赖相机数据，新旧图像频率和丢帧行为会不同；如果只是力控/轨迹，这个不是主因。

我的判断：你现在结果明显不一样，**第一嫌疑是新版笛卡尔 IK 不再直接写 `qpos`**。旧版其实像“IK 位置投影 + 再仿真一步”，新版是“IK 生成位置执行器目标 + 动力学跟踪”。这两个控制器不是同一个系统。

如果你想让新版先复现旧结果，可以把笛卡尔分支临时改回：IK 成功后同时写 `self.data.qpos[:] = ik_solution`、`mj_forward()`，再设置 `ctrl`。这样能验证差异是不是主要来自这里。
