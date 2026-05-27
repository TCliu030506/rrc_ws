# asm_description 速查版（联调一页清单）

适用场景：UR5 + ASM 在 RViz/Gazebo 中联调，出现不显示、不动、抖动、倒地等问题时快速排查。

## 1. 三步启动（最常用）

1. 编译并加载环境

```bash
colcon build --packages-select asm_description
source install/setup.bash
```

2. 启动 Gazebo 联合仿真

```bash
ros2 launch asm_description ur5_with_asm_gazebo.launch.py
```

3. 启动键盘控制

```bash
source install/setup.bash
ros2 run asm_description ur5_with_asm_keyboard_teleop.py
```

## 2. 快速健康检查（30 秒）

1. 控制器是否 active

```bash
ros2 control list_controllers
```

期望至少看到：
- joint_state_broadcaster active
- joint_trajectory_controller 或 scaled_joint_trajectory_controller active
- asm_arm_controller active

2. 动作接口是否存在

```bash
ros2 action list | grep follow_joint_trajectory
```

期望至少看到：
- /joint_trajectory_controller/follow_joint_trajectory 或 /scaled_joint_trajectory_controller/follow_joint_trajectory
- /asm_arm_controller/follow_joint_trajectory

3. 关节状态是否持续更新

```bash
ros2 topic echo /joint_states --once
```

## 3. 常见问题 -> 直接处理

1. 现象：一发指令就倒地/抖动
- 先确认控制脚本已从 /joint_states 初始化目标，不是从全零位起跳。
- 降低步长，增大轨迹时间（例如 0.8 s）。
- 检查初始姿态是否穿模或自碰撞。
- 检查惯量是否异常（过小、主惯量不合理、质心偏置异常）。

2. 现象：模型能显示但控制器不工作
- joints 名称与 URDF 不一致（含前缀不一致）最常见。
- 同一关节被多个 active 控制器抢占。
- spawner 启动顺序不对：先 joint_state_broadcaster，再轨迹控制器。

3. 现象：RViz 正常但 Gazebo 异常
- Gazebo 插件参数未指向正确 controllers.yaml。
- world 或模型资源未安装到 share 目录。
- Gazebo 与 display 启动参数不一致（前缀、装配位姿、use_sim_time）。

## 4. 调参建议（稳定优先）

1. 控制层
- 小步长增量控制：UR 关节 0.02~0.05 rad，ASM 关节 0.005~0.02 rad。
- time_from_start 建议先 0.6~1.0 s，再逐步加快。

2. 物理层
- 先简化 collision（盒/圆柱）验证稳定，再替换精细网格。
- 惯量参数优先真实测量或 CAD 导出，避免手填随意值。

## 5. 关键文件速查

- launch/ur5_with_asm_gazebo.launch.py
- launch/ur5_with_asm.launch.py
- config/ur5_with_asm_controllers.yaml
- urdf/ur5_with_asm.urdf.xacro
- urdf/asm_description_mount.xacro
- scripts/ur5_with_asm_keyboard_teleop.py

## 6. 一句话原则

先保证“状态连续”（从当前姿态起步），再追求“响应速度”（缩短轨迹时间）。
