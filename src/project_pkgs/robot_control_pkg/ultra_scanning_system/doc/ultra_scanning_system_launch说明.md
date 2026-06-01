# ultra_scanning_system.launch.py 简要说明

本文档说明真实 UR5 超声扫查系统的 launch 组成、关键话题、配置位置和调试方法。

## 1. 启动目标

`ultra_scanning_system.launch.py` 启动完整扫查控制链路：

- 硬件和 TF 启动组：UR driver、编码器、六维力传感器、ASM 工具 TF、`asm_ee_site` 状态发布。
- 工具动态重力补偿。
- 扫查名义轨迹源。
- 工具坐标系导纳控制。
- 一体化 RTDE servoL 位姿执行节点。

当前不再使用独立的 `asm_ee_command_transform_node` 和旧的 `rtde_servol_pose_controller_node`。一体化 servoL 节点会直接订阅 `/admittance/asm_ee_cmd_pose`，在节点内部完成 `base -> asm_ee_site` 到 `base -> tool0` 的 TF 转换，然后调用 servoL。

整体数据流：

```text
trajectory source
  -> /scan/desired_pose, /scan/desired_twist, /scan/desired_accel
  -> admittance_controller_node
  -> /admittance/asm_ee_cmd_pose
  -> rtde_servol_frame_pose_controller_node
  -> servoL
```

调试位姿流：

```text
rtde_servol_frame_pose_controller_node
  -> /arm_desired_pose_tool0
```

力控数据流：

```text
/external_force_torque_wrench
  -> dynamic_gravity_compensation_node
  -> /external_force_torque_wrench_compensated
  -> admittance_controller_node
```

## 2. Launch 分工

| 文件 | 作用 |
|---|---|
| `ultra_scanning_hardware_tf.launch.py` | 启动 UR driver、双编码器、力传感器、ASM 工具 TF、`asm_ee_site` 位姿/速度状态 |
| `ultra_scanning_system.launch.py` | 启动硬件组、重力补偿、轨迹源、导纳控制和一体化 servoL 执行 |

## 3. 配置方式

现在不再通过 `ros2 launch ... 参数:=值` 修改系统参数。调试时直接修改 launch 文件顶部的常量。

主系统参数位于 `ultra_scanning_system.launch.py` 顶部：

```python
GLOBAL_LOG_LEVEL = 'WARN'
ROBOT_IP = '192.168.1.102'
ENABLE_ADMITTANCE = True
TRAJECTORY_PLANNER = 'current_pose_hold'  # current_pose_hold, path_map, contact_scan
```

硬件和 TF 参数位于 `ultra_scanning_hardware_tf.launch.py` 顶部：

```python
UR_TYPE = 'ur5'
ROBOT_IP = '192.168.1.102'
FORCE_SENSOR_PORT = '/dev/ttyUSB0'
```

修改后需要重新编译或使用 symlink install：

```bash
colcon build --packages-select ultra_scanning_system --symlink-install
source install/setup.bash
```

## 4. 轨迹源选择

通过 `ultra_scanning_system.launch.py` 顶部的 `TRAJECTORY_PLANNER` 选择轨迹源。

| 值 | 节点 | 用途 |
|---|---|---|
| `current_pose_hold` | `current_pose_hold_node` | 保持当前位姿，适合调试 TF、力传感器、导纳和 servoL 链路 |
| `path_map` | `path_map_trajectory_node` | 直接播放 path-map 粗路径 |
| `contact_scan` | `contact_scan_trajectory_node` | 接近、预接触搜索、接触稳定、接触扫查、撤离的完整状态机 |

启动命令统一为：

```bash
ros2 launch ultra_scanning_system ultra_scanning_system.launch.py
```

如果要切换到接触扫查，先修改：

```python
TRAJECTORY_PLANNER = 'contact_scan'
```

再启动：

```bash
ros2 launch ultra_scanning_system ultra_scanning_system.launch.py
```

## 5. 关键坐标系

| 名称 | 含义 |
|---|---|
| `base` | 控制基坐标系 |
| `tool0` | UR 机械臂法兰/工具坐标系 |
| `asm_ee_site` | 实际受控末端点，通常对应探头接触控制点 |
| `asm_force_sensor_link` | 六维力传感器坐标系 |

导纳控制器输出的是 `base -> asm_ee_site` 目标位姿。一体化 servoL 节点会通过 TF 转成 `base -> tool0` 目标位姿。

## 6. 主要话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/external_force_torque_wrench` | `WrenchStamped` | 原始六维力传感器数据 |
| `/external_force_torque_wrench_compensated` | `WrenchStamped` | 动态重力补偿后的外力 |
| `/asm_ee_site/pose` | `Pose` | 受控末端实时位姿 |
| `/asm_ee_site/twist` | `Twist` | 受控末端实时速度 |
| `/scan/desired_pose` | `Pose` | 扫查名义位姿 |
| `/scan/desired_twist` | `Twist` | 扫查名义速度 |
| `/scan/desired_accel` | `Accel` | 扫查名义加速度 |
| `/arm_admittance_control/control_wrench` | `WrenchStamped` | 导纳控制目标 wrench |
| `/admittance/asm_ee_cmd_pose` | `Pose` | 导纳补偿后的 `asm_ee_site` 目标位姿 |
| `/arm_desired_pose_tool0` | `Pose` | 一体化 servoL 节点发布的 `tool0` 调试目标位姿 |
| `/contact_scan/state` | `String` | `contact_scan` 模式下的当前状态 |
| `/ur5/servol_verify` | `Float64MultiArray` | servoL 执行验证信息 |

## 7. path-map 参数

这些参数位于 `ultra_scanning_system.launch.py` 顶部：

| 常量 | 当前值 | 物理意义和调参影响 |
|---|---:|---|
| `PATH_MAP_PUBLISH_RATE` | `100.0` | 名义轨迹发布频率，单位 Hz。频率越高，轨迹点更新越密，导纳控制器收到的期望位姿越连续；过低会导致速度估计和 servoL 跟踪更粗糙。 |
| `PATH_MAP_MAX_LINEAR_SPEED` | `0.01` | 沿路径运动的最大末端线速度，单位 m/s。它决定 path-map 插值时每段路径最短执行时间；值越大扫查越快，但接触力波动和 servoL 跟踪误差通常也会增大。 |
| `PATH_MAP_MAX_ANGULAR_SPEED` | `0.05` | 沿路径运动的最大末端姿态角速度，单位 rad/s。路径点姿态变化较大时，该值决定姿态过渡速度；过大会使探头姿态变化激进，过小会拉长整段轨迹时间。 |
| `PATH_MAP_LOOP_PATH` | `False` | 是否循环播放 path-map 路径。`path_map` 直接播放模式可用；接触扫查通常应保持 `False`，避免扫查完成后重复压回工件。 |
| `PATH_MAP_ENABLE_RESAMPLING` | `True` | 是否对原始路径点进行重采样加密。启用后会限制相邻期望点的位置和姿态差，有利于 servoL 和导纳控制平滑跟踪。 |
| `PATH_MAP_MAX_LINEAR_STEP` | `0.001` | 重采样后相邻路径点最大位置间隔，单位 m。值越小路径越密、曲面跟踪更细，但点数增加；值越大路径更稀，可能带来插值误差或局部速度突变。 |
| `PATH_MAP_MAX_ANGULAR_STEP` | `0.01` | 重采样后相邻路径点最大姿态差，单位 rad。值越小姿态变化越平滑，适合曲面法向变化明显的路径；过大可能导致探头姿态分段变化明显。 |

当前默认路径文件为：

```text
robot_trajectory_planner/data/path_map_offset.txt
```

## 8. contact_scan 参数

这些参数位于 `ultra_scanning_system.launch.py` 顶部：

| 常量 | 当前值 | 物理意义和调参影响 |
|---|---:|---|
| `CONTACT_SCAN_FORCE_AXIS` | `z` | 用于接触判定和恒力控制的力轴。当前探头压紧主要体现在工具/导纳坐标系 Z 向，因此使用 `z`。 |
| `CONTACT_SCAN_FORCE_AXIS_SIGN` | `-1.0` | 力符号转换系数，用来把“压紧力增大”统一成状态机内部的正值。若传感器 Z 向压紧读数为负，例如 -15N 表示压紧 15N，则这里应设为 `-1.0`。 |
| `CONTACT_SCAN_APPROACH_AXIS_SIGN` | `1.0` | 预接触搜索方向相对工具 Z 轴的符号。它决定 PRE_CONTACT 阶段沿工具 +Z 还是 -Z 推进，也决定用接触搜索距离偏置整条曲面路径的方向。 |
| `CONTACT_SCAN_CONTACT_FORCE_THRESHOLD` | `5.0` | 初始接触判定阈值，单位 N。PRE_CONTACT 中法向压紧力达到该值后进入 CONTACT_SETTLE；值太小容易被噪声触发，值太大可能导致接触前压入过深。 |
| `CONTACT_SCAN_TARGET_CONTACT_FORCE` | `20.0` | CONTACT_SETTLE 和 CONTACT_SCAN 阶段的目标压紧力，单位 N。状态机内部始终使用正值表达压紧大小，实际发布到 wrench 时再乘以 `CONTACT_SCAN_FORCE_AXIS_SIGN`。 |
| `CONTACT_SCAN_FORCE_RAMP_RATE` | `0.2` | 目标压紧力变化率，单位 N/s。CONTACT_SETTLE 中从当前接触力平滑爬升到目标力；值越小越平稳但到达目标力越慢，值越大响应更快但更容易激发弹跳。 |
| `CONTACT_SCAN_ZERO_TORQUE_RX_RY_ENABLED` | `True` | 是否启用 rx/ry 零期望力矩贴合。启用后，CONTACT_SETTLE/CONTACT_SCAN 会把目标 `torque.x` 和 `torque.y` 设为指定值，默认 0 Nm，用于让探头绕 rx/ry 柔顺调整，减小倾斜接触力矩。 |
| `CONTACT_SCAN_TARGET_TORQUE_RX` | `0.0` | rx 方向目标力矩，单位 Nm。默认 0 表示希望绕 X 轴的接触力矩趋近零，从而改善法向贴合。 |
| `CONTACT_SCAN_TARGET_TORQUE_RY` | `0.0` | ry 方向目标力矩，单位 Nm。默认 0 表示希望绕 Y 轴的接触力矩趋近零，从而改善法向贴合。 |
| `CONTACT_SCAN_TORQUE_RAMP_RATE` | `0.05` | 目标力矩变化率，单位 Nm/s。它独立于 `CONTACT_SCAN_FORCE_RAMP_RATE`，用于让 rx/ry 力矩目标从 PRE_CONTACT 末端实测值平滑过渡到 0；值太大可能引起姿态突变，值太小则法向贴合调整较慢。 |
| `CONTACT_SCAN_SETTLE_DURATION` | `0.5` | CONTACT_SETTLE 中目标力达到且实测力稳定后继续保持的时间，单位 s。用于在正式扫查前让力和姿态收敛。 |
| `CONTACT_SCAN_SETTLE_FORCE_TOLERANCE` | `2.0` | CONTACT_SETTLE 中实测法向力相对目标力的允许误差，单位 N。误差进入该范围并持续 `CONTACT_SCAN_SETTLE_DURATION` 后才进入 CONTACT_SCAN。 |
| `CONTACT_SCAN_MAX_CONTACT_FORCE` | `50.0` | 最大安全接触力，单位 N。状态机内部法向压紧力超过该值会进入 FAULT，并锁定故障位姿。实机调试时应按探头和工件安全余量保守设置。 |
| `CONTACT_SCAN_MAX_SEARCH_DISTANCE` | `0.1` | PRE_CONTACT 最大搜索距离，单位 m。若沿接近方向搜索超过该距离仍未达到接触阈值，则进入 FAULT，避免路径或工件位置错误时持续下压。 |
| `CONTACT_SCAN_PRE_CONTACT_SPEED` | `0.0002` | PRE_CONTACT 接触搜索速度，单位 m/s。值越小接触建立越温和，但调试时间更长；值越大接触更快，但容易产生冲击力。 |
| `CONTACT_SCAN_RETRACT_DISTANCE` | `0.05` | 扫查完成后的撤离距离，单位 m。RETRACT 阶段沿接近方向反向退出该距离，用于让探头离开工件。 |
| `CONTACT_SCAN_STATE_TOPIC` | `/contact_scan/state` | 状态机当前状态发布话题。该话题只在初始状态和状态变化时发布，并使用 transient local QoS 保留最近状态。 |

## 9. servoL 参数

这些参数位于 `ultra_scanning_system.launch.py` 顶部：

| 常量 | 当前值 | 说明 |
|---|---:|---|
| `SERVO_ENABLE_DEBUG_POSE_PUBLISH` | `True` | 是否发布 `/arm_desired_pose_tool0` 调试位姿 |
| `SERVO_SPEED` | `0.15` | servoL speed 参数 |
| `SERVO_ACCELERATION` | `0.1` | servoL acceleration 参数 |
| `SERVO_LOOKAHEAD_TIME` | `0.1` | servoL lookahead_time 参数 |
| `SERVO_GAIN` | `300.0` | servoL gain 参数 |
| `SERVO_TF_LOOKUP_TIMEOUT_SEC` | `0.05` | TF 查询超时，单位 s |

## 10. 调试建议

- 首次实机测试建议使用 `TRAJECTORY_PLANNER = 'current_pose_hold'`。
- `contact_scan` 首次测试时应降低目标力、接触阈值和路径速度。
- 状态流为 `APPROACH -> PRE_CONTACT -> CONTACT_SETTLE -> CONTACT_SCAN -> RETRACT`。
- 运行中可用 `ros2 topic echo /contact_scan/state` 查看当前状态。
- `/contact_scan/state` 只在初始状态和状态切换时发布，使用 transient local QoS 保留最近一次状态。
- 确认 `/external_force_torque_wrench_compensated` 在自由空间接近零。
- 确认 `CONTACT_SCAN_FORCE_AXIS_SIGN` 配置后，探头压紧时法向力数值增大为正。
- 确认 `/admittance/asm_ee_cmd_pose` 和 `/arm_desired_pose_tool0` 不出现 `.nan`。
- 如果 `/arm_desired_pose_tool0` 与当前 TCP 姿态差异很大，应先停止实机运动，检查 TF 链和导纳输出。
- 如果接触后弹跳明显，优先降低 `CONTACT_SCAN_TARGET_CONTACT_FORCE`、`CONTACT_SCAN_FORCE_RAMP_RATE` 和路径速度。
