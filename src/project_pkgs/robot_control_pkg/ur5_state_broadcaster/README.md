# ur5_state_broadcaster

`ur5_state_broadcaster` 提供 UR5 末端状态相关的辅助节点，用于将已有位姿或
TF 信息转换为导纳控制、重力/惯性补偿等模块更方便使用的 ROS2 标准消息。

## 节点

### tcp_twist_estimator

`tcp_twist_estimator` 订阅 `PoseStamped`，发布：

- `geometry_msgs/msg/Pose`
- `geometry_msgs/msg/Twist`

该节点主要用于兼容已有只需要非 stamped `Pose` / `Twist` 的控制链路。

默认启动：

```bash
ros2 launch ur5_state_broadcaster tcp_twist_estimator.launch.py
```

常用参数：

```text
input_pose_topic   默认 /tcp_pose_broadcaster/pose
output_pose_topic  默认 /UR5/ee_pose
output_twist_topic 默认 /UR5/ee_twist
min_dt             默认 0.0001
max_angular_speed  默认 10.0
```

### frame_motion_from_tf

`frame_motion_from_tf` 从 TF 中实时读取 `source_frame -> target_frame`，并通过
位姿差分估计目标坐标系的速度和加速度。

默认输出：

- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/TwistStamped`
- `geometry_msgs/msg/AccelStamped`

默认配置：

```text
source_frame: base
target_frame: tool0
publish_rate: 125.0 Hz
express_in_target_frame: true
```

当 `express_in_target_frame` 为 `true` 时，发布的速度和加速度会表达在
`target_frame` 中。这适合后续做工具/力传感器坐标系下的惯性力补偿。

默认启动：

```bash
ros2 launch ur5_state_broadcaster frame_motion_from_tf.launch.py
```

用于力传感器坐标系 `asm_force_sensor_link`：

```bash
ros2 launch ur5_state_broadcaster frame_motion_from_tf.launch.py \
  target_frame:=asm_force_sensor_link \
  output_pose_topic:=/asm_force_sensor_link/pose \
  output_twist_topic:=/asm_force_sensor_link/twist \
  output_accel_topic:=/asm_force_sensor_link/accel
```

## frame_motion_from_tf 参数

```text
source_frame              TF 源坐标系，默认 base
target_frame              TF 目标坐标系，默认 tool0
output_pose_topic         PoseStamped 输出话题，默认 /tool0/pose
output_twist_topic        TwistStamped 输出话题，默认 /tool0/twist
output_accel_topic        AccelStamped 输出话题，默认 /tool0/accel
publish_rate              发布频率，默认 125.0 Hz
tf_lookup_timeout_sec     TF 查询超时时间，默认 0.01 s
express_in_target_frame   是否将 twist/accel 表达到 target_frame，默认 true
min_dt                    最小有效差分时间，默认 0.0001 s
max_dt                    最大有效差分时间，默认 0.2 s
velocity_filter_tau       速度一阶低通时间常数，默认 0.02 s
accel_filter_tau          加速度一阶低通时间常数，默认 0.05 s
max_linear_speed          线速度限幅，默认 5.0 m/s
max_angular_speed         角速度限幅，默认 20.0 rad/s
max_linear_accel          线加速度限幅，默认 30.0 m/s^2
max_angular_accel         角加速度限幅，默认 80.0 rad/s^2
```

## 输出消息语义

`frame_motion_from_tf` 发布的 `PoseStamped` 始终表示：

```text
source_frame -> target_frame
```

如果 `express_in_target_frame=true`，则：

```text
TwistStamped.header.frame_id = target_frame
AccelStamped.header.frame_id = target_frame
```

对应量为：

```text
v_target^target
omega_target^target
a_target^target
alpha_target^target
```

如果 `express_in_target_frame=false`，则 twist 和 accel 表达在 `source_frame` 中。

## 验证

运行数学单元测试：

```bash
PYTHONPATH=src/project_pkgs/robot_control_pkg/ur5_state_broadcaster \
python3 -m pytest src/project_pkgs/robot_control_pkg/ur5_state_broadcaster/test/test_frame_motion_estimator_math.py -q
```

构建包：

```bash
colcon build --packages-select ur5_state_broadcaster
```

查看 launch 参数：

```bash
source install/setup.bash
ros2 launch ur5_state_broadcaster frame_motion_from_tf.launch.py --show-args
```

## 注意事项

- 加速度由速度差分得到，天然比速度更容易受噪声影响，因此节点提供低通滤波和限幅。
- 如果 TF 短时间不可用，节点会重置估计器，避免跨越异常时间间隔计算错误加速度。
- 如果用于惯性力补偿，推荐保持 `express_in_target_frame=true`，并将
  `target_frame` 设置为力传感器坐标系，例如 `asm_force_sensor_link`。

