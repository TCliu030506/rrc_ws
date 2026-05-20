# BRT RS485 Encoder ROS 2 Topics

本文说明 `brt_rs485_encoder_node` 发布的各个话题分别表示什么量。

节点采用主动轮询方式读取编码器。哪些数据会被读取和发布，由
`config/brt_rs485_encoder.yaml` 中的 `publish_*` 参数决定。

## 话题总览

| 话题 | 消息类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `/encoder/joint_state` | `sensor_msgs/msg/JointState` | rad, rad/s | 按配置汇总出的关节状态 |
| `/encoder/position` | `std_msgs/msg/Float64` | rad | 单圈绝对角度 |
| `/encoder/position_raw_count` | `std_msgs/msg/Int64` | count | 单圈原始计数 |
| `/encoder/position2` | `std_msgs/msg/Float64` | rad | 宽位单圈绝对角度 |
| `/encoder/position2_raw_count` | `std_msgs/msg/Int64` | count | 宽位单圈原始计数 |
| `/encoder/multiturn` | `std_msgs/msg/Float64` | rad | 虚拟多圈累计角度 |
| `/encoder/multiturn_raw_count` | `std_msgs/msg/Int64` | count | 虚拟多圈累计原始计数 |
| `/encoder/turns` | `std_msgs/msg/Int64` | turn | 虚拟圈数 |
| `/encoder/speed16` | `std_msgs/msg/Float64` | rad/s | 16 位速度读数换算出的角速度 |
| `/encoder/speed16_raw_count` | `std_msgs/msg/Int64` | count/sample | 16 位速度原始增量计数 |
| `/encoder/speed32` | `std_msgs/msg/Float64` | rad/s | 32 位速度读数换算出的角速度 |
| `/encoder/speed32_raw_count` | `std_msgs/msg/Int64` | count/sample | 32 位速度原始增量计数 |

## 各话题说明

### `/encoder/joint_state`

`/encoder/joint_state` 是节点生成的标准关节状态消息，适合接入
`robot_state_publisher`、控制器或其他使用 `sensor_msgs/msg/JointState`
的 ROS 工具链。

它不是编码器协议里的单独寄存器，而是由下面两个参数指定来源：

```yaml
joint_state_position_source: position
joint_state_velocity_source: none
```

可选位置来源：

- `position`
- `position2`
- `multiturn`
- `none`

可选速度来源：

- `speed16`
- `speed32`
- `none`

例如：

```yaml
publish_joint_state: true
joint_state_position_source: position
joint_state_velocity_source: speed16
```

此时 `JointState.position[0]` 来自 `/encoder/position`，单位 rad；
`JointState.velocity[0]` 来自 `/encoder/speed16`，单位 rad/s。

### `/encoder/position`

单圈绝对角度，适合普通 16 bit 单圈读数。

对应底层读取：

- 读数类型：`position`
- 原始寄存器：单圈值寄存器
- 发布开关：`publish_position`
- 话题参数：`position_topic`

节点会把原始计数换算成弧度：

```text
angle_rad = raw_count * 2π / resolution
```

然后再应用零点和方向参数：

```text
published_rad =
  position_sign * (angle_rad - zero_offset_count * 2π / resolution)
  + angle_offset_rad
```

### `/encoder/position_raw_count`

`/encoder/position` 对应的原始单圈计数值，未换算成角度。

这个话题用于调试、标定零点、检查编码器原始输出是否正常。

相关参数：

```yaml
publish_raw_counts: true
position_raw_count_topic: encoder/position_raw_count
```

### `/encoder/position2`

宽位单圈绝对角度，适合 17 bit 或更高分辨率的单圈编码器读数。

对应底层读取：

- 读数类型：`position2`
- 原始寄存器：宽位单圈值寄存器组
- 发布开关：`publish_position2`
- 话题参数：`position2_topic`

单位和换算方式与 `/encoder/position` 相同，都是 rad。

### `/encoder/position2_raw_count`

`/encoder/position2` 对应的宽位单圈原始计数值。

如果你的编码器分辨率高于 16 bit，建议同时观察：

```bash
ros2 topic echo /encoder/position2
ros2 topic echo /encoder/position2_raw_count
```

### `/encoder/multiturn`

虚拟多圈累计角度，表示编码器累计旋转后的角度值。

对应底层读取：

- 读数类型：`multiturn`
- 发布开关：`publish_multiturn`
- 话题参数：`multiturn_topic`

单位为 rad。它和单圈角度不同：单圈角度通常限制在一圈范围内，
而虚拟多圈角度会随旋转圈数继续累计。

### `/encoder/multiturn_raw_count`

`/encoder/multiturn` 对应的虚拟多圈累计原始计数值。

这个值通常用于保存累计位置、检查多圈累计是否连续，以及做外部标定。

### `/encoder/turns`

虚拟圈数，表示编码器当前累计了多少圈。

对应底层读取：

- 读数类型：`turns`
- 发布开关：`publish_turns`
- 话题参数：`turns_topic`

消息类型为 `std_msgs/msg/Int64`。它发布的是整数圈数，不是角度。

### `/encoder/speed16`

16 位速度读数换算出的角速度。

对应底层读取：

- 读数类型：`speed16`
- 发布开关：`publish_speed16`
- 话题参数：`speed16_topic`

节点先把编码器返回的有符号 16 位增量计数换算为 RPM，再换算成 rad/s：

```text
rpm = delta_count * 60000 / (resolution * sample_time_ms)
rad_per_sec = rpm * 2π / 60
```

最后应用方向系数：

```text
published_rad_per_sec = position_sign * rad_per_sec
```

### `/encoder/speed16_raw_count`

`/encoder/speed16` 对应的原始速度增量计数。

注意它不是绝对位置计数，而是速度采样周期内的有符号增量计数。
采样周期由参数决定：

```yaml
sample_time_ms: 100
```

### `/encoder/speed32`

32 位速度读数换算出的角速度。

对应底层读取：

- 读数类型：`speed32`
- 发布开关：`publish_speed32`
- 话题参数：`speed32_topic`

它和 `/encoder/speed16` 的物理含义相同，都是 rad/s，但原始速度增量使用
32 位有符号数，适合更大速度范围或更高分辨率场景。

### `/encoder/speed32_raw_count`

`/encoder/speed32` 对应的原始 32 位速度增量计数。

同样，它表示采样周期内的有符号增量计数，不是绝对位置。

## 常用配置示例

只发布单圈角度和原始计数：

```yaml
publish_position: true
publish_raw_counts: true
publish_position2: false
publish_multiturn: false
publish_turns: false
publish_speed16: false
publish_speed32: false
```

发布单圈角度、速度，并合成 `JointState`：

```yaml
publish_joint_state: true
joint_state_position_source: position
joint_state_velocity_source: speed16

publish_position: true
publish_speed16: true
publish_raw_counts: true
```

发布所有可读内容：

```yaml
publish_position: true
publish_position2: true
publish_multiturn: true
publish_turns: true
publish_speed16: true
publish_speed32: true
publish_raw_counts: true
```

全部打开时，每个发布周期会连续进行多次 RS485 读寄存器操作。
如果通信不稳定或频率达不到预期，可以降低：

```yaml
publish_rate_hz: 5.0
```
