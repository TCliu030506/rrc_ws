# path_map_trajectory_node 说明文档

## 1. 节点用途

`path_map_trajectory_node` 是 `robot_trajectory_planner` 包中的点云路径轨迹发布节点。它读取 `path_map.txt` 中的离散目标位姿，并把这些点转换为导纳控制器需要的连续期望轨迹：

- `geometry_msgs/msg/Pose`
- `geometry_msgs/msg/Twist`
- `geometry_msgs/msg/Accel`

在 `ultra_scanning_system.launch.py` 中选择：

```bash
ros2 launch ultra_scanning_system ultra_scanning_system.launch.py \
  trajectory_planner:=path_map
```

即可用该节点替代 `current_pose_hold_node` 作为 `/scan/desired_*` 的轨迹源。

## 2. 输入输出接口

### 订阅

| 话题 | 类型 | 作用 |
|---|---|---|
| `/asm_ee_site/pose` | `geometry_msgs/msg/Pose` | 当前受控末端 `asm_ee_site` 在控制基坐标系下的位姿 |

节点启动后不会立即发布运动轨迹，而是先等待该当前位姿。收到第一帧当前位姿后，节点会把它作为起始点，并平滑移动到 `path_map.txt` 的第一个目标点。

### 发布

| 话题 | 类型 | 作用 |
|---|---|---|
| `/scan/desired_pose` | `geometry_msgs/msg/Pose` | 导纳控制目标位姿 |
| `/scan/desired_twist` | `geometry_msgs/msg/Twist` | 导纳控制目标速度 |
| `/scan/desired_accel` | `geometry_msgs/msg/Accel` | 导纳控制目标加速度 |

当前实现中，`desired_accel` 始终发布 0。这是有意设计：点云路径文件只包含离散位姿，没有可靠的加速度信息，直接差分二阶量容易引入噪声。

## 3. path_map.txt 格式

默认路径文件安装在：

```text
robot_trajectory_planner/share/robot_trajectory_planner/data/path_map.txt
```

源码中的默认文件位置是：

```text
src/project_pkgs/robot_control_pkg/robot_trajectory_planner/data/path_map.txt
```

每行格式：

```text
x y z rx ry rz
```

含义：

- `x y z`：目标位置，单位为 m。
- `rx ry rz`：rotation vector 姿态，单位为 rad。

示例：

```text
0.525609 -0.0555979 0.269315 3.07638 -0.00757589 -0.295333
```

空行和以 `#` 开头的注释行会被忽略。其他非空行必须严格包含 6 个数，否则节点会在启动时抛出错误。

## 4. 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `current_pose_topic` | `/asm_ee_site/pose` | 当前位姿输入 |
| `topic_desired_pose` | `/scan/desired_pose` | 期望位姿输出 |
| `topic_desired_twist` | `/scan/desired_twist` | 期望速度输出 |
| `topic_desired_accel` | `/scan/desired_accel` | 期望加速度输出 |
| `path_file` | 空字符串 | 路径文件。为空时读取包内默认 `data/path_map.txt` |
| `publish_rate` | `50.0` | 轨迹发布频率，单位 Hz |
| `max_linear_speed` | `0.005` | 最大线速度，单位 m/s |
| `max_angular_speed` | `0.05` | 最大角速度，单位 rad/s |
| `min_segment_duration` | `0.0` | 每段轨迹的最短持续时间，单位 s |
| `loop_path` | `false` | 到达最后一个路径点后是否循环回第一个点 |
| `enable_path_resampling` | `true` | 发布前是否先对文件路径点做加密重采样 |
| `max_path_linear_step` | `0.002` | 重采样后相邻内部路径点的最大位置间隔，单位 m |
| `max_path_angular_step` | `0.02` | 重采样后相邻内部路径点的最大姿态间隔，单位 rad |

指定点云包生成路径的示例：

```bash
ros2 launch ultra_scanning_system ultra_scanning_system.launch.py \
  trajectory_planner:=path_map \
  path_map_file:=/home/liutiancheng/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/pointcloudslam_cpp/data/path_planning/path_map.txt
```

## 5. 程序结构

该功能分成两层：

### 5.1 ROS 节点层

文件：

```text
robot_trajectory_planner/path_map_trajectory_node.py
```

职责：

1. 声明和读取 ROS 参数。
2. 定位并读取 `path_map.txt`。
3. 订阅当前 `/asm_ee_site/pose`。
4. 创建 `PathMapTrajectory` 轨迹对象。
5. 通过 timer 周期性发布期望 `Pose/Twist/Accel`。

节点层不直接实现复杂轨迹数学，只负责把纯逻辑层输出的结果转换成 ROS message。

### 5.2 轨迹逻辑层

文件：

```text
robot_trajectory_planner/path_map_trajectory_logic.py
```

职责：

1. 解析 `path_map.txt`。
2. 把 rotation vector 转换成 quaternion。
3. 根据 `max_path_linear_step` 和 `max_path_angular_step` 对文件路径点做预重采样。
4. 根据最大线速度和最大角速度计算每段轨迹持续时间。
5. 对位置做线性插值。
6. 对姿态做 quaternion slerp 插值。
7. 根据相邻发布周期的位姿变化计算 twist。

这层没有 ROS 依赖，因此可以用普通 pytest 单元测试验证。

## 6. 轨迹生成原理

### 6.1 初始化阶段

节点启动后先读取路径文件，但不会马上创建轨迹。原因是它还不知道实际机械臂末端当前在哪里。

流程：

```text
启动节点
  -> 读取 path_map.txt
  -> 等待 /asm_ee_site/pose
  -> 收到当前位姿
  -> 以当前位姿为 start_pose 创建 PathMapTrajectory
```

这样可以避免节点一启动就把目标位姿跳到路径第一个点。

### 6.2 过渡到第一个路径点

轨迹对象的第一个目标点就是 `path_map.txt` 的第一行。由于起点来自 `/asm_ee_site/pose`，所以第一段轨迹自然就是：

```text
当前 asm_ee_site 位姿 -> path_map 第一个点
```

这一段同样受 `max_linear_speed` 和 `max_angular_speed` 限制，因此是慢速平滑过渡。

### 6.3 路径点预重采样

在计算实时轨迹前，节点会先对 `path_map.txt` 读出的原始路径点做一次预重采样。对每一段原始路径点 `A -> B`，程序计算：

```text
linear_steps = ceil(位置距离 / max_path_linear_step)
angular_steps = ceil(姿态夹角 / max_path_angular_step)
step_count = max(linear_steps, angular_steps, 1)
```

然后使用位置线性插值和 quaternion slerp 姿态插值生成中间点。这样可以把文件中间隔较大的点拆成更密的内部路径点。

例如两个原始路径点相距 `0.064 m`，默认 `max_path_linear_step=0.002` 时，这一段会被拆成约 32 小段。

需要注意：预重采样生成的是原始点之间的直线/球面插值点，能让目标点更密，但不会自动生成圆角，也不会改变原始路径的几何形状。

### 6.4 每段轨迹持续时间

对每两个相邻位姿，程序分别计算：

```text
linear_time = 位置距离 / max_linear_speed
angular_time = 姿态夹角 / max_angular_speed
segment_duration = max(linear_time, angular_time, min_segment_duration)
```

这样可以保证：

- 位置运动不会超过 `max_linear_speed`。
- 姿态运动不会超过 `max_angular_speed`。
- 如果设置了 `min_segment_duration`，每段不会短于该时间。

### 6.5 位置插值

位置采用线性插值：

```text
p(t) = p_start + ratio * (p_target - p_start)
```

其中：

```text
ratio = segment_elapsed / segment_duration
```

### 6.6 姿态插值

`path_map.txt` 中的姿态是 rotation vector。程序先将其转成 quaternion：

```text
angle = norm([rx, ry, rz])
axis = [rx, ry, rz] / angle
q = [axis * sin(angle / 2), cos(angle / 2)]
```

每段轨迹内部再用 quaternion slerp 做姿态插值。这样比直接对 `rx ry rz` 做线性插值更稳定，能避免 rotation vector 表达在接近 pi 附近的跳变问题。

### 6.7 Twist 计算

每个 timer 周期都会记录上一周期发布的 pose 和当前周期发布的 pose。

线速度：

```text
v = (p_current - p_previous) / dt
```

角速度：

```text
q_delta = inverse(q_previous) * q_current
rotvec_delta = quaternion_to_rotation_vector(q_delta)
omega = rotvec_delta / dt
```

这些速度作为 `/scan/desired_twist` 发布。

## 7. 与 ultra_scanning_system 的关系

在 `ultra_scanning_system.launch.py` 中，`path_map_trajectory_node` 只负责生成期望轨迹：

```text
path_map_trajectory_node
  -> /scan/desired_pose
  -> /scan/desired_twist
  -> /scan/desired_accel
```

后续控制链路仍然是：

```text
robot_admittance_control
  -> /admittance/asm_ee_cmd_pose
  -> asm_ee_command_transform
  -> /arm_desired_pose_tool0
  -> rtde_servol_pose_controller_node
```

所以如果 `path_map_trajectory_node` 正常启动，但机械臂不动，优先检查：

1. `/scan/desired_pose` 是否在发布。
2. `admittance_controller_node` 是否正常运行。
3. `/admittance/asm_ee_cmd_pose` 是否在发布。
4. `/arm_desired_pose_tool0` 是否在发布。
5. `rtde_servol_pose_controller_node` 是否收到命令。

## 8. 常用调试命令

查看节点是否启动：

```bash
ros2 node list | grep path_map
```

查看期望位姿：

```bash
ros2 topic echo /scan/desired_pose
```

查看期望速度：

```bash
ros2 topic echo /scan/desired_twist
```

查看当前末端位姿：

```bash
ros2 topic echo /asm_ee_site/pose
```

确认路径文件安装内容：

```bash
ros2 pkg prefix robot_trajectory_planner
```

然后检查：

```text
<prefix>/share/robot_trajectory_planner/data/path_map.txt
```

## 9. 安全注意事项

1. `path_map.txt` 的坐标系必须和 `/asm_ee_site/pose`、导纳控制器使用的坐标系一致，目前系统中按 `base` 理解。
2. `path_map.txt` 的姿态必须是 rotation vector，不是 RPY，也不是 quaternion。
3. 首次真实机械臂联调时建议使用很小的速度，例如：

```bash
path_map_max_linear_speed:=0.002
path_map_max_angular_speed:=0.02
path_map_max_linear_step:=0.001
path_map_max_angular_step:=0.01
```

4. 不要同时启动 `current_pose_hold_node` 和 `path_map_trajectory_node`，否则它们会同时发布 `/scan/desired_*`。
5. 如果导纳控制器没有启动成功，`path_map_trajectory_node` 仍然会发布轨迹，但机械臂不会执行。
