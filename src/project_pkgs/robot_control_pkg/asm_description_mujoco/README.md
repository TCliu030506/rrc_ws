# asm_description_mujoco

基于 MuJoCo + ROS 2 的 UR5e + ASM 末端执行器仿真功能包。该包负责加载 MuJoCo 模型、运行动力学仿真、接收 ROS 2 控制命令，并发布关节状态、末端位姿、传感器、相机图像和外力反馈。

## 功能概览

- 加载 `ur5e_with_asm/scene_all.xml` MuJoCo 场景。
- 使用独立仿真线程推进 `mj_step()`。
- 通过状态快照发布 ROS 2 数据，避免发布线程直接读写主仿真 `MjData`。
- 支持关节目标控制、原始 actuator 控制和末端笛卡尔位姿控制。
- 对控制命令做 NaN/Inf 检查、actuator `ctrlrange` 限幅和四元数归一化。
- 使用 reliable QoS 接收控制命令，传感器/图像输出继续使用 sensor data QoS。
- 支持 MuJoCo viewer、离屏相机渲染和多相机图像发布。
- 发布 MuJoCo force/torque 传感器组合后的外部 wrench，供导纳控制使用。

## 主要程序结构

- `asm_ros2_node.py`：ROS 2 主节点，负责装配模型、线程、接口和数据流。
- `command_validation.py`：控制命令校验、限幅和笛卡尔目标检查。
- `ik_solver.py`：基于 MuJoCo Jacobian 的阻尼最小二乘 IK 求解器。
- `sim_types.py`：仿真状态、命令状态、相机帧和发布器描述等数据结构。
- `qos_profiles.py`：控制话题使用的 reliable QoS 配置。
- `keyboard_teleop.py`：用于快速测试的键盘控制节点。
- `mujoco_tf_broadcaster.py`：根据 `/joint_states` 广播 MuJoCo 模型 TF。
- `camera_visualizer_node.py`：相机图像可视化节点。

## 启动

构建并加载环境：

```bash
colcon build --packages-select asm_description_mujoco
source install/setup.bash
```

启动默认仿真：

```bash
ros2 launch asm_description_mujoco asm_ros2_node.launch.py
```

常用无界面启动：

```bash
ros2 launch asm_description_mujoco asm_ros2_node.launch.py use_viewer:=false publish_rgb:=false
```

## 控制接口

### `/command_joint`

类型：`sensor_msgs/msg/JointState`

按关节名发送目标值。默认 `control_mode=position`，节点读取 `msg.position`；如果设置为 `velocity` 或 `effort`，节点会改读 `msg.velocity` 或 `msg.effort`。

重要说明：`control_mode` 只决定从 `JointState` 的哪个字段取数，并不会自动改变 MuJoCo actuator 类型。当前 `ur5e_with_asm.xml` 中 UR5e 使用 `general` actuator 配成位置伺服形式，因此这些 `data.ctrl` 输入实际仍按 XML 中的 actuator 动力学解释。若要真正做速度或力矩控制，需要同步修改 MuJoCo XML 的 actuator/控制律。

### `/command_raw`

类型：`std_msgs/msg/Float64MultiArray`

按 MuJoCo actuator 顺序直接写入控制量。默认要求数组长度等于 `model.nu`；长度不匹配时会拒绝该命令，避免未指定 actuator 被误置零。可通过 `allow_partial_raw_commands:=true` 允许短数组。

当前默认模型的 actuator 顺序：

```text
0 shoulder_pan  -> shoulder_pan_joint
1 shoulder_lift -> shoulder_lift_joint
2 elbow         -> elbow_joint
3 wrist_1       -> wrist_1_joint
4 wrist_2       -> wrist_2_joint
5 wrist_3       -> wrist_3_joint
```

### `/cartesian_target_pose`

类型：`geometry_msgs/msg/PoseStamped`

发送末端目标位姿。节点默认使用 `asm_ee_site` 作为末端 site，并在 MuJoCo 内部用阻尼最小二乘 IK 计算一组目标关节角，再写入 actuator 控制目标。

注意：

- 目标位姿默认按 `world` 坐标系解释。
- 节点不会做 TF 坐标变换；如果收到非 `world` 的 `frame_id`，会报警并仍按 `world` 数值解释。
- 四元数会自动归一化；NaN、Inf 或近零四元数会被拒绝。
- IK 不直接改 `qpos`，而是通过 actuator 目标驱动机械臂运动，更接近动力学仿真。

## 发布接口

### `/joint_states`

类型：`sensor_msgs/msg/JointState`

发布 MuJoCo 模型中 hinge/slide 关节的 `qpos` 和 `qvel`。默认使用 sensor data QoS。

### `/end_effector_pose`

类型：`geometry_msgs/msg/PoseStamped`

发布 `end_effector_site_name` 对应 site 的世界位姿。默认 site 为 `asm_ee_site`。

### `/sensors/<sensor_name>`

根据 MuJoCo sensor 自动发布：

- 3 维 `_pos`：`PointStamped`
- 4 维 `_quat`：`QuaternionStamped`
- 3 维 `_vel`、`_force`、`_torque`：`Vector3Stamped`
- 6 维 `_wrench`：`WrenchStamped`
- 其他：`Float64MultiArray`

可用 `sensor_names` 指定只发布部分传感器；为空时发布模型中所有命名 sensor。

### `/sensors/wrench`

类型：`geometry_msgs/msg/WrenchStamped`

由 `external_force_sensor_name` 和 `external_torque_sensor_name` 指定的 MuJoCo force/torque sensor 组合而来。当前实现会对 MuJoCo 读到的 force/torque 乘以 `-1.0` 后发布，这是为了匹配现有导纳控制链路中的外力方向约定。使用其他控制器时需要确认该正负号是否符合你的坐标系定义。

### 相机图像

单相机模式：

- `/camera/rgb/image_raw`
- `/camera/depth/image_raw`

多相机模式：

- `/camera/<camera_name>/rgb/image_raw`
- `/camera/<camera_name>/depth/image_raw`

默认 `publish_rgb=true`，`publish_depth=false`，`render_only_with_subscribers=true`。

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model_path` | `scene_all.xml` | MuJoCo XML 模型路径 |
| `keyframe_name` | `home` | 启动时加载的 keyframe |
| `step_rate_hz` | `500.0` | 仿真线程目标频率；低于模型 timestep 时自动按 timestep |
| `publish_rate_hz` | `50.0` | ROS 数据发布频率 |
| `render_rate_hz` | `15.0` | 离屏相机渲染频率 |
| `control_qos_depth` | `10` | reliable 控制话题队列深度 |
| `enable_command_clipping` | `true` | 是否按 actuator `ctrlrange` 限幅 |
| `allow_partial_raw_commands` | `false` | 是否允许短 `command_raw` 数组 |
| `command_timeout_sec` | `0.0` | 控制命令超时；0 表示禁用，超时后保持最后目标 |
| `control_mode` | `position` | `command_joint` 读取字段；不改变 actuator 类型 |
| `end_effector_site_name` | `asm_ee_site` | FK/IK 使用的末端 site |
| `ik_max_iters` | `50` | IK 最大迭代次数 |
| `ik_damping` | `0.01` | 阻尼最小二乘 IK 阻尼项 |
| `ik_position_tolerance` | `0.0001` | IK 位置收敛阈值 |
| `ik_orientation_tolerance` | `0.001` | IK 姿态收敛阈值 |

## 线程模型

主节点使用多个独立工作线程：

- `mujoco_sim_thread`：唯一写主 `self.data` 的线程，负责读取最新控制命令、写 `data.ctrl`、调用 `mj_step()`。
- `ros2_publish_thread`：从 `SimState` 快照发布 `/joint_states`、末端位姿、传感器和外部 wrench。
- `camera_render_thread`：使用独立 `MjData` 从状态快照渲染相机图像。
- `image_pack_thread_*`：把渲染结果打包为 ROS `Image`。
- `mujoco_viewer_thread`：使用独立 `MjData` 同步 MuJoCo viewer。

线程之间不共享主 MuJoCo `self.data` 的直接读写。仿真线程每步拷贝出 `SimState` 快照，发布、渲染和 viewer 线程只读快照并写各自独立的 `MjData`。工作线程带有统一异常保护；如果某个工作线程崩溃，会记录 traceback 并触发停止事件。

## 键盘测试

关节模式：

```bash
ros2 run asm_description_mujoco asm_keyboard_teleop --mode joint
```

笛卡尔模式：

```bash
ros2 run asm_description_mujoco asm_keyboard_teleop --mode cartesian
```

键盘节点会先从 `/joint_states` 或 `/end_effector_pose` 初始化目标，然后发布到 `/command_joint` 或 `/cartesian_target_pose`。控制话题使用 reliable QoS，与仿真节点订阅端匹配。

## 与 ultra_scanning_sim 配合

`ultra_scanning_sim_mujoco.launch.py` 中的导纳控制输出会经过 `pose_to_pose_stamped_bridge` 转成 `/cartesian_target_pose`，该桥接节点同样使用 reliable QoS 发布控制命令。

典型启动：

```bash
ros2 launch ultra_scanning_sim ultra_scanning_sim_mujoco.launch.py
```

## 维护建议

- 修改 MuJoCo actuator 类型后，必须同步检查 `control_mode` 和控制输入单位。
- 新增 actuator 后，`command_raw` 的数组长度和顺序会随 `model.nu` 改变。
- 新增 mesh 后需要重新构建安装包，否则安装目录可能缺失资源文件。
- 若导纳控制效果异常，优先检查 `/sensors/wrench` 的方向约定、`/cartesian_target_pose` 的 frame_id，以及 IK 是否持续报警未收敛。
