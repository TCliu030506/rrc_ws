# Dynamic Gravity Calibration And Compensation Plan

## 目标

当前 `gravity_compensation_node` 将末端工具近似为一个刚体，使用单个质量、单个质心和固定偏置做重力补偿。ASM 工具有两个活动关节，真实重力力矩会随 `asm_tool_joint1`、`asm_tool_joint2` 改变，因此需要升级为动态重力补偿：

- 将传感器后端的 ASM 机构拆成多个刚体 link。
- 分别标定每个 link 的质量和相对本 link 坐标系的质心。
- 在线补偿时根据当前 TF 链实时计算各 link 质心在力传感器坐标系下的位置。
- 累加每个 link 的重力力和重力力矩，得到当前姿态下的重力 wrench model。
- 从原始外力传感器数据中减去该模型，输出补偿后的 wrench。

拟新增：

- `gravity_calibration_dynamic.launch.py`
- `gravity_compensation_dynamic.launch.py`

功能上参考现有：

- `gravity_calibration_mujoco_dynamic.launch.py`
- `gravity_compensation_mujoco_dynamic.launch.py`

但硬件版本应面向真实 UR + 编码器 + 力传感器系统，不直接依赖 MuJoCo 的 `/command_joint` 自动运动接口。

## 坐标系和 TF 前提

硬件系统中的主要 TF 链为：

```text
base
└── ... UR chain ...
    └── tool0
        └── asm_base
            └── asm_force_sensor_link
                └── asm_tool_base_link
                    └── asm_tool_link1
                        └── asm_tool_link2
                            └── asm_ee_site
```

动态重力补偿使用：

- `world_frame`: `base`
- `sensor_frame`: `asm_force_sensor_link`
- `link_frames`:
  - `asm_tool_base_link`
  - `asm_tool_link1`
  - `asm_tool_link2`

其中 `asm_tool_link1`、`asm_tool_link2` 的 TF 由编码器节点和 `asm_tool_tf_broadcaster` 维护。动态补偿正确性的前提是：

- `encoder1/joint_state` 和 `encoder2/joint_state` 单位为 rad。
- 编码器零点、方向和 offset 已经校正。
- `tool0 -> asm_ee_site` 随两个 ASM 关节真实变化。
- `base -> asm_force_sensor_link` 可由 TF 实时查到。

## 重力模型

对第 `i` 个 link：

- 质量：`m_i`
- link 局部质心：`c_i_link`
- 当前 TF：`T_sensor_link_i = (R_i, p_i)`
- 重力在传感器坐标系下的向量：`g_sensor`

该 link 的质心在传感器坐标系下为：

```text
r_i_sensor = p_i + R_i * c_i_link
```

该 link 对传感器的重力力为：

```text
F_i = m_i * g_sensor
```

该 link 对传感器原点的重力力矩为：

```text
tau_i = r_i_sensor x F_i
```

总重力模型为：

```text
F_model   = force_bias  + Σ F_i
tau_model = torque_bias + Σ tau_i
```

补偿输出为：

```text
wrench_compensated = wrench_raw - wrench_model
```

## 标定原理

标定时采集多组姿态样本。每组样本包括：

- 原始力传感器 wrench。
- `base -> asm_force_sensor_link`，用于计算 `g_sensor`。
- `asm_force_sensor_link -> link_frame_i`，用于计算每个 link 的 `R_i` 和 `p_i`。

理想测量方程为：

```text
F_meas   = force_bias  + Σ(m_i * g_sensor)
tau_meas = torque_bias + Σ((p_i + R_i * c_i_link) x (m_i * g_sensor))
```

为保持最小二乘问题线性，推荐直接估计：

```text
s_i = m_i * c_i_link
```

未知量为：

```text
force_bias(3), torque_bias(3),
m_0 ... m_n,
s_0x s_0y s_0z ... s_nx s_ny s_nz
```

求解后：

```text
c_i_link = s_i / m_i
```

相比直接估计 `c_i_link`，估计 `s_i = m_i * c_i_link` 可以避免质量和质心相乘造成的非线性问题。求解后需要做合理性检查：

- `m_i > 0`
- `c_i_link` 不应明显超出对应 link 的几何范围
- 样本数应显著多于未知量数量
- `rank` 应足够高
- `rms_residual` 应明显低于单刚体模型

## 标定采样策略

真实硬件版本不建议一开始做自动运动。推荐先实现安全的服务式采样：

1. 启动硬件 TF、编码器、力传感器和动态标定节点。
2. 人工或外部脚本将 UR wrist 和 ASM 两个关节移动到不同姿态。
3. 每个姿态静止后调用服务采集一组样本。
4. 采集足够姿态后调用服务求解并保存 JSON。

需要覆盖的姿态变化：

- UR wrist 姿态变化：改变重力方向在传感器坐标系下的投影。
- ASM joint1 变化：让 `asm_tool_link1` 和下游 link 的质心相对传感器变化。
- ASM joint2 变化：让 `asm_tool_link2` 的质心相对传感器变化。

建议初始采样矩阵：

- UR wrist 姿态不少于 8 个。
- 每个 UR wrist 姿态下，ASM joint1/joint2 至少覆盖负、中、正 3 个区域。
- 每个静止姿态采集 10 到 30 帧 wrench 并取入最小二乘。

如果后续需要自动化，可在服务式版本稳定后再增加硬件自动运动模式。自动运动模式应复用已有 UR RTDE 控制接口，但必须单独加速度限制、工作空间限制和急停说明。

## 拟新增节点

建议新增硬件专用节点，而不是直接复用 `mujoco_dynamic_*` 类名：

- `dynamic_gravity_calibration_node`
- `dynamic_gravity_compensation_node`

这样可以保留 MuJoCo 版本作为仿真专用实现，避免硬件 launch 误触发仿真命令接口。

### dynamic_gravity_calibration_node

职责：

- 订阅原始 wrench。
- 监听 TF。
- 提供采样和求解服务。
- 保存动态标定 JSON。

建议参数：

```yaml
dynamic_gravity_calibration_node:
  ros__parameters:
    wrench_topic: /external_force_torque_wrench
    world_frame: base
    sensor_frame: asm_force_sensor_link
    gravity_norm: 9.81
    sample_period_sec: 0.05
    samples_per_capture: 20
    link_frames:
      - asm_tool_base_link
      - asm_tool_link1
      - asm_tool_link2
    tool_joint_names:
      - brt_encoder1_joint
      - brt_encoder2_joint
    output_file: /home/liutiancheng/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_dynamic.json
```

建议服务：

- `~/clear_samples`
- `~/collect_current_pose`
- `~/solve_and_save`
- `~/sample_count`

### dynamic_gravity_compensation_node

职责：

- 订阅原始 wrench。
- 每帧读取当前 TF。
- 从 JSON 加载每个 link 的质量、质心和 wrench bias。
- 发布补偿后的 wrench 和模型 wrench。

建议参数：

```yaml
dynamic_gravity_compensation_node:
  ros__parameters:
    wrench_in_topic: /external_force_torque_wrench
    wrench_out_topic: /external_force_torque_wrench_compensated
    gravity_wrench_topic: /external_gravity_compensation_wrench_model
    calibration_file: /home/liutiancheng/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_dynamic.json
    world_frame: base
    sensor_frame: asm_force_sensor_link
    gravity_norm: 9.81
    link_frames:
      - asm_tool_base_link
      - asm_tool_link1
      - asm_tool_link2
```

## 标定文件格式

建议输出：

```json
{
  "estimated_at": "...",
  "mode": "hardware_dynamic_service",
  "world_frame": "base",
  "sensor_frame": "asm_force_sensor_link",
  "gravity_norm": 9.81,
  "sample_count": 360,
  "rank": 18,
  "rms_residual": 0.12,
  "link_frames": [
    "asm_tool_base_link",
    "asm_tool_link1",
    "asm_tool_link2"
  ],
  "link_masses": [0.1, 0.2, 0.8],
  "link_com_x": [0.0, 0.0, 0.0],
  "link_com_y": [0.0, 0.0, 0.0],
  "link_com_z": [0.01, 0.02, 0.03],
  "force_bias": [0.0, 0.0, 0.0],
  "torque_bias": [0.0, 0.0, 0.0],
  "tool_joint_names": [
    "brt_encoder1_joint",
    "brt_encoder2_joint"
  ]
}
```

## Launch 设计

### gravity_calibration_dynamic.launch.py

只启动动态标定节点，不自动启动 UR 运动控制。

默认参数文件：

```text
config/gravity_calibration_dynamic_params.yaml
```

### gravity_compensation_dynamic.launch.py

只启动动态补偿节点。

默认参数文件：

```text
config/gravity_compensation_dynamic_params.yaml
```

在 `ultra_scanning_system.launch.py` 中替换旧补偿节点时，应将当前：

```text
gravity_compensation_node
```

替换为：

```text
dynamic_gravity_compensation_node
```

并保持输入输出 topic 不变：

```text
/external_force_torque_wrench
/external_force_torque_wrench_compensated
/external_gravity_compensation_wrench_model
```

## 验证流程

### 离线合理性检查

标定完成后检查 JSON：

- `estimated_at` 是当前标定时间。
- `world_frame == base`
- `sensor_frame == asm_force_sensor_link`
- link 数量与配置一致。
- 质量为正。
- 质心数值在机械结构合理范围内。
- `rms_residual` 相比单刚体标定下降。

### 在线 topic 检查

启动补偿后检查：

```bash
ros2 topic echo /external_gravity_compensation_wrench_model --once
ros2 topic echo /external_force_torque_wrench --once
ros2 topic echo /external_force_torque_wrench_compensated --once
```

在无接触、静止条件下：

- 原始 wrench 应接近 `gravity_wrench_model`。
- 补偿 wrench 应接近零。
- 改变 ASM 两个关节姿态时，`gravity_wrench_model` 应随姿态变化。

### TF 检查

```bash
ros2 run tf2_ros tf2_echo base asm_force_sensor_link
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_base_link
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link1
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link2
```

如果 TF 查不到或跳变，动态补偿不可信。

## 风险和约束

- 动态标定依赖姿态多样性。只转 UR wrist、不动 ASM 关节，无法可靠区分各 link 的质量和质心。
- 力传感器必须在无接触状态下采样，否则外部接触力会被误认为重力。
- 编码器方向或零点错误会直接污染 TF，从而导致标定结果错误。
- 若 link 质量和质心同时自由估计，问题可能病态；需要使用 `s_i = m_i * c_i` 线性参数化，并对结果做合理性检查。
- 真实硬件自动运动应作为第二阶段功能，不能在首版 launch 中默认启用。

## 推荐实施顺序

1. 新增硬件动态标定/补偿参数 YAML。
2. 新增 `dynamic_gravity_calibration_node`，先实现服务式采样和求解。
3. 新增 `dynamic_gravity_compensation_node`，复用当前 TF 累加补偿模型。
4. 新增两个 launch 文件。
5. 用静态无接触数据验证补偿输出。
6. 再考虑把动态补偿接入 `ultra_scanning_system.launch.py`。
7. 最后再评估是否需要硬件自动姿态采样。
