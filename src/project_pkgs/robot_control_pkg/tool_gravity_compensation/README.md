# tool_gravity_compensation

ROS 2 Python package for:
- multi-pose least-squares calibration of tool gravity parameters (mass and CoM)
- online force/torque gravity compensation

## Nodes
- `gravity_calibration_node`
- `gravity_compensation_node`

## Identification Model

For each pose, with gravity projected in sensor frame as `g_s`:

- `f = b_f + m * g_s`
- `tau = b_tau + (r_com x (m * g_s))`

Unknowns are solved by least squares from multiple poses:

- tool mass `m`
- CoM in sensor frame `r_com`
- force bias `b_f`
- torque bias `b_tau`

## Quick Start

1. Build package:

```bash
cd ~/rrc_ws
colcon build --packages-select tool_gravity_compensation
source install/setup.bash
```

2. Run calibration node:

```bash
ros2 launch tool_gravity_compensation gravity_calibration.launch.py
```

3. Start collection, move robot through diverse orientations, then stop:

```bash
ros2 service call /start_collection std_srvs/srv/Trigger {}
ros2 service call /stop_collection std_srvs/srv/Trigger {}
```

4. Solve and save calibration result:

```bash
ros2 service call /solve_and_save std_srvs/srv/Trigger {}
```

5. Run compensation node (uses saved JSON by default):

```bash
ros2 launch tool_gravity_compensation gravity_compensation.launch.py
```

## 动态重力标定用户操作指南

动态标定用于包含活动关节的 ASM 工具。标定过程中需要改变 UR 末端姿态和
ASM 关节角度，使各个 link 在重力方向下具有足够丰富的姿态变化。

### 1. 标定前检查

标定前确认：

- 工具未接触工件、桌面或其他外部物体。
- 力传感器、UR 驱动、编码器和 `asm_tool_tf_broadcaster` 已启动。
- 编码器零点、方向和单位正确，旋转关节单位为 rad。
- 当前机构版本、`link_frames` 和输出标定文件相互匹配。
- 标定过程中不要重新清零力传感器，否则不同姿态的数据偏置不一致。

检查原始力传感器数据：

```bash
ros2 topic echo /external_force_torque_wrench --once
```

检查必要的 TF：

```bash
ros2 run tf2_ros tf2_echo base asm_force_sensor_link
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link1
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link2
```

如果出现 frame 不存在或两棵 TF 树不连通，不要继续标定，应先修复机构 TF。

### 2. 检查标定参数

编辑：

```text
config/gravity_calibration_dynamic_params.yaml
```

重点确认：

- `world_frame` 和 `sensor_frame` 与实际 TF 一致。
- `link_frames` 与当前机构版本一致。
- `tool_joint_names` 与编码器发布的 joint name 一致。
- `output_file` 指向当前机构专用的标定文件。

新旧机构不要共用同一个标定结果。建议分别保存，例如：

```text
config/tool_gravity_calibration_dynamic_v1.json
config/tool_gravity_calibration_dynamic_v2.json
config/tool_gravity_calibration_dynamic_v3.json
```

### 3. 启动动态标定节点

```bash
cd ~/Lab_WS/rrc_ws
source install/setup.bash
ros2 launch ultra_scanning_system ultra_scanning_hardware_init.launch.py
```

```bash
cd ~/Lab_WS/rrc_ws
source install/setup.bash
ros2 launch tool_gravity_compensation gravity_calibration_dynamic.launch.py
```

节点启动后应显示原始 wrench 话题、坐标系、参与标定的 link 和输出文件。

### 4. 清空旧样本

每次开始一轮新标定时执行：

```bash
ros2 service call /dynamic_gravity_calibration_node/clear_samples std_srvs/srv/Trigger {}
```

### 5. 逐姿态采集

每个姿态按以下顺序操作：

1. 将 UR 末端和 ASM 关节移动到目标姿态。
2. 确认工具无接触并完全静止。
3. 等待力传感器读数稳定。
4. 调用一次采集服务：

```bash
ros2 service call /dynamic_gravity_calibration_node/collect_current_pose std_srvs/srv/Trigger {}
```

一次调用会按照 `samples_per_capture` 连续采集多帧。日志出现
`Capture complete` 后，才能移动到下一个姿态。采集期间不要移动机构。

可随时查询样本数量和当前采集状态：

```bash
ros2 service call /dynamic_gravity_calibration_node/sample_count std_srvs/srv/Trigger {}
```

如果返回的 `capture_remaining` 大于 0，表示当前姿态仍在采集中。

### 6. 姿态覆盖建议

采集姿态应同时覆盖：

- 多个 UR wrist 方向，使重力在力传感器坐标系中的方向明显变化。
- ASM 第一个活动关节的负、中、正角度区域。
- ASM 第二个活动关节的负、中、正角度区域。
- 新机构存在直线关节时，还应覆盖不同的伸出位置。

建议至少使用 8 个明显不同的 UR 末端方向，并在整个采样集中覆盖各 ASM
关节的完整工作区间。姿态数量越多、差异越明显，参数通常越容易被充分辨识。
不要只改变一个关节，也不要采集大量几乎相同的姿态。

### 7. 求解并保存

完成所有姿态采集后，确保 `capture_remaining=0`，然后执行：

```bash
ros2 service call /dynamic_gravity_calibration_node/solve_and_save std_srvs/srv/Trigger {}
```

成功时返回内容包含：

- `samples`：参与求解的总样本数。
- `rank`：最小二乘矩阵秩。
- `rms`：拟合残差。
- `file`：标定结果保存路径。

### 8. 检查标定结果

打开输出 JSON，检查：

- `link_frames` 与当前机构版本一致。
- `link_masses` 均为正数，且量级符合实际机构。
- 各 link 的质心没有明显超出对应机构尺寸。
- `rank` 没有因姿态单一而明显不足。
- `rms_residual` 足够小，且优于此前的标定结果。

若出现非正质量、异常质心、秩不足或残差过大，应清空样本后重新采集，并增加
UR 姿态和 ASM 关节姿态的变化范围。

### 9. 使用标定结果验证补偿

将动态补偿配置中的 `calibration_file` 设置为刚生成的 JSON：

```text
config/gravity_compensation_dynamic_params.yaml
```

然后启动：

```bash
ros2 launch tool_gravity_compensation gravity_compensation_dynamic.launch.py
```

保持工具静止且无接触，在多个 UR 和 ASM 姿态下检查：

```bash
ros2 topic echo /external_force_torque_wrench_compensated
```

补偿后的力和力矩应在各姿态下接近零，并且不应随 ASM 关节运动出现明显的
重力趋势。验证通过前不要直接用于接触扫查或导纳控制。

## Config Files

## ASM v2 合并刚体模式

当 `asm_tool_link2` 与 `asm_tool_link3` 之间的旋转关节在标定和补偿期间保持
基本不变时，可将二者及末端固定零件作为一个刚体，并使用三刚体模型：

```text
asm_tool_base_link
asm_tool_link1
asm_tool_link2  # 代表 link2 + link3 + waterproof + ee
```

该模式将 X/Y 质心软约束尺度设为 30 mm，并按采样数归一化测量方程，避免
重复采样削弱正则项。启动命令：

```bash
ros2 launch tool_gravity_compensation gravity_calibration_dynamic_v2_merged.launch.py
ros2 launch tool_gravity_compensation gravity_compensation_dynamic_v2_merged.launch.py
```

使用此模式期间必须保持 `asm_tool_link2 -> asm_tool_link3` 的关节角接近标定
时的固定角度；若该关节明显运动，应改回四刚体模式。

- `config/gravity_calibration_params.yaml`
- `config/gravity_compensation_params.yaml`

## ASM v2 动态重力标定

标定前确认传感器到四个刚体帧的 TF 均可用：

```bash
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_base_link
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link1
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link2
ros2 run tf2_ros tf2_echo asm_force_sensor_link asm_tool_link3
```

启动 v2 标定，清空样本，逐姿态采集并求解：

```bash
ros2 launch tool_gravity_compensation gravity_calibration_dynamic_v2.launch.py
ros2 service call /dynamic_gravity_calibration_node/clear_samples std_srvs/srv/Trigger {}
ros2 service call /dynamic_gravity_calibration_node/collect_current_pose std_srvs/srv/Trigger {}
ros2 service call /dynamic_gravity_calibration_node/solve_and_save std_srvs/srv/Trigger {}
```

建议使用 30--50 个覆盖 UR 和三个 ASM 关节工作区的静止姿态进行标定，另留
10 个未参与求解的姿态验收。`constrained_v2` 识别的是在当前四帧运动学模型下
产生相同重力 wrench 的等效质量和质心参数，不要求逐项等于 CAD 刚体参数。

启动 v2 补偿：

```bash
ros2 launch tool_gravity_compensation gravity_compensation_dynamic_v2.launch.py
```

建议在 10 个留出姿态下验收静止无接触残差：力不超过 `0.5 N`，力矩不超过
`0.03 Nm`。
