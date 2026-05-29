# 工具坐标系导纳控制

本文档说明 `robot_admittance_control` 中的工具坐标系导纳控制实现。

## 目标

控制器在受控工具坐标系 `asm_ee_site` 下计算导纳动力学。外部力/力矩
和控制力/力矩都会先转换到该坐标系；虚拟导纳状态作为工具局部坐标系
下的柔顺位移进行积分；最后再将得到的柔顺目标位姿转换回机器人命令
基坐标系并发布。

## 坐标系

- `base_frame`：发布目标位姿时使用的命令/参考基坐标系。在真实扫查
  launch 中该值为 `base`。
- `admittance_frame`：导纳动力学计算坐标系。对于工具坐标系导纳控制，
  该值应为 `asm_ee_site`。
- `ft_sensor_frame`：当外部 wrench 消息的 `header.frame_id` 为空时使用
  的默认力传感器坐标系。
- `control_wrench_frame`：当控制 wrench 消息的 `header.frame_id` 为空时
  使用的默认控制力坐标系。

真实系统默认配置如下：

```yaml
base_frame: "base"
arm_base_frame: "base"
ft_sensor_frame: "asm_force_sensor_link"
control_wrench_frame: "asm_ee_site"
admittance_frame: "asm_ee_site"
tool_displacement_reference: "current"
```

## 理论说明

设 `{B}` 为命令基坐标系，`{E}` 为 `asm_ee_site`。导纳律使用的 wrench
需要表达在 `{E}` 下：

```text
^E W = [ ^E f ; ^E tau ]
```

对于在源坐标系 `{S}` 中测得的 wrench，控制器使用完整的 wrench 变换，
其中包含由于力作用点平移产生的力矩项：

```text
^E f   = R_ES * ^S f
^E tau = R_ES * ^S tau + p_ES x (R_ES * ^S f)
```

矩阵形式为：

```text
^E W = [ R_ES             0 ] [ ^S f   ]
       [ [p_ES]x * R_ES R_ES ] [ ^S tau ]
```

导纳动力学在 `{E}` 中积分：

```text
M_E * xdd_a^E + D_E * xd_a^E + K_E * x_a^E = W_ext^E - W_ctrl^E
```

其中：

```text
x_a^E  = [ delta_p_E ; delta_theta_E ]
xd_a^E = [ v_E ; omega_E ]
```

`delta_p_E` 是工具坐标系下的局部平移量，`delta_theta_E` 是工具坐标系下
的局部旋转向量。最终发布的命令位姿表达在 `{B}` 中：

```text
p_cmd_B = p_ref_B + R_BE * delta_p_E
q_cmd_B = q_ref_B * Exp(delta_theta_E)
```

这里对 `Exp(delta_theta_E)` 使用右乘，表示柔顺旋转作用在工具局部坐标系
中，而不是作为世界/基坐标系中的旋转。

由于 `{E}` 会随时间运动，控制器中保存的导纳状态必须始终表达在当前工具
坐标系下。因此，每个控制周期在应用动力学之前，都会先把上一周期工具
坐标系下的状态从上一周期工具姿态重新表达为当前工具姿态下的状态：

```text
x_current^E = R_Ecurrent_Eprevious * x_previous^E
```

这样可以避免工具旋转后，旧工具坐标系下的状态分量与新工具坐标系下的
wrench 分量混用。

## 程序数据流

1. `wrench_external_callback()` 读取外部 `WrenchStamped`，使用消息中的
   `header.frame_id` 或默认的 `ft_sensor_frame`，并将 wrench 转换到
   `admittance_frame`。
2. `wrench_control_callback()` 读取控制 `WrenchStamped`，使用消息中的
   `header.frame_id` 或默认的 `control_wrench_frame`，并将 wrench 转换到
   `admittance_frame`。
3. `compute_admittance()` 在 `admittance_frame` 中积分虚拟导纳状态。
4. 虚拟局部位移会叠加到选定的参考位姿上：
   - `tool_displacement_reference: "current"`：将柔顺量叠加到当前
     `asm_ee_site` 位姿。
   - `tool_displacement_reference: "desired"`：将柔顺量叠加到期望轨迹位姿。
5. 控制器通过 `topic_arm_pose_command` 发布受控坐标系在基坐标系下的
   目标位姿。
6. 在 `ultra_scanning_system` 中，`asm_ee_command_transform` 会将
   `base -> asm_ee_site` 转换为 `base -> tool0`，供 RTDE servoL 控制器
   使用。

## 主要代码改动

- `tool_frame_admittance_math.h` 提供纯数学辅助函数，用于：
  - 工具局部坐标系下的位姿叠加；
  - 将局部 twist 旋转到命令基坐标系；
  - 工具坐标系旋转时重新表达导纳状态。
- `AdmittanceController.cpp` 会将外部 wrench 和控制 wrench 都转换到
  `admittance_frame`。
- `AdmittanceController.cpp` 将 `admittance_displacement_` 和
  `admittance_twist_` 明确定义为工具坐标系状态。
- `admittance_controller_node.cpp` 声明了新增参数。
- `admittance_params_ros2.yaml` 和 `admittance_params_ros2_sim.yaml` 设置了
  `admittance_frame: "asm_ee_site"`。

## 验证

包中新增了 `tool_frame_admittance_math_test`，用于验证：

- 工具局部平移会正确旋转到命令基坐标系；
- 工具局部旋转会以右乘方式叠加到参考姿态上；
- 当工具坐标系旋转时，已保存的工具坐标系导纳状态会被正确重新表达。

运行：

```bash
colcon build --packages-select robot_admittance_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select robot_admittance_control --ctest-args -R tool_frame_admittance_math_test --output-on-failure
```
