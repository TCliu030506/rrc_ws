# 末端整体刚体惯性力补偿方案

## 背景

当前导纳控制使用力传感器输出作为外力输入。实际运行中发现：机械臂稳定在某
个位姿时，力传感器读数波动较小；但机械臂加减速时，力传感器读数会出现明显
波动。这种波动主要来自末端工具随机器人运动产生的惯性力，而不是外界接触力。

现有 `tool_gravity_compensation` 已经可以补偿静态重力项：

```text
wrench_compensated = wrench_raw - wrench_gravity_model
```

后续希望在此基础上加入动态惯性项：

```text
wrench_compensated = wrench_raw - wrench_gravity_model - wrench_inertia_model
```

由于 ASM 机构中的两个关节是被动柔顺关节，通常不会高频、大幅运动，因此第一
阶段不需要建立复杂多刚体动力学模型。更方便、稳健的做法是：将力传感器之后的
整个末端机构近似为一个整体刚体，只补偿该整体刚体随机械臂运动产生的惯性
wrench。

## 建模对象

定义：

- `{W}`：世界/机器人基坐标系，当前系统中通常为 `base`。
- `{S}`：力传感器坐标系，当前系统中通常为 `asm_force_sensor_link`。
- `m`：力传感器后端整体负载质量。
- `r_C^S`：整体质心相对传感器原点的位置，表达在 `{S}` 中。
- `g^S`：重力加速度向量，表达在 `{S}` 中。
- `a_O^S`：传感器原点线加速度，表达在 `{S}` 中。
- `omega^S`：传感器坐标系角速度，表达在 `{S}` 中。
- `alpha^S`：传感器坐标系角加速度，表达在 `{S}` 中。

如果忽略末端自身弹性变形和 ASM 被动关节的高频相对运动，则质心在惯性系中的
加速度表达到 `{S}` 后可近似为：

```text
a_C^S = a_O^S + alpha^S × r_C^S + omega^S × (omega^S × r_C^S)
```

其中：

- `a_O^S` 是传感器原点平动加速度项。
- `alpha^S × r_C^S` 是角加速度导致的切向加速度项。
- `omega^S × (omega^S × r_C^S)` 是角速度导致的向心加速度项。

末端负载作用在传感器上的非接触力模型可以写为：

```text
F_model^S = m * (g^S - a_C^S)
```

当机械臂静止时：

```text
a_C^S = 0
F_model^S = m * g^S
```

这就退化为当前的重力补偿模型。

对应传感器原点处的力矩模型为：

```text
tau_model^S = r_C^S × F_model^S
```

如果后续发现力矩方向的动态误差也很明显，可以进一步加入刚体转动惯量项：

```text
tau_rot^S = -I_C^S * alpha^S - omega^S × (I_C^S * omega^S)
tau_model^S = r_C^S × F_model^S + tau_rot^S
```

但第一阶段建议先不引入 `I_C`。原因是转动惯量标定更复杂，而导纳控制最敏感的
通常是力传感器的三轴力分量。先补偿平动惯性和质心力矩，调通后再评估是否需要
转动惯量补偿。

## 与现有重力补偿模型的关系

现有动态重力补偿已经支持多个 link：

```text
F_gravity = force_bias + Σ m_i * g^S
tau_gravity = torque_bias + Σ r_i^S × (m_i * g^S)
```

如果继续使用多 link 的重力模型，同时希望整体惯性补偿，可以把多 link 参数聚合
成一个等效整体刚体：

```text
m_total = Σ m_i
r_C^S = (Σ m_i * r_i^S) / m_total
```

其中 `r_i^S` 可以由当前 TF 和每个 link 的局部质心计算：

```text
r_i^S = p_i^S + R_i^S * c_i
```

这样做的好处是：

- 重力补偿仍然保留当前多 link 动态重力模型。
- 惯性补偿只使用一个整体质量和整体质心，避免复杂多刚体动力学。
- ASM 关节慢速变化时，整体质心 `r_C^S` 仍可以随当前 TF 缓慢更新。

如果想进一步简化，也可以完全使用单刚体标定文件中的 `mass` 和 `com`。但既然
当前项目已有动态重力标定结果，优先建议复用多 link 的质量/质心，并在线聚合出
整体等效质心。

## 加速度估计方案

最方便的实现方式是不增加额外硬件，而是从 TF 中估计 `{S}` 的运动状态。

每个补偿周期读取：

```text
T_W_S(t)
```

由相邻时刻位姿差分得到：

```text
v_O^W(t)     = d p_W_S / dt
a_O^W(t)     = d v_O^W / dt
omega^S(t)   = log(R_S(t-dt)^T * R_S(t)) / dt
alpha^S(t)   = d omega^S / dt
```

再将线加速度从 `{W}` 转换到 `{S}`：

```text
a_O^S = R_W_S^T * a_O^W
```

实际实现中必须滤波，因为二阶差分对噪声非常敏感。建议：

1. 对位置和姿态输入不直接滤波，保留 TF 原始状态。
2. 对速度和角速度做一阶低通。
3. 对线加速度和角加速度做更强的一阶低通。
4. 设置最大加速度限幅，避免 TF 抖动产生异常补偿力。
5. 当 `dt` 过小、过大或 TF 跳变时，跳过该帧惯性补偿，只保留重力补偿。

推荐初始参数：

```yaml
enable_inertia_compensation: true
inertia_motion_frame: asm_force_sensor_link
inertia_filter_tau_velocity: 0.03
inertia_filter_tau_accel: 0.08
max_linear_accel: 5.0
max_angular_accel: 20.0
max_inertia_force: 30.0
max_inertia_torque: 5.0
```

## 补偿符号约定

现有补偿输出是：

```text
wrench_compensated = wrench_raw - wrench_model
```

惯性补偿应保持同样约定：

```text
wrench_model = wrench_gravity_model + wrench_inertia_model
```

其中整体刚体模型为：

```text
F_rigid^S = m * (g^S - a_C^S)
tau_rigid^S = r_C^S × F_rigid^S
```

因此惯性项本质上是：

```text
F_inertia^S = -m * a_C^S
tau_inertia^S = r_C^S × F_inertia^S
```

这样在静止时惯性项为 0，模型退化为纯重力补偿。

注意：不同力传感器驱动可能存在力方向符号约定差异。当前项目中的重力补偿已经
通过标定文件吸收了符号约定，因此后续实现时应与现有 `predict_dynamic_gravity_wrench`
的输出方向保持一致，不应单独改变传感器原始 wrench 的符号。

## 推荐实现路径

### 第一阶段：在动态重力补偿节点中加入可选惯性项

修改目标：

- `dynamic_gravity_compensation_node.py`
- 新增纯数学模块，例如 `rigid_body_inertia_model.py`
- 新增单元测试
- 新增参数文件字段

原因：

- 当前导纳控制已经使用 `/external_force_torque_wrench_compensated`。
- 在同一个补偿节点中发布重力模型和惯性模型，便于调试。
- 不需要改变导纳控制器接口。

建议新增话题：

```yaml
inertia_wrench_topic: /external_inertia_compensation_wrench_model
```

建议补偿节点输出：

- `wrench_out_topic`：最终补偿后的 wrench。
- `gravity_wrench_topic`：重力模型。
- `inertia_wrench_topic`：惯性模型。
- 可选 `non_contact_wrench_topic`：重力 + 惯性总模型。

### 第二阶段：先只补偿平动惯性

第一版只实现：

```text
F_inertia^S = -m * a_O^S
tau_inertia^S = r_C^S × F_inertia^S
```

先忽略：

```text
alpha^S × r_C^S
omega^S × (omega^S × r_C^S)
```

原因：

- UR 末端扫查运动中，线加减速通常是力波动主因。
- 角加速度由姿态差分得到，噪声通常更大。
- 先补平动项更容易判断是否有效。

验证有效后，再加入完整质心加速度：

```text
a_C^S = a_O^S + alpha^S × r_C^S + omega^S × (omega^S × r_C^S)
```

### 第三阶段：评估是否需要转动惯量项

如果补偿后力分量明显改善，但力矩分量仍随加减速有明显波动，再考虑加入：

```text
tau_rot^S = -I_C^S * alpha^S - omega^S × (I_C^S * omega^S)
```

这需要额外估计或手动配置整体转动惯量：

```yaml
inertia_tensor_com:
  [Ixx, Ixy, Ixz,
   Ixy, Iyy, Iyz,
   Ixz, Iyz, Izz]
```

第一阶段不建议直接加入，避免调参维度过多。

## 参数设计建议

建议在 `gravity_compensation_dynamic_params.yaml` 中增加：

```yaml
dynamic_gravity_compensation_node:
  ros__parameters:
    enable_inertia_compensation: true
    inertia_wrench_topic: /external_inertia_compensation_wrench_model

    # 用哪个 TF frame 估计整体运动；通常就是传感器坐标系。
    inertia_motion_frame: asm_force_sensor_link

    # 使用线加速度项；第一阶段建议 true。
    enable_linear_inertia: true

    # 使用角速度/角加速度导致的质心附加加速度；第一阶段建议 false。
    enable_angular_inertia: false

    # 使用刚体转动惯量力矩项；第一阶段建议 false。
    enable_rotational_inertia_torque: false

    inertia_filter_tau_velocity: 0.03
    inertia_filter_tau_accel: 0.08
    max_linear_accel: 5.0
    max_angular_accel: 20.0
    max_inertia_force: 30.0
    max_inertia_torque: 5.0
```

## 调试和验证方法

### 1. 离线确认静止状态

机械臂静止时：

- `/external_inertia_compensation_wrench_model` 应接近 0。
- `/external_gravity_compensation_wrench_model` 应与现有重力模型一致。
- `/external_force_torque_wrench_compensated` 不应比加入惯性前更差。

### 2. 空载运动验证

让机械臂在无外界接触条件下做一段平滑往复运动，记录：

- 原始 wrench。
- 纯重力补偿后的 wrench。
- 重力 + 惯性补偿后的 wrench。
- 惯性模型 wrench。
- `asm_force_sensor_link` 的估计线加速度。

目标是：在加减速阶段，重力 + 惯性补偿后的 wrench 峰值明显小于纯重力补偿。

### 3. 接触状态验证

在轻微接触状态下验证时，需要注意惯性补偿只应该去掉机器人自身运动导致的非接触
负载，不应消除真实接触力。若发现接触力被明显削弱，通常说明：

- 加速度估计噪声过大；
- 滤波滞后过大导致相位错误；
- `max_inertia_force` 太大；
- 运动 frame 或 COM 位置配置错误。

## 风险和限制

1. **二阶差分噪声大**  
   TF 位姿差分得到加速度会放大噪声，必须滤波和限幅。

2. **滤波会引入相位滞后**  
   滞后过大时，惯性补偿可能在错误时间抵消力，反而增加波动。需要在
   `inertia_filter_tau_accel` 和响应速度之间折中。

3. **接触期间模型不再完全成立**  
   如果工具与环境发生强接触，末端运动受到外力约束，单纯用机器人 TF 差分估计的
   惯性项可能与真实接触动力学耦合。导纳控制中建议对惯性补偿设置限幅。

4. **ASM 被动关节不是严格刚体**  
   本方案假设 ASM 关节相对运动低频且幅度不大。若两个被动关节发生明显快速摆动，
   单刚体惯性模型会漏掉内部相对运动导致的动态力。

5. **质量/质心必须可靠**  
   惯性力和质量成正比，质心错误会直接影响力矩补偿。应优先使用已经验证过的动态
   重力标定结果。

## 推荐结论

最方便的实现方式是：**在现有 `dynamic_gravity_compensation_node` 中加入可选的
整体刚体惯性补偿项**。第一阶段只使用整体质量、整体质心和传感器 frame 的线加速度
补偿平动惯性力；验证有效后，再加入角速度/角加速度导致的质心附加加速度；最后视
力矩残差情况决定是否加入转动惯量项。

这种方案不需要修改导纳控制器，也不需要建立完整 ASM 多刚体动力学模型，适合当前
“ASM 被动关节低频变化、主要波动来自机械臂加减速”的实际情况。

