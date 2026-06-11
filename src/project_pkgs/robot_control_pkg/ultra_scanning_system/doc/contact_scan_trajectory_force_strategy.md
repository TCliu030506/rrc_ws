# Contact Scan Trajectory And Force Control Strategy

本文档总结 `contact_scan_trajectory_node.py` 和
`contact_scan_trajectory_logic.py` 中实现的接触式超声扫查轨迹与力控制策略。

## 1. 总体目标

该模块实现的是一个上层接触扫查状态机。它本身不直接控制机器人关节，也不直接做导纳积分，而是持续发布：

- 名义期望位姿：`/scan/desired_pose`
- 名义期望速度：`/scan/desired_twist`
- 名义期望加速度：`/scan/desired_accel`
- 目标控制 wrench：`/arm_admittance_control/control_wrench`
- 当前状态：`/contact_scan/state`

下游导纳控制器根据名义轨迹、实测外力和目标 wrench 计算柔顺修正后的执行位姿。再经过 pose mux 和 servoL 执行链路发送给机器人。

因此，该模块的职责可以概括为：

1. 按 path-map 生成粗扫查轨迹。
2. 在接触前使用位置控制搜索真实表面。
3. 接触建立后，根据实际接触点修正整条扫查路径。
4. 在接触稳定和正式扫查阶段发布目标恒力 wrench。
5. 在异常力或搜索距离超限时进入故障保持。

## 2. 两条控制链路

### 2.1 名义轨迹链路

名义轨迹通过三个话题发布：

- `/scan/desired_pose`
- `/scan/desired_twist`
- `/scan/desired_accel`

其中：

- `desired_pose` 是状态机当前阶段希望末端到达或保持的几何位姿。
- `desired_twist` 是对应的名义线速度和角速度。
- `desired_accel` 始终发布零加速度 `Accel()`。

轨迹来源有两类：

1. `PathMapTrajectory`
   - 用于 `APPROACH`、`CONTACT_SCAN`、`RETRACT`。
   - 在两点之间进行时间参数化和位姿插值。
   - 速度由相邻发布位姿差分得到。

2. 直接法向搜索命令
   - 用于 `PRE_CONTACT`。
   - 末端沿工具接近方向以 `pre_contact_speed` 低速推进。

### 2.2 目标力链路

目标力通过 `/arm_admittance_control/control_wrench` 发布。

状态机并不直接施加力，而是给导纳控制器一个目标 wrench。导纳控制器使用：

```text
wrench_external - wrench_control
```

作为导纳方程中的力误差来源，从而生成柔顺位移。

不同阶段的目标 wrench 策略不同：

- `APPROACH` / `PRE_CONTACT`
  - 发布当前实测 wrench。
  - 目的是让 `wrench_control` 近似等于 `wrench_external`，抵消导纳外力项，使下游近似退化为位置跟踪。

- `CONTACT_SETTLE` / `CONTACT_SCAN`
  - 发布目标接触力 wrench。
  - 目标力从当前接触力开始按 `force_ramp_rate` 斜坡逼近 `target_contact_force`。

- `RETRACT` / `FAULT`
  - 发布零目标 wrench。

## 3. 坐标系与符号约定

### 3.1 工具接近方向

接近方向由 `compute_approach_axis()` 计算：

```text
approach_axis = tool_z_axis_in_base * sign(approach_axis_sign)
```

当前约定使用工具局部 Z 轴作为探头接近方向。`approach_axis_sign` 用于适配探头实际是沿工具 `+Z` 还是 `-Z` 压向表面。

该方向表达在 base 坐标系下。

### 3.2 法向力

法向力由 `normal_force()` 计算：

```text
fn = force_axis_sign * wrench[force_axis]
```

其中：

- `force_axis` 指定使用 wrench 的 `x`、`y` 或 `z` 分量。
- `force_axis_sign` 用来统一“压紧力增大为正”的约定。

状态机中所有接触阈值判断都基于这个正向法向力 `fn`。

## 4. 状态机总览

`ContactScanState` 包含以下状态：

```text
APPROACH
PRE_CONTACT
CONTACT_SETTLE
CONTACT_SCAN
RETRACT
FINISHED
FAULT
```

典型流程为：

```text
APPROACH
  -> PRE_CONTACT
  -> CONTACT_SETTLE
  -> CONTACT_SCAN
  -> RETRACT
  -> FINISHED
```

任意阶段如果检测到接触力超过安全上限，可进入：

```text
FAULT
```

`PRE_CONTACT` 中如果搜索距离超过上限，也会进入 `FAULT`。

## 5. 各阶段策略

### 5.1 APPROACH：运动到粗路径起点

目标：

将机器人从当前真实末端位姿移动到 path-map 的第一个路径点 `path_points[0]`。

轨迹策略：

- 创建一段 `PathMapTrajectory`。
- 起点是当前实测末端位姿。
- 目标点是 `path_points[0]`。
- 发布插值出的 `desired_pose` 和差分得到的 `desired_twist`。
- `desired_accel` 为零。

力策略：

- 发布当前实测 wrench 作为 `control_wrench`。
- 目的是抵消导纳外力项，使该阶段主要表现为几何位置跟踪。

状态切换：

- 当 approach 轨迹结束后，切换到 `PRE_CONTACT`。

### 5.2 PRE_CONTACT：低速搜索接触面

目标：

沿工具接近方向低速前进，直到力传感器检测到接触。

轨迹策略：

不使用 `PathMapTrajectory`，而是直接计算位姿：

```text
pose = path_points[0].position + search_distance * approach_axis
orientation = path_points[0].orientation
```

其中：

```text
search_distance += pre_contact_speed * dt
```

发布的速度为：

```text
linear = pre_contact_speed * approach_axis
angular = 0
```

力策略：

- 继续发布当前实测 wrench 作为 `control_wrench`。
- 避免未真正进入接触力控制前，导纳主动把接触力调回 0。

接触判定：

当：

```text
fn >= contact_force_threshold
```

认为接触建立。

安全判定：

当：

```text
search_distance >= max_search_distance
```

认为搜索失败，进入 `FAULT`。

### 5.3 接触建立瞬间处理

一旦 `PRE_CONTACT` 检测到接触，状态机不会立刻扫查，而是做一组初始化。

#### 5.3.1 计算真实表面偏移 `delta_c`

接触建立时：

```text
start = path_points[0].position
contact = current_pose.position
```

`delta_c` 是真实接触点相对粗路径起点在接近方向上的投影：

```text
delta_c = dot(contact - start, approach_axis)
```

含义：

视觉或粗路径给出的起点不一定正好在真实表面上。机器人实际接触时的位置与粗路径起点之间，沿探头接近方向的偏差就是 `delta_c`。

这个值后续用于修正整条扫查路径。

#### 5.3.2 初始化目标接触力

`contact_force_ref` 不从 0 开始，而是从当前实测接触力开始：

```text
contact_force_ref = clamp(fn, 0, target_contact_force)
```

这样可以避免进入 `CONTACT_SETTLE` 后目标力突然变成 0，导致导纳控制器主动卸力。

#### 5.3.3 初始化完整 wrench 斜坡

`control_wrench_ref` 保存接触瞬间的完整六维实测 wrench：

```text
control_wrench_ref = latest_wrench
control_wrench_ramp_frame = latest_wrench_frame
```

后续从这个完整 wrench 平滑过渡到目标恒力 wrench。这样非目标轴的力和力矩不会瞬间清零。

#### 5.3.4 锁存接触稳定保持位姿

进入 `CONTACT_SETTLE` 时，保持位姿选择顺序为：

```text
last_published_desired_pose
or last_pre_contact_command_pose
or current_pose
```

优先使用上一帧已经发布到 `/scan/desired_pose` 的位姿，是为了让状态切换前后的 `desired_pose` 连续。

如果直接使用 `current_pose`，可能因为反馈延迟、采样不同步或执行误差导致期望位姿突变。

#### 5.3.5 生成接触修正路径

调用 `apply_contact_offset()`：

```text
contact_path_points = path_points shifted by delta_c along each point's approach axis
```

这一步假设粗路径主要存在整体法向高度误差。每个路径点保持原始姿态，只沿该点工具接近方向平移 `delta_c`。

完成这些初始化后，状态切换到 `CONTACT_SETTLE`。

### 5.4 CONTACT_SETTLE：保持位姿并建立目标力

目标：

不推进扫查路径，先在接触点附近稳定目标接触力。

轨迹策略：

- 保持 `contact_settle_pose`。
- 发布零线速度和零角速度。
- 加速度仍为零。

力策略：

调用 `_publish_target_control_wrench(dt)`：

- `contact_force_ref` 按 `force_ramp_rate` 逐步逼近 `target_contact_force`。
- `control_wrench_ref` 的六个分量分别斜坡过渡到目标 wrench。
- 力分量使用 `force_ramp_rate`。
- 力矩分量使用 `torque_ramp_rate`。

目标 wrench 构造规则：

- 只在 `force_axis` 对应方向填入目标接触力。
- 力符号由 `force_axis_sign` 决定。
- 如果 `zero_torque_rx_ry_enabled` 为真，则同时设置目标 `rx`、`ry` 力矩。

稳定判定：

只有当：

```text
contact_force_ref >= target_contact_force
```

并且：

```text
abs(fn - target_contact_force) <= contact_settle_force_tolerance
```

持续达到：

```text
contact_settle_duration
```

才认为接触稳定。

进入扫查前的路径对齐：

在切换到 `CONTACT_SCAN` 前，会调用 `align_contact_path_start()`。它把接触修正路径整体平移，使路径起点对齐到稳定后的保持位姿。

这一步的目的：

- `delta_c` 主要修正法向误差。
- 如果实际接触位姿与修正路径起点仍有切向偏差，直接开始扫查会导致 `/scan/desired_pose` 跳变。
- 对齐路径起点可以提高状态切换连续性。

### 5.5 CONTACT_SCAN：接触恒力扫查

目标：

沿修正后的接触路径进行扫查，同时维持目标接触力。

轨迹策略：

- 使用 `contact_path_points` 创建 `PathMapTrajectory`。
- 起点为稳定接触位姿。
- 沿接触修正路径依次插值发布 `desired_pose` 和 `desired_twist`。
- `desired_accel` 为零。

力策略：

- 持续调用 `_publish_target_control_wrench(dt)`。
- 导纳控制器根据目标 wrench 和实测 wrench 的差值生成柔顺补偿。

状态切换：

当接触扫查轨迹结束：

1. 记录最终扫查位姿。
2. 调用 `compute_retract_pose()` 生成撤离目标。
3. 创建撤离轨迹。
4. 切换到 `RETRACT`。

### 5.6 RETRACT：撤离

目标：

扫查完成后沿接近方向反向退出。

轨迹策略：

撤离目标由 `compute_retract_pose()` 生成：

```text
retract_position = final_position - retract_distance * approach_axis
retract_orientation = final_orientation
```

使用 `PathMapTrajectory` 从扫查终点移动到撤离目标。

力策略：

- 发布零目标 wrench。
- 不再维持目标接触力。

状态切换：

- 撤离轨迹结束后进入 `FINISHED`。

### 5.7 FINISHED：正常结束

`FINISHED` 表示流程正常完成。

当前实现中进入 `FINISHED` 后，状态机不再继续发布新的 `desired_pose`、`desired_twist`、`desired_accel`，只在循环末尾发布零目标 wrench。

如果下游执行器需要持续保持最后一帧目标位姿，需要由其他保持节点或下游控制器处理。

### 5.8 FAULT：故障保持

进入条件：

- 任意阶段检测到：

```text
fn >= max_contact_force
```

- 或 `PRE_CONTACT` 中搜索距离超过：

```text
max_search_distance
```

行为：

- 发布零目标 wrench。
- 锁存首次进入故障时的当前实测位姿为 `fault_hold_pose`。
- 后续持续发布该保持位姿，速度为零。

锁存故障位姿的目的：

避免在故障状态下持续使用“当前位姿 + 导纳偏移”形成逃逸或漂移运动。

## 6. 纯逻辑函数职责

`contact_scan_trajectory_logic.py` 保存不依赖 ROS 的几何、力阈值和状态逻辑。

主要函数如下：

### `compute_approach_axis()`

根据路径点姿态和 `approach_axis_sign` 计算 base 坐标系下的工具接近方向。

### `apply_contact_offset()`

把 `delta_c` 应用到整条粗路径。

每个路径点沿自身工具接近方向平移，姿态保持不变。

### `align_contact_path_start()`

将接触路径整体平移到稳定接触位姿起点。

第一个点的姿态也对齐到稳定接触姿态，后续点保持原路径姿态。

### `normal_force()`

根据 `force_axis` 和 `force_axis_sign` 提取正向法向力。

### `should_enter_contact()`

判断是否达到初始接触阈值。

```text
normal_force >= contact_force_threshold
```

### `should_fault_by_force()`

判断接触力是否超过安全上限。

```text
normal_force >= max_contact_force
```

### `should_fault_by_search_distance()`

判断预接触搜索距离是否超过允许范围。

```text
search_distance >= max_search_distance
```

### `ramp_toward()`

按最大变化率让当前值逼近目标值。

用于目标接触力和六维目标 wrench 的平滑过渡。

### `compute_retract_pose()`

根据最终扫查位姿和撤离距离，沿接近方向反向生成撤离目标。

## 7. 连续性设计

该状态机中有多处专门用于降低状态切换突变的设计。

### 7.1 位姿连续性

`PRE_CONTACT -> CONTACT_SETTLE`：

- 使用上一帧已发布的 `desired_pose` 作为 `contact_settle_pose`。
- 避免突然切到有延迟或噪声的实测位姿。

`CONTACT_SETTLE -> CONTACT_SCAN`：

- 使用 `align_contact_path_start()` 将扫查路径起点对齐到稳定保持位姿。
- 避免正式扫查第一帧路径点与保持位姿不一致。

`CONTACT_SCAN -> RETRACT`：

- 撤离轨迹从扫查终点开始生成。
- 姿态保持最终扫查姿态。

### 7.2 力连续性

`PRE_CONTACT -> CONTACT_SETTLE`：

- `contact_force_ref` 从当前实测接触力开始，而不是从 0 开始。
- `control_wrench_ref` 从完整实测 wrench 开始。

`CONTACT_SETTLE / CONTACT_SCAN`：

- 目标力按 `force_ramp_rate` 平滑变化。
- 力矩按 `torque_ramp_rate` 平滑变化。

### 7.3 导纳直通阶段

`APPROACH` 和 `PRE_CONTACT` 的目标不是恒力控制，而是位置轨迹执行和低速搜索。因此状态机会发布实测 wrench 作为 control wrench，尽量抵消导纳外力项。

在系统整体架构中，如果导纳控制器启用了冻结或直通模式，也应在这两个状态下让导纳输出贴近名义轨迹，避免未使用导纳输出时内部虚拟位移持续积累。

## 8. 安全策略

当前实现包含两个主要安全条件：

1. 最大接触力保护

```text
normal_force >= max_contact_force
```

任意阶段触发后进入 `FAULT`。

2. 最大搜索距离保护

```text
search_distance >= max_search_distance
```

仅在 `PRE_CONTACT` 中触发，表示没有在允许距离内找到接触面。

进入 `FAULT` 后：

- 清零目标 wrench。
- 固定保持故障位姿。
- 等待人工处理。

## 9. 调试关注点

调试时建议同时观察以下话题：

- `/contact_scan/state`
- `/scan/desired_pose`
- `/scan/desired_twist`
- `/scan/desired_accel`
- `/arm_admittance_control/control_wrench`
- 补偿后实测 wrench 话题
- 导纳输出位姿话题
- servo 最终输入位姿话题

重点检查：

1. `PRE_CONTACT -> CONTACT_SETTLE` 时 `/scan/desired_pose` 是否连续。
2. `CONTACT_SETTLE` 初期目标 wrench 是否从当前实测 wrench 平滑过渡。
3. `CONTACT_SETTLE -> CONTACT_SCAN` 时路径起点是否已对齐。
4. `CONTACT_SCAN -> RETRACT` 时撤离轨迹是否从扫查终点开始。
5. `FAULT` 中是否保持固定目标位姿，而不是跟随当前实测位姿漂移。

## 10. 策略总结

整个控制策略可以简化为：

```text
先按视觉粗路径接近表面
-> 沿工具法向低速搜索真实接触点
-> 用接触点计算粗路径法向偏移 delta_c
-> 修正整条扫查路径
-> 原地建立目标接触力
-> 沿修正路径恒力扫查
-> 扫查结束后反向撤离
```

其中：

- 几何轨迹由 `PathMapTrajectory` 和预接触法向搜索共同生成。
- 真实表面误差通过 `delta_c` 修正。
- 接触力由目标 wrench 和导纳控制器共同实现。
- 状态切换处通过位姿锁存、路径对齐和 wrench 斜坡降低突变。
- 安全条件通过最大接触力和最大搜索距离保护实现。
