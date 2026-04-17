# tus_ui

遥操作超声扫查系统的主端集成启动功能包。

## 1. 功能概述

tus_ui 用于统一启动主端相关节点，包括：

- UI 图形界面节点
- 图像采集节点
- USB 相机节点
- 主端设备状态节点（omni_state）
- 遥操作目标发布节点（teleoperation_control_ui_pub，来自 ur5_rtde_control 包）

在双机部署模式下，tus_ui 还提供主从启动文件，并在 launch 中统一配置 DDS 相关环境变量，方便跨电脑通信。

## 2. 包内启动文件

- tus.launch.py
- tus_master.launch.py
- tus_slave.launch.py

说明：

- tus.launch.py 为历史启动入口，保留兼容。
- 双机部署建议使用 tus_master.launch.py 和 tus_slave.launch.py。

## 3. 系统架构（双机）

推荐角色划分：

- 主端电脑：运行 tus_master.launch.py（UI、相机、主端状态、目标发布）
- 从端电脑：运行 tus_slave.launch.py（机械臂执行与 TCP 状态回传）

核心话题：

- /ur5/target_cmd：主端发给从端的控制与目标位姿
- /ur5/tcp_state：从端回传当前 TCP 位姿

## 4. 依赖与环境

基础环境：

- Ubuntu + ROS 2
- colcon 工作空间
- 两台机器网络互通（建议同一网段）

关键运行依赖（本包已声明）：

- ur5_rtde_control
- omni_common
- usb_cam
- Qt5Core / Qt5Gui / Qt5Widgets
- sensor_msgs / cv_bridge / image_transport

## 5. 构建与安装

在工作空间根目录执行：

```bash
colcon build --packages-select tus_ui ur5_rtde_control
source install/setup.bash
```

建议两台机器都执行同样的构建与 source 操作，保证节点版本一致。

## 6. 启动方式

### 6.1 单机（兼容老入口）

```bash
ros2 launch tus_ui tus.launch.py
```

### 6.2 双机（推荐）

主端电脑：

```bash
ros2 launch tus_ui tus_master.launch.py
```

从端电脑：

```bash
ros2 launch tus_ui tus_slave.launch.py
```

推荐顺序：

- 先启动从端（slave）
- 再启动主端（master）

这样主端初始化和进入遥操作时，更容易及时拿到从端 TCP 反馈，完成映射原点对齐。

## 7. Launch 参数说明

### tus_master.launch.py

- ros_domain_id，默认 10
- ros_localhost_only，默认 0
- target_cmd_topic，默认 /ur5/target_cmd
- tcp_state_topic，默认 /ur5/tcp_state

### tus_slave.launch.py

- ros_domain_id，默认 10
- ros_localhost_only，默认 0
- robot_ip，默认 192.168.1.102
- target_cmd_topic，默认 /ur5/target_cmd
- tcp_state_topic，默认 /ur5/tcp_state
- publish_rate_hz，默认 125.0
- command_timeout_sec，默认 0.2

## 8. 跨电脑运行与 DDS 通信（详细）

### 8.1 必须满足的条件

两台 ROS 2 电脑要互相发现，至少满足：

- ROS_DOMAIN_ID 一致
- ROS_LOCALHOST_ONLY 为 0
- 网络可达，防火墙允许 DDS 所需 UDP 通信

本包的 tus_master.launch.py 和 tus_slave.launch.py 已在 launch 中统一设置：

- ROS_DOMAIN_ID
- ROS_LOCALHOST_ONLY

只要两端启动参数一致，通常不需要再手动 export。

### 8.2 为什么这样配置

- ROS_DOMAIN_ID：类似“通信房间号”，不同 ID 会互相隔离。
- ROS_LOCALHOST_ONLY=0：允许跨主机发现与通信；如果为 1，节点只在本机可见。

### 8.3 标准双机启动流程

在两台电脑分别执行：

1. 进入同一个工作空间并 source。
2. 从端启动 tus_slave.launch.py。
3. 主端启动 tus_master.launch.py。
4. 在任意一端验证话题是否互通。

推荐验证命令：

```bash
ros2 topic list
ros2 topic echo /ur5/tcp_state
ros2 topic echo /ur5/target_cmd
```

### 8.4 常见问题排查

现象：看不到对方节点或话题。

优先检查：

1. 两端 ros_domain_id 参数是否一致。
2. 两端 ros_localhost_only 是否为 0。
3. 两台机器是否在同一网段或路由可达。
4. 是否有防火墙限制 UDP 广播/组播。
5. 两端是否都 source 了正确的 install/setup.bash。

现象：只能看到本机话题。

优先检查：

1. 是否误用了 ROS_LOCALHOST_ONLY=1。
2. 是否启动了错误的 launch 文件或错误参数。

现象：主端遥操作起始对齐不稳定。

优先检查：

1. 从端是否先于主端启动。
2. /ur5/tcp_state 是否持续有数据。
3. target_cmd_topic 与 tcp_state_topic 是否两端一致。

### 8.5 进阶说明

当前采用的是标准 DDS 自动发现，通用且易用。

如果网络环境较复杂（跨网段、组播受限、企业网络严格管控），可进一步考虑：

- CycloneDDS peers 配置
- Fast DDS discovery server

这类方式能把发现过程做成“固定对端”，提高复杂网络下的稳定性。

## 9. 维护建议

- 建议将主从两台机器的 ROS 2 版本与工作空间代码保持一致。
- 建议把关键参数（domain id、topic 名称、robot_ip）纳入团队部署文档统一管理。
- 修改话题名时务必同时修改 master 和 slave，避免静默失联。

## 10. License

待补充。
