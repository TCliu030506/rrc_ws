ur5_rtde_control
=================

概述
----

`ur5_rtde_control` 是一个基于 UR 官方 RTDE 接口的 UR5 机械臂控制 ROS 2 Python 包，用于通过网络与 UR5 控制箱进行实时数据交换和运动控制。

官方文档
--------

- [UR5 RTDE Control 官方文档](https://www.universal-robots.com/articles/ur5/rtde-control-python-interface/)
- [UR5 RTDE Control API 文档](https://sdurobotics.gitlab.io/ur_rtde/api/api.html)

本包主要提供：

- 使用 `rtde_control` / `rtde_receive` 对 UR 机器人进行连接与断开；
- 通过 `servoL` 指令发送 TCP 目标位姿，实现平滑插补运动；
- 获取当前 TCP 位姿等基础状态信息；
- 预留遥操作程序接口，可结合 Phantom Omni 等主端设备进行机械臂遥操作控制（见 `teleoperation_control.py`）。

主要功能
--------

1. URCONTROL 类（文件：`ur5_rtde_control/ur5_rtde_control.py`）

	- 封装 RTDE 接口：
	  - `rtde_control.RTDEControlInterface`
	  - `rtde_receive.RTDEReceiveInterface`
	- 主要方法：
	  - `__init__(robot_ip)`：根据机器人 IP 建立控制与接收接口；
	  - `sevol_l(target_pose)`：向 UR 控制箱发送 `servoL` 指令，其中 `target_pose` 为 `[x, y, z, rx, ry, rz]`；
	  - `get_tcp_pose()`：获取当前 TCP 的位姿（`[x, y, z, rx, ry, rz]`）；
	  - `disconnect()`：断开与机器人的 RTDE 连接。

2. 示例程序入口

	在 `main()` 中给出一个简单示例：

	- 连接到指定 IP 的 UR5；
	- 读取当前 TCP 姿态并打印；
	- 调用 `sevol_l(target_pose)` 发送一个目标位姿；
	- 再次读取并打印运动后的 TCP 姿态；
	- 退出时调用 `disconnect()` 断开连接。

3. 遥操作控制（文件：`ur5_rtde_control/teleoperation_control.py`）

	- 订阅 `omni_msgs/OmniState` 消息（例如话题 `/phantom/state`），接收主端设备的位姿与速度；
	- 使用线程安全的方式缓存主端位姿信息；
	- 通过单独线程以 100 Hz 频率发布机械臂目标位姿（`geometry_msgs/PoseStamped`），实现主从 1:1 或带缩放/变换的映射控制；
	- 可根据需要在该文件中添加坐标系变换、比例缩放、死区等逻辑，实现更复杂的遥操作策略。

环境依赖
--------

- Ubuntu + ROS 2（已配置 `colcon` 构建环境）；
- Python 3；
- UR 官方 RTDE Python 库：
  - `rtde_control`
  - `rtde_receive`
- 一台可通过网络访问的 UR5 机器人控制箱（已配置静态 IP）。

编译与安装
----------

在工作空间根目录（如 `zzrobot_ws`）执行：

.. code-block:: bash

	colcon build --packages-select ur5_rtde_control
	source install/setup.bash

构建成功后，ROS 2 会安装本包的 Python 节点与可执行入口（具体入口名以 `setup.py` 为准）。

使用说明
--------

1. 作为 Python 库直接调用

.. code-block:: python

	from ur5_rtde_control.ur5_rtde_control import URCONTROL

	if __name__ == "__main__":
		 robot_ip = "192.168.1.102"  # 根据实际 UR5 控制箱 IP 修改
		 ur = URCONTROL(robot_ip)

		 # 期望的 TCP 目标位姿 [x, y, z, rx, ry, rz]
		 target_pose = [-0.176, 0.173, 0.583,
							 0.0, 0.0, 0.0]

		 pose = ur.get_tcp_pose()
		 print("初始 TCP 姿态:", pose)

		 ur.sevol_l(target_pose)

		 pose = ur.get_tcp_pose()
		 print("运动后 TCP 姿态:", pose)

		 ur.disconnect()

2. 遥操作节点（示意）

在 `teleoperation_control.py` 中：

- 节点订阅主端设备话题 `/phantom/state`；
- 把接收到的主端位姿映射到机械臂基坐标系下的目标位姿；
- 通过话题（例如 `/ur5_target_pose`）连续发布 `PoseStamped`，供下游控制节点或控制器使用；
- 发布频率默认为 100 Hz，可根据需要修改。

你可以在此基础上：

- 加入坐标变换矩阵，实现主端坐标系 → 机械臂基坐标系的映射；
- 加时间滤波 / 低通滤波，提高操作平滑性；
- 集成夹爪开合、锁定按钮等功能，实现更完整的遥操作系统。

注意事项
--------

- 使用前请确认机器人处于安全工作空间范围内，并已设置合适的速度 / 加速度 / 安全限幅；
- 使用 `servoL` 进行连续插补时，建议在控制回路外侧增加碰撞检测与急停机制；
- 建议在仿真或无负载状态下先验证控制策略，再在实际应用场景中逐步放开限幅参数。

