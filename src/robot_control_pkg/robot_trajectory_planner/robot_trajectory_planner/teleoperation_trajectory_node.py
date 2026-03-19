# 控制信号含义说明（来自话题 tus_control 的 UIControl.control_flag）：
#   1：机器人初始化
#   2：机器人作业准备（ur5 运动到初始位姿）
#   3：开启系统遥操作（根据主端位姿增量持续控制 UR5 运动）
#   4：关闭系统遥操作（暂停主从端位姿增量控制）
#   5：退出系统（程序退出，关闭所有线程、发布者、订阅者、节点）
#   10：系统遥操作中（持续根据主端位姿增量控制 UR5 运动）---本节点内部使用--

import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
import threading
import time
import copy
from geometry_msgs.msg import Pose, Twist, Accel
import math
from ui_control_msg.msg import UiControl


def _quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / n, y / n, z / n, w / n]


def _quat_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_inv(q):
    # unit quaternion inverse
    return _quat_conjugate(q)


def _quat_mul(q1, q2):
    # Hamilton product; matches composition like: R(q1) * R(q2)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quat_from_axis_angle(axis, theta):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    ax /= n
    ay /= n
    az /= n
    half = 0.5 * float(theta)
    s = math.sin(half)
    c = math.cos(half)
    return [ax * s, ay * s, az * s, c]


def _quat_to_rotvec(q):
    # ensure unit
    q = _quat_normalize(q)
    x, y, z, w = q

    # choose canonical sign to keep angle <= pi (helps reduce output jumps)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    v_norm = math.sqrt(x * x + y * y + z * z)
    if v_norm < 1e-12:
        return [0.0, 0.0, 0.0]

    theta = 2.0 * math.atan2(v_norm, w)
    ax = x / v_norm
    ay = y / v_norm
    az = z / v_norm
    return [ax * theta, ay * theta, az * theta]


def _rotvec_to_quat(rvec):
    rx, ry, rz = rvec
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    ax = rx / theta
    ay = ry / theta
    az = rz / theta
    return _quat_from_axis_angle([ax, ay, az], theta)


def _twist_angle_about_axis_from_quat(delta_q, axis):
    """从增量旋转四元数 delta_q 中提取绕给定 axis 的 twist（有符号角度，rad）。

    axis 表达在 delta_q 所在的坐标系内。
    """
    delta_q = _quat_normalize(delta_q)
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return 0.0
    ax /= n
    ay /= n
    az /= n

    vx, vy, vz, w = delta_q
    # 向量部在 axis 上投影
    proj = vx * ax + vy * ay + vz * az
    vpx, vpy, vpz = ax * proj, ay * proj, az * proj
    qt = _quat_normalize([vpx, vpy, vpz, w])

    # qt 对应纯 twist：v = axis*sin(theta/2), w=cos(theta/2)
    sin_half = qt[0] * ax + qt[1] * ay + qt[2] * az
    cos_half = qt[3]
    return 2.0 * math.atan2(sin_half, cos_half)


def _twist_decompose_xyz_from_quat(delta_q):
    """将增量旋转 delta_q 近似分解为绕 x/y/z 三轴的 twist + 残差。

    分解形式（按顺序）：
        delta_q = qx * qy * qz * q_residual

    其中 qx/qy/qz 分别为绕参考坐标系 x/y/z 轴的 twist 四元数；
    q_residual 为剩余旋转（不做缩放时可保留以尽量保持原始姿态）。

    注意：这不是严格的欧拉角分解（因此不会出现典型的万向节锁），
    但它是一种连续、可用于“分轴调手感”的近似分解。
    """
    q = _quat_normalize(delta_q)

    ax_x = [1.0, 0.0, 0.0]
    ax_y = [0.0, 1.0, 0.0]
    ax_z = [0.0, 0.0, 1.0]

    # 依次提取并剥离 x/y/z twist
    ang_x = _twist_angle_about_axis_from_quat(q, ax_x)
    qx = _quat_from_axis_angle(ax_x, ang_x)
    q = _quat_mul(_quat_inv(qx), q)

    ang_y = _twist_angle_about_axis_from_quat(q, ax_y)
    qy = _quat_from_axis_angle(ax_y, ang_y)
    q = _quat_mul(_quat_inv(qy), q)

    ang_z = _twist_angle_about_axis_from_quat(q, ax_z)
    qz = _quat_from_axis_angle(ax_z, ang_z)
    q = _quat_mul(_quat_inv(qz), q)

    q_residual = _quat_normalize(q)
    return ang_x, ang_y, ang_z, q_residual


class Subscriber(Node):

    def __init__(self):
        # 初始化节点
        super().__init__('teleoperation_trajectory_node')

        self.declare_parameter('topic_arm_pose', '/UR5/ee_pose')
        self.declare_parameter('topic_master_state', '/phantom/state')
        self.declare_parameter('topic_ui_control', 'tus_control')
        self.declare_parameter('topic_desired_pose', '/desired_pose')
        self.declare_parameter('topic_desired_twist', '/desired_twist')
        self.declare_parameter('topic_desired_accel', '/desired_accel')
        self.declare_parameter('publish_rate', 125.0)

        # 初始从端位姿（作为映射零点）[x, y, z, rx, ry, rz]
        self.declare_parameter(
            'initial_slave_pose',
            [0.071, -0.50, 0.45, 0.00, math.pi, 0.00]
        )

        # 位置、姿态映射比例（可根据实际手感自行调整）
        self.declare_parameter('pos_scale', [2.0, 2.0, 2.0])
        self.declare_parameter('rot_scale', [0.6, 0.6, 0.6])

        self._topic_arm_pose = self.get_parameter('topic_arm_pose').value
        self._topic_master_state = self.get_parameter('topic_master_state').value
        self._topic_ui_control = self.get_parameter('topic_ui_control').value
        self._topic_desired_pose = self.get_parameter('topic_desired_pose').value
        self._topic_desired_twist = self.get_parameter('topic_desired_twist').value
        self._topic_desired_accel = self.get_parameter('topic_desired_accel').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)

        self._initial_pose_param = [float(v) for v in self.get_parameter('initial_slave_pose').value]
        self._pos_scale = [float(v) for v in self.get_parameter('pos_scale').value]
        self._rot_scale = [float(v) for v in self.get_parameter('rot_scale').value]

        if len(self._initial_pose_param) != 6:
            raise ValueError('initial_slave_pose must be length 6')
        if len(self._pos_scale) != 3:
            raise ValueError('pos_scale must be length 3')
        if len(self._rot_scale) != 3:
            raise ValueError('rot_scale must be length 3')
        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')

        self._dt = 1.0 / self._publish_rate

        # 与 AdmittanceController.cpp 一致：从话题订阅机械臂当前位姿
        self.subscription_arm_pose = self.create_subscription(
            Pose,
            self._topic_arm_pose,
            self.handle_arm_pose,
            10
        )

        # 创建主端机器人订阅者
        self.subscription = self.create_subscription(
            OmniState,
            self._topic_master_state,
            self.handle_msg,
            10
        )
        # 创建订阅者，用于接收控制遥操作开始/关闭的控制信号
        self.subscription_ui = self.create_subscription(
            UiControl,
            self._topic_ui_control,
            self.handle_msg_ui,
            10
        )

        self.pub_desired_pose = self.create_publisher(Pose, self._topic_desired_pose, 10)
        self.pub_desired_twist = self.create_publisher(Twist, self._topic_desired_twist, 10)
        self.pub_desired_accel = self.create_publisher(Accel, self._topic_desired_accel, 10)
        # 存储遥操作控制信号变量（1~5 外加内部使用的 10）
        self._control_signal = None
        # 发布线程运行标志（用于退出）
        self._running = True

        # 存储主端状态（在回调中更新，在发布线程中读取）变量
        self._lock = threading.Lock()
        self._master_position = None
        self._master_orientation = None
        self._master_velocity = None
        self._master_pose = None
        self._master_quat = None  # [qx, qy, qz, qw]（已归一化、已连续化）

        # 机械臂实时位姿（由 /UR5/ee_pose 更新）
        self._arm_pose_realtime = None  # [x, y, z, rx, ry, rz]
        self._arm_pose_ready = False

        # 存储从端初始位姿与目标位姿变量
        self._initial_pose = None  # [x, y, z, rx, ry, rz]
        self._target_pose = None

        # 主端“零点”位姿（第一次进入遥操作时的参考位姿，用于做增量）
        self._ref_master_pose = None  # [x, y, z, rx, ry, rz]
        self._ref_master_quat = None  # [qx, qy, qz, qw]

        # 为避免“角度突变”，用相邻帧四元数做相对旋转，并累计“绕y轴”的分量
        self._prev_master_quat = None
        self._last_master_quat = None
        self._cum_master_y = 0.0

        # 当前参考目标位姿、上一时刻状态（用于速度和加速度计算）
        self._initial_pose = list(self._initial_pose_param)
        self._target_pose = list(self._initial_pose_param)
        self._prev_target_pose = None
        self._prev_twist = [0.0] * 6

        # 启动 125Hz 发布线程（根据控制信号决定是否执行遥操作）
        self._pub_thread = threading.Thread(
            target=self._publish_target_pose_loop,
            daemon=True
        )
        self._pub_thread.start()

    def handle_msg(self, msg: OmniState):
        # 存储接收到的数据，供发布线程使用
        with self._lock:
            self._master_position = copy.deepcopy(msg.pose.position)
            self._master_orientation = copy.deepcopy(msg.pose.orientation)
            self._master_velocity = copy.deepcopy(msg.velocity)
            # 将单位由 mm 转换为 m，并转换为 UR 风格的旋转向量
            self._master_position.x /= 1000.0
            self._master_position.y /= 1000.0
            self._master_position.z /= 1000.0
            self._master_velocity.x /= 1000.0
            self._master_velocity.y /= 1000.0
            self._master_velocity.z /= 1000.0

            # 主端四元数：先归一化，再做符号连续化（q 与 -q 等价，但会导致 angle/rotvec 跳变）
            q = _quat_normalize([
                float(self._master_orientation.x),
                float(self._master_orientation.y),
                float(self._master_orientation.z),
                float(self._master_orientation.w),
            ])
            if self._prev_master_quat is not None and _quat_dot(q, self._prev_master_quat) < 0.0:
                q = [-q[0], -q[1], -q[2], -q[3]]
            self._prev_master_quat = q
            self._master_quat = q

            # 用稳定的 quat->rotvec 生成 master_pose（供其它映射用）
            rx, ry, rz = _quat_to_rotvec(q)
            self._master_pose = [
                float(self._master_position.x),
                float(self._master_position.y),
                float(self._master_position.z),
                rx, ry, rz,
            ]
            # self.get_logger().info(
            #     f'Received Master Pose: '
            #     f'x={self._master_pose[0]:.3f}, y={self._master_pose[1]:.3f}, '
            #     f'z={self._master_pose[2]:.3f}, rx={self._master_pose[3]:.3f}, '
            #     f'ry={self._master_pose[4]:.3f}, rz={self._master_pose[5]:.3f}'
            # )

    def handle_arm_pose(self, msg: Pose):
        # 机械臂实时位姿：与 AdmittanceController.cpp 一样由 Pose 话题获取
        with self._lock:
            self._arm_pose_realtime = self._xyzquat_to_xyzrxryrz(msg.position, msg.orientation)
            self._arm_pose_ready = True

    def _set_initial_pose_from_realtime(self):
        # 优先使用实时机械臂位姿作为映射零点；未就绪时回退到参数默认值
        if self._arm_pose_ready and self._arm_pose_realtime is not None:
            self._initial_pose = list(self._arm_pose_realtime)
        else:
            self._initial_pose = list(self._initial_pose_param)
            self.get_logger().warn(
                'Arm realtime pose not ready, fallback to initial_slave_pose parameter.'
            )
        self._target_pose = list(self._initial_pose)
        self._prev_target_pose = None
        self._prev_twist = [0.0] * 6

    def handle_msg_ui(self, msg: UiControl):
        """根据 UI 发送的控制信号（control_flag）执行不同功能。"""
        # 读取 control_flag 作为控制信号
        signal = int(msg.control_flag)
        with self._lock:
            self._control_signal = signal
        self.get_logger().info(f'Received Control Signal: {self._control_signal}')

        # 分发到具体功能
        self._handle_control_signal(signal)

    def _handle_control_signal(self, signal: int):
        """实现 1~5 控制命令的具体功能。"""
        if signal == 1:
            # 1：机器人初始化（重置映射参考状态）
            self.get_logger().info('Command 1: 机器人初始化（重置映射参考状态）')
            with self._lock:
                self._set_initial_pose_from_realtime()
                self._ref_master_pose = None
                self._ref_master_quat = None
            self._publish_desired_trajectory(self._target_pose)

        elif signal == 2:
            # 2：机器人作业准备（将期望轨迹重置到初始位姿）
            self.get_logger().info('Command 2: 机器人作业准备（重置期望轨迹初始位姿）')
            with self._lock:
                self._set_initial_pose_from_realtime()
            self._publish_desired_trajectory(self._target_pose)

        elif signal == 3:
            # 3：开启系统遥操作（开始主从增量映射）
            self.get_logger().info('Command 3: 开启系统遥操作')
            with self._lock:
                # 重新设定参考位姿，在 _map_master_to_slave_pose 中会使用
                self._ref_master_pose = None
                self._ref_master_quat = None
                # 不清空 _initial_pose，这样可以保留之前的初始从端位姿

        elif signal == 4:
            # 4：关闭系统遥操作（暂停主从端位姿增量控制）
            self.get_logger().info('Command 4: 关闭系统遥操作')
            with self._lock:
                # 将控制信号保持为 4，发布线程会停止发送目标位姿
                # 清空参考位姿，下一次重新开启时重新标定零点
                self._ref_master_pose = None
                self._ref_master_quat = None

        elif signal == 5:
            # 5：退出系统（程序退出，关闭所有线程、发布者、订阅者、节点）
            self.get_logger().info('Command 5: 退出系统')
            # 停止发布线程循环
            self._running = False
            # 调用 rclpy.shutdown，使 spin 退出
            rclpy.shutdown()

        else:
            # 其他控制信号（包括内部使用的 10）不在此处单独处理
            self.get_logger().debug(f'Unhandled control signal in dispatcher: {signal}')

    def _publish_target_pose_loop(self):
        """125Hz 发布线程：根据控制信号决定是否执行遥操作。"""
        period = self._dt  # 125 Hz
        self.get_logger().info('Publish thread started.')
        while rclpy.ok() and self._running:
            # 拷贝当前控制信号和主端位姿
            with self._lock:
                control = self._control_signal
                master_pose = copy.deepcopy(self._master_pose)
                master_quat = copy.deepcopy(self._master_quat)

            # 仅在 control_signal 为 3 或 10 时执行遥操作
            if control not in (3, 10):
                time.sleep(period)
                continue

            if master_pose is not None and master_quat is not None:
                # # 默认：只映射主端绕自身y轴 -> 从端TCP绕自身y轴（用于旋转实验）
                # target_pose = self._map_master_to_slave_pose_rotatey(master_quat)

                # 如需对比“6DoF 四元数增量映射”和“角轴分量增量映射”，可切换为：
                # target_pose = self._map_master_to_slave_pose(master_pose)
                # target_pose = self._map_master_to_slave_pose_quat(master_pose, master_quat)
                target_pose = self._map_master_to_slave_pose_quat_twist_xyz(master_pose, master_quat)
                self._target_pose = target_pose
                self._publish_desired_trajectory(target_pose)

            time.sleep(period)

        self.get_logger().info('Publish thread stopped.')

    def _map_master_to_slave_pose(self, master_pose):
        """
        基于“增量式映射”的主从位姿映射函数。

        输入:
            master_pose: [x, y, z, rx, ry, rz]
                         主端当前位姿（已转换为 UR 风格的旋转向量）

        输出:
            target_pose: [x, y, z, rx, ry, rz]
                         从端（UR5）目标位姿
        """
        # 如果收到“开启遥操作”（3），则在第一次进入时设置初始位姿并切到 10 状态
        if self._control_signal == 3:
            self.get_logger().info('Control started: Initial pose set.')
            self._set_initial_pose_from_realtime()
            self._ref_master_pose = None  # 重置主端参考位姿
            # 更新为 10 表示“遥操作进行中”
            self._control_signal = 10

        # 确保从端初始位姿存在
        if self._initial_pose is None:
            self._set_initial_pose_from_realtime()

        # 首帧：记录主端参考位姿，作为“零点”
        if self._ref_master_pose is None:
            self._ref_master_pose = master_pose
            # 首帧不移动，从端保持在初始位姿
            return self._initial_pose

        # 计算主端相对参考位姿的增量
        dx = master_pose[0] - self._ref_master_pose[0]
        dy = master_pose[1] - self._ref_master_pose[1]
        dz = master_pose[2] - self._ref_master_pose[2]
        drx = master_pose[3] - self._ref_master_pose[3]
        dry = master_pose[4] - self._ref_master_pose[4]
        drz = master_pose[5] - self._ref_master_pose[5]

        # 应用比例系数
        sx, sy, sz = self._pos_scale
        srx, sry, srz = self._rot_scale

        dx *= sx
        dy *= sy
        dz *= sz
        drx *= srx
        dry *= sry
        drz *= srz

        # 叠加到从端初始位姿，得到目标位姿
        target_pose = [
            self._initial_pose[0] + dx,
            self._initial_pose[1] + dy,
            self._initial_pose[2] + dz,
            self._initial_pose[3] + drx,
            self._initial_pose[4] + dry,
            self._initial_pose[5] + drz,
        ]

        return target_pose

    def _map_master_to_slave_pose_rotatey(self, master_quat):
        """仅映射主端绕自身y轴旋转到从端TCP绕自身y轴旋转。

        关键点：
        - 不使用 rotvec 分量差（会在 pi 附近跳变）
        - 使用相邻帧四元数相对旋转提取“绕y轴”的 twist 分量并累计
        """
        if self._control_signal == 3:
            self.get_logger().info('Control started: Initial pose set.')
            self._set_initial_pose_from_realtime()
            self._control_signal = 10
            self._last_master_quat = None
            self._cum_master_y = 0.0

        if self._initial_pose is None:
            self._set_initial_pose_from_realtime()

        # 首帧：设定累计起点
        q_now = _quat_normalize(master_quat)
        if self._last_master_quat is None:
            self._last_master_quat = q_now
            return self._initial_pose

        q_last = self._last_master_quat
        self._last_master_quat = q_now

        # delta = last^{-1} * now：表达在“上一帧（主端自身）坐标系”下的相对旋转
        delta_q = _quat_mul(_quat_inv(q_last), q_now)

        # 只提取绕主端自身 y 轴的增量角，并累计
        step_y = _twist_angle_about_axis_from_quat(delta_q, [0.0, 1.0, 0.0])
        self._cum_master_y += step_y

        # 只用 rot_scale 的 y 作为映射比例
        _, sry, _ = self._rot_scale
        theta_slave = self._cum_master_y * float(sry)

        return self.rotate_about_tcp_y(self._initial_pose, theta_slave)

    def _map_master_to_slave_pose_quat(self, master_pose, master_quat):
        """6DoF 增量式映射（基于四元数/指数对数映射），用于对比角轴分量叠加方式。

        - 位置：仍按 master_pose 相对参考点的增量做线性比例映射
        - 姿态：用 delta_q = inv(q_ref) * q_now 得到相对旋转
              再用 log 映射得到 rotvec，在“参考帧(主端零点)坐标系”下做分轴缩放
              最后 exp 回四元数并右乘到从端初始姿态 q_slave0 上

        说明：这里的分轴缩放是在 delta 的旋转向量坐标（参考帧）上进行，
        它并不等价于严格的 yaw/pitch/roll 缩放，但比直接对 rx/ry/rz 分量做差更连续。
        """
        if self._control_signal == 3:
            self.get_logger().info('Control started: Initial pose set.')
            self._set_initial_pose_from_realtime()
            self._ref_master_pose = None
            self._ref_master_quat = None
            self._control_signal = 10

        if self._initial_pose is None:
            self._set_initial_pose_from_realtime()

        # 首帧：建立主端参考点（位置 + 姿态）
        if self._ref_master_pose is None or self._ref_master_quat is None:
            self._ref_master_pose = master_pose
            self._ref_master_quat = _quat_normalize(master_quat)
            return self._initial_pose

        # 平移增量
        dx = float(master_pose[0] - self._ref_master_pose[0])
        dy = float(master_pose[1] - self._ref_master_pose[1])
        dz = float(master_pose[2] - self._ref_master_pose[2])
        sx, sy, sz = self._pos_scale
        dx *= float(sx)
        dy *= float(sy)
        dz *= float(sz)

        # 姿态增量：delta_q = inv(q_ref) * q_now（在参考帧坐标下表达）
        q_ref = _quat_normalize(self._ref_master_quat)
        q_now = _quat_normalize(master_quat)
        delta_q = _quat_mul(_quat_inv(q_ref), q_now)

        # 分轴缩放：log(delta_q) -> rotvec -> scale -> exp
        delta_rv = _quat_to_rotvec(delta_q)
        srx, sry, srz = self._rot_scale
        delta_rv_scaled = [
            float(delta_rv[0]) * float(srx),
            float(delta_rv[1]) * float(sry),
            float(delta_rv[2]) * float(srz),
        ]
        delta_q_scaled = _rotvec_to_quat(delta_rv_scaled)

        # 叠加到从端初始姿态：右乘表示在从端 TCP（自身）坐标下追加相对旋转
        q_slave0 = _rotvec_to_quat([
            float(self._initial_pose[3]),
            float(self._initial_pose[4]),
            float(self._initial_pose[5]),
        ])
        q_slave_target = _quat_mul(q_slave0, delta_q_scaled)
        rx_t, ry_t, rz_t = _quat_to_rotvec(q_slave_target)

        return [
            float(self._initial_pose[0]) + dx,
            float(self._initial_pose[1]) + dy,
            float(self._initial_pose[2]) + dz,
            rx_t,
            ry_t,
            rz_t,
        ]

    def _map_master_to_slave_pose_quat_twist_xyz(self, master_pose, master_quat):
        """6DoF 增量式映射（基于四元数，先做 x/y/z 三轴 twist 分解，再分轴缩放并重构）。

        和 _map_master_to_slave_pose_quat 的区别：
        - _map_master_to_slave_pose_quat：对 log(delta_q) 的 rotvec 分量缩放（连续但不是真正 yaw/pitch/roll）
        - 本函数：对 delta_q 做 x/y/z twist 分解后分别缩放，再重构（更接近“按轴调手感”）

        分解与重构：
            delta_q = qx * qy * qz * q_residual
            delta_q_scaled = qx(srx*ax) * qy(sry*ay) * qz(srz*az) * q_residual

        其中 q_residual 保留不缩放，用于尽量维持原始姿态（避免丢失非纯轴扭转分量）。
        """
        if self._control_signal == 3:
            self.get_logger().info('Control started: Initial pose set.')
            self._set_initial_pose_from_realtime()
            self._ref_master_pose = None
            self._ref_master_quat = None
            self._control_signal = 10

        if self._initial_pose is None:
            self._set_initial_pose_from_realtime()

        # 首帧：建立主端参考点（位置 + 姿态）
        if self._ref_master_pose is None or self._ref_master_quat is None:
            self._ref_master_pose = master_pose
            self._ref_master_quat = _quat_normalize(master_quat)
            return self._initial_pose

        # 平移增量
        dx = float(master_pose[0] - self._ref_master_pose[0])
        dy = float(master_pose[1] - self._ref_master_pose[1])
        dz = float(master_pose[2] - self._ref_master_pose[2])
        sx, sy, sz = self._pos_scale
        dx *= float(sx)
        dy *= float(sy)
        dz *= float(sz)

        # 姿态增量：delta_q = inv(q_ref) * q_now（在参考帧坐标下表达）
        q_ref = _quat_normalize(self._ref_master_quat)
        q_now = _quat_normalize(master_quat)
        delta_q = _quat_mul(_quat_inv(q_ref), q_now)

        # twist 分解 -> 分轴缩放 -> 重构
        ang_x, ang_y, ang_z, q_residual = _twist_decompose_xyz_from_quat(delta_q)
        srx, sry, srz = self._rot_scale
        qx_s = _quat_from_axis_angle([1.0, 0.0, 0.0], float(ang_x) * float(srx))
        qy_s = _quat_from_axis_angle([0.0, 1.0, 0.0], float(ang_y) * float(sry))
        qz_s = _quat_from_axis_angle([0.0, 0.0, 1.0], float(ang_z) * float(srz))
        delta_q_scaled = _quat_mul(_quat_mul(_quat_mul(qx_s, qy_s), qz_s), q_residual)

        # 叠加到从端初始姿态：右乘表示在从端 TCP（自身）坐标下追加相对旋转
        q_slave0 = _rotvec_to_quat([
            float(self._initial_pose[3]),
            float(self._initial_pose[4]),
            float(self._initial_pose[5]),
        ])
        q_slave_target = _quat_mul(q_slave0, delta_q_scaled)
        rx_t, ry_t, rz_t = _quat_to_rotvec(q_slave_target)

        return [
            float(self._initial_pose[0]) + dx,
            float(self._initial_pose[1]) + dy,
            float(self._initial_pose[2]) + dz,
            rx_t,
            ry_t,
            rz_t,
        ]

    def rotate_about_tcp_y(self, current_pose, theta):
        """在 current_pose 的姿态基础上，绕 TCP 自身 y 轴旋转 theta（rad）。"""
        q0 = _rotvec_to_quat([float(current_pose[3]), float(current_pose[4]), float(current_pose[5])])
        qd = _quat_from_axis_angle([0.0, 1.0, 0.0], float(theta))
        # 右乘：在 TCP（自身）坐标系下追加旋转
        q_new = _quat_mul(q0, qd)
        rx, ry, rz = _quat_to_rotvec(q_new)
        return [
            float(current_pose[0]),
            float(current_pose[1]),
            float(current_pose[2]),
            rx, ry, rz,
        ]

    def _xyzquat_to_xyzrxryrz(self, position, orientation):
        """
        将 (x, y, z + 四元数) 转换为 (x, y, z, rx, ry, rz)
        position: geometry_msgs.msg.Point
        orientation: geometry_msgs.msg.Quaternion
        返回: [x, y, z, rx, ry, rz]，其中 rx, ry, rz 为旋转向量 (轴 * 角度, 单位: rad)
        """
        x = position.x
        y = position.y
        z = position.z

        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # 归一化四元数，避免数值误差
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            # 无效四元数，退化为零旋转
            return [x, y, z, 0.0, 0.0, 0.0]
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        # 四元数 -> 旋转向量（用 atan2，数值更稳；并在 _quat_to_rotvec 内做了符号规范）
        rx, ry, rz = _quat_to_rotvec([qx, qy, qz, qw])

        return [x, y, z, rx, ry, rz]

    def _pose6_to_pose_msg(self, pose6):
        pose_msg = Pose()
        pose_msg.position.x = float(pose6[0])
        pose_msg.position.y = float(pose6[1])
        pose_msg.position.z = float(pose6[2])
        q = _rotvec_to_quat([float(pose6[3]), float(pose6[4]), float(pose6[5])])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
        return pose_msg

    def _compute_twist_accel(self, pose6):
        # 线速度：位置差分；角速度：相邻姿态差分四元数转旋转向量后除 dt
        if self._prev_target_pose is None:
            twist6 = [0.0] * 6
            accel6 = [0.0] * 6
            return twist6, accel6

        prev = self._prev_target_pose
        dt = self._dt
        eps = 1e-9
        if dt < eps:
            dt = eps

        twist6 = [0.0] * 6
        for i in range(3):
            twist6[i] = (float(pose6[i]) - float(prev[i])) / dt

        q_prev = _rotvec_to_quat([float(prev[3]), float(prev[4]), float(prev[5])])
        q_now = _rotvec_to_quat([float(pose6[3]), float(pose6[4]), float(pose6[5])])
        dq = _quat_mul(_quat_inv(q_prev), q_now)
        omega_rv = _quat_to_rotvec(dq)
        twist6[3] = omega_rv[0] / dt
        twist6[4] = omega_rv[1] / dt
        twist6[5] = omega_rv[2] / dt

        accel6 = [0.0] * 6
        for i in range(6):
            accel6[i] = (twist6[i] - self._prev_twist[i]) / dt

        return twist6, accel6

    def _publish_desired_trajectory(self, pose6):
        twist6, accel6 = self._compute_twist_accel(pose6)

        pose_msg = self._pose6_to_pose_msg(pose6)

        twist_msg = Twist()
        twist_msg.linear.x = float(twist6[0])
        twist_msg.linear.y = float(twist6[1])
        twist_msg.linear.z = float(twist6[2])
        twist_msg.angular.x = float(twist6[3])
        twist_msg.angular.y = float(twist6[4])
        twist_msg.angular.z = float(twist6[5])

        accel_msg = Accel()
        accel_msg.linear.x = float(accel6[0])
        accel_msg.linear.y = float(accel6[1])
        accel_msg.linear.z = float(accel6[2])
        accel_msg.angular.x = float(accel6[3])
        accel_msg.angular.y = float(accel6[4])
        accel_msg.angular.z = float(accel6[5])

        self.pub_desired_pose.publish(pose_msg)
        self.pub_desired_twist.publish(twist_msg)
        self.pub_desired_accel.publish(accel_msg)

        self._prev_target_pose = list(pose6)
        self._prev_twist = list(twist6)


def main(args=None):
    rclpy.init(args=args)
    master_sub = Subscriber()
    rclpy.spin(master_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()