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
import math
from ui_control_msg.msg import UiControl
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray


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

    _TELEOP_SIGNALS = (3, 10)
    _TRANSLATION_POS_SCALE = [0.0, 1.5, 0.0]
    _TRANSLATION_ROT_SCALE = [0.0, 0.0, 0.0]
    _ROTATION_POS_SCALE = [0.0, 0.0, 0.0]
    _ROTATION_ROT_SCALE = [0.0, 0.5, 0.0]

    def __init__(self):
        # 初始化节点
        super().__init__('Aimooe_coordinate_transfer')
        # 发布目标位姿（供执行节点订阅后控制机械臂）
        self.declare_parameter('target_cmd_topic', '/ur5/target_cmd')
        self.declare_parameter('tcp_state_topic', '/ur5/tcp_state')
        self.declare_parameter(
            'initial_slave_pose',
            [0.071, -0.50, 0.45, 0.00, math.pi, 0.00]
        )
        self._initial_pose_default = [
            float(v) for v in self.get_parameter('initial_slave_pose').value
        ]
        target_cmd_topic = self.get_parameter('target_cmd_topic').value
        tcp_state_topic = self.get_parameter('tcp_state_topic').value
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._target_cmd_pub = self.create_publisher(Float64MultiArray, target_cmd_topic, qos_profile)
        self.get_logger().info(f'Publish target command topic: {target_cmd_topic}')
        self._tcp_state_sub = self.create_subscription(
            Float64MultiArray,
            tcp_state_topic,
            self._handle_tcp_state,
            qos_profile,
        )
        self.get_logger().info(f'Subscribe tcp state topic: {tcp_state_topic}')
        # 创建主端机器人订阅者
        self.subscription = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.handle_msg,
            10
        )
        # 创建订阅者，用于接收控制遥操作开始/关闭的控制信号
        self.subscription_ui = self.create_subscription(
            UiControl,
            'tus_control',
            self.handle_msg_ui,
            10
        )
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
        self._latest_slave_tcp_pose = None  # [x, y, z, rx, ry, rz]

        # 存储从端初始位姿与目标位姿变量
        self._initial_pose = list(self._initial_pose_default)  # [x, y, z, rx, ry, rz]
        self._target_pose = None

        # 主端“零点”位姿（第一次进入遥操作时的参考位姿，用于做增量）
        self._ref_master_pose = None  # [x, y, z, rx, ry, rz]
        self._ref_master_quat = None  # [qx, qy, qz, qw]

        # 为避免“角度突变”，用相邻帧四元数做相对旋转，并累计“绕y轴”的分量
        self._prev_master_quat = None
        self._last_master_quat = None
        self._cum_master_y = 0.0

        # 位置、姿态映射比例（可根据实际手感自行调整）
        # self._pos_scale = [2.0, 2.0, 2.0]      # X、Y、Z 比例
        # self._rot_scale = [0.6, 0.6, 0.6]      # RX、RY、RZ 比例
        # 仅保留y轴
        self._pos_scale = [0.0, 1.5, 0.0]      # X、Y、Z 比例
        self._rot_scale = [0.0, 0.0, 0.0]      # RX、RY、RZ 比例

        # 作业准备位姿（UR5 初始位姿），支持参数配置
        self.declare_parameter(
            'job_ready_pose',
            [0.071, -0.50, 0.45, 0.00, math.pi, 0.00]  # 示例位姿，根据实际情况修改
        )
        self._job_ready_pose = self.get_parameter('job_ready_pose').value

        self._signal_handlers = {
            1: self._handle_signal_init,
            2: self._handle_signal_job_ready,
            3: self._handle_signal_start,
            4: self._handle_signal_pause,
            5: self._handle_signal_exit,
            6: self._handle_signal_translation_mode,
            7: self._handle_signal_rotation_mode,
        }

        # 启动 125Hz 发布线程（根据控制信号决定是否执行遥操作）
        self._pub_thread = threading.Thread(
            target=self._publish_target_pose_loop,
            daemon=True
        )
        self._pub_thread.start()

    def _handle_tcp_state(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return

        tcp_pose = [
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]),
            float(msg.data[4]),
            float(msg.data[5]),
        ]
        with self._lock:
            self._latest_slave_tcp_pose = tcp_pose

    def _reset_initial_pose_from_feedback(self):
        """优先使用从端反馈TCP对齐初始位姿，缺失时回退到参数默认值。"""
        with self._lock:
            latest_tcp_pose = None if self._latest_slave_tcp_pose is None else list(self._latest_slave_tcp_pose)
            if latest_tcp_pose is not None and len(latest_tcp_pose) == 6:
                self._initial_pose = latest_tcp_pose
                aligned = True
            else:
                self._initial_pose = list(self._initial_pose_default)
                aligned = False

        if aligned:
            self.get_logger().info('Control started: Initial pose aligned from slave tcp state.')
        else:
            self.get_logger().warn('Slave tcp state not ready, fallback to parameter initial_slave_pose.')

    def _clear_master_reference_locked(self):
        self._ref_master_pose = None
        self._ref_master_quat = None

    def _current_output_pose_locked(self):
        return list(self._target_pose or self._initial_pose)

    def _publish_current_pose_command(self, signal: int):
        with self._lock:
            output_pose = self._current_output_pose_locked()
        self._publish_command(signal, output_pose)

    def _set_mapping_mode(self, pos_scale, rot_scale):
        with self._lock:
            self._pos_scale = list(pos_scale)
            self._rot_scale = list(rot_scale)
            self._clear_master_reference_locked()
            self._control_signal = 3  # 切回遥操作运行态（如果之前在 4/10 状态）

    def _handle_signal_init(self):
        self.get_logger().info('Command 1: 机器人初始化（重置内部状态）')
        self._reset_initial_pose_from_feedback()
        with self._lock:
            self._clear_master_reference_locked()
            output_pose = self._current_output_pose_locked()
        self._publish_command(1, output_pose)

    def _handle_signal_job_ready(self):
        self.get_logger().info('Command 2: 机器人作业准备（发布准备位姿）')
        ready_pose = [float(v) for v in self._job_ready_pose]
        with self._lock:
            self._initial_pose = list(ready_pose)
            self._clear_master_reference_locked()
        self._publish_command(2, ready_pose)
        self.get_logger().info(f'Published job_ready_pose: {ready_pose}')

    def _handle_signal_start(self):
        self.get_logger().info('Command 3: 开启系统遥操作')
        with self._lock:
            self._clear_master_reference_locked()
        self._publish_current_pose_command(3)

    def _handle_signal_pause(self):
        self.get_logger().info('Command 4: 关闭系统遥操作')
        with self._lock:
            self._clear_master_reference_locked()
        self._publish_current_pose_command(4)

    def _handle_signal_exit(self):
        self.get_logger().info('Command 5: 退出系统')
        self._publish_current_pose_command(5)
        self._running = False
        rclpy.shutdown()

    def _handle_signal_translation_mode(self):
        self.get_logger().info('Command 6: 调整映射比例到平移：仅映射y轴')
        self._set_mapping_mode(self._TRANSLATION_POS_SCALE, self._TRANSLATION_ROT_SCALE)
        self._publish_current_pose_command(6)

    def _handle_signal_rotation_mode(self):
        self.get_logger().info('Command 7: 调整映射比例到旋转：仅映射绕y轴旋转')
        self._set_mapping_mode(self._ROTATION_POS_SCALE, self._ROTATION_ROT_SCALE)
        self._publish_current_pose_command(7)

    def _update_master_state_from_msg(self, msg: OmniState):
        with self._lock:
            self._master_position = copy.deepcopy(msg.pose.position)
            self._master_orientation = copy.deepcopy(msg.pose.orientation)
            self._master_velocity = copy.deepcopy(msg.velocity)
            self._master_position.x /= 1000.0
            self._master_position.y /= 1000.0
            self._master_position.z /= 1000.0
            self._master_velocity.x /= 1000.0
            self._master_velocity.y /= 1000.0
            self._master_velocity.z /= 1000.0

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

            rx, ry, rz = _quat_to_rotvec(q)
            self._master_pose = [
                float(self._master_position.x),
                float(self._master_position.y),
                float(self._master_position.z),
                rx,
                ry,
                rz,
            ]

    def handle_msg(self, msg: OmniState):
        # 存储接收到的数据，供发布线程使用
        self._update_master_state_from_msg(msg)

    def handle_msg_ui(self, msg: UiControl):
        """根据 UI 发送的控制信号（control_flag）执行不同功能。"""
        # 读取 control_flag 作为控制信号
        signal = int(msg.control_flag)
        with self._lock:
            # 6/7 仅用于切换映射比例，不覆盖遥操作运行态（3/10）
            if signal not in (6, 7):
                self._control_signal = signal
            current_control = self._control_signal
        self.get_logger().info(
            f'Received Control Signal: {signal}, current control state: {current_control}'
        )

        # 分发到具体功能
        self._handle_control_signal(signal)

    def _handle_control_signal(self, signal: int):
        """实现 1~5 控制命令的具体功能。"""
        handler = self._signal_handlers.get(signal)
        if handler is None:
            self.get_logger().debug(f'Unhandled control signal in dispatcher: {signal}')
            return
        handler()

    def _publish_target_pose_loop(self):
        """125Hz 发布线程：根据控制信号决定是否执行遥操作。"""
        period = 0.008  # 125 Hz
        self.get_logger().info('Publish thread started.')
        while rclpy.ok() and self._running:
            # 拷贝当前控制信号和主端位姿
            with self._lock:
                control = self._control_signal
                master_pose = copy.deepcopy(self._master_pose)
                master_quat = copy.deepcopy(self._master_quat)

            # 仅在 control_signal 为 3 或 10 时执行遥操作
            if control not in self._TELEOP_SIGNALS:
                time.sleep(period)
                continue

            if master_pose is not None and master_quat is not None:
                target_pose = self._map_master_to_slave_pose_quat_twist_xyz(master_pose, master_quat)
                with self._lock:
                    self._target_pose = list(target_pose)
                    control_to_publish = self._control_signal
                self._publish_command(control_to_publish, target_pose)

            time.sleep(period)

        self.get_logger().info('Publish thread stopped.')

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
            self._reset_initial_pose_from_feedback()
            self._ref_master_pose = None
            self._ref_master_quat = None
            self._control_signal = 10

        if self._initial_pose is None:
            self._initial_pose = list(self._initial_pose_default)

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

    def _publish_command(self, control_flag, target_pose):
        """发布控制模式与目标位姿到同一话题（std_msgs/Float64MultiArray）。

        data 格式: [control_flag, x, y, z, rx, ry, rz]
        """
        if target_pose is None or len(target_pose) != 6:
            return

        msg = Float64MultiArray()
        msg.data = [
            float(control_flag),
            float(target_pose[0]),
            float(target_pose[1]),
            float(target_pose[2]),
            float(target_pose[3]),
            float(target_pose[4]),
            float(target_pose[5]),
        ]
        self._target_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    master_sub = Subscriber()
    rclpy.spin(master_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
