import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Accel, Pose, Twist, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_trajectory_planner.path_map_trajectory_logic import (
    PathMapTrajectory,
    PoseState,
    parse_path_map_file,
    resample_waypoints,
)
from ultra_scanning_system.contact_scan_trajectory_logic import (
    ContactScanState,
    align_contact_path_start,
    apply_contact_offset,
    compute_approach_axis,
    compute_retract_pose,
    normal_force,
    ramp_toward,
    should_enter_contact,
    should_fault_by_force,
    should_fault_by_search_distance,
)


class ContactScanTrajectoryNode(Node):
    """
    恒定接触力曲面扫查的上层轨迹状态机节点.

    本节点只生成名义轨迹和目标 wrench，不直接做导纳积分。
    实际柔顺补偿仍由 `robot_admittance_control` 在工具坐标系下完成。
    """

    def __init__(self) -> None:
        super().__init__('contact_scan_trajectory_node')

        # 输入/输出话题。desired_* 继续复用现有导纳控制器接口。
        self.declare_parameter('current_pose_topic', '/asm_ee_site/pose')
        self.declare_parameter(
            'compensated_wrench_topic',
            '/external_force_torque_wrench_compensated',
        )
        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')
        self.declare_parameter(
            'topic_control_wrench',
            '/arm_admittance_control/control_wrench',
        )
        self.declare_parameter('state_topic', '/contact_scan/state')
        self.declare_parameter('control_wrench_frame', 'asm_ee_site')

        # 路径和速度参数。path_file 为空时读取 robot_trajectory_planner 默认路径。
        self.declare_parameter('path_file', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_linear_speed', 0.005)
        self.declare_parameter('max_angular_speed', 0.05)
        self.declare_parameter('max_path_linear_step', 0.001)
        self.declare_parameter('max_path_angular_step', 0.01)

        # 接触搜索和恒力控制参数。符号以当前项目实测为准。
        self.declare_parameter('approach_axis_sign', 1.0)
        self.declare_parameter('force_axis', 'z')
        self.declare_parameter('force_axis_sign', 1.0)
        self.declare_parameter('contact_force_threshold', 2.0)
        self.declare_parameter('target_contact_force', 10.0)
        self.declare_parameter('force_ramp_rate', 2.0)
        self.declare_parameter('zero_torque_rx_ry_enabled', True)
        self.declare_parameter('target_torque_rx', 0.0)
        self.declare_parameter('target_torque_ry', 0.0)
        self.declare_parameter('torque_ramp_rate', 0.05)
        self.declare_parameter('contact_settle_duration', 0.5)
        self.declare_parameter('contact_settle_force_tolerance', 2.0)
        self.declare_parameter('max_contact_force', 15.0)
        self.declare_parameter('max_search_distance', 0.10)
        self.declare_parameter('pre_contact_speed', 0.002)
        self.declare_parameter('retract_distance', 0.04)

        self.current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        self.compensated_wrench_topic = str(
            self.get_parameter('compensated_wrench_topic').value
        )
        self.desired_pose_topic = str(self.get_parameter('topic_desired_pose').value)
        self.desired_twist_topic = str(self.get_parameter('topic_desired_twist').value)
        self.desired_accel_topic = str(self.get_parameter('topic_desired_accel').value)
        self.control_wrench_topic = str(self.get_parameter('topic_control_wrench').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.control_wrench_frame = str(self.get_parameter('control_wrench_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_path_linear_step = float(
            self.get_parameter('max_path_linear_step').value
        )
        self.max_path_angular_step = float(
            self.get_parameter('max_path_angular_step').value
        )
        self.approach_axis_sign = float(
            self.get_parameter('approach_axis_sign').value
        )
        self.force_axis = str(self.get_parameter('force_axis').value)
        self.force_axis_sign = float(self.get_parameter('force_axis_sign').value)
        self.contact_force_threshold = float(
            self.get_parameter('contact_force_threshold').value
        )
        self.target_contact_force = float(
            self.get_parameter('target_contact_force').value
        )
        self.force_ramp_rate = float(self.get_parameter('force_ramp_rate').value)
        self.zero_torque_rx_ry_enabled = bool(
            self.get_parameter('zero_torque_rx_ry_enabled').value
        )
        self.target_torque_rx = float(self.get_parameter('target_torque_rx').value)
        self.target_torque_ry = float(self.get_parameter('target_torque_ry').value)
        self.torque_ramp_rate = float(self.get_parameter('torque_ramp_rate').value)
        self.contact_settle_duration = float(
            self.get_parameter('contact_settle_duration').value
        )
        self.contact_settle_force_tolerance = float(
            self.get_parameter('contact_settle_force_tolerance').value
        )
        self.max_contact_force = float(self.get_parameter('max_contact_force').value)
        self.max_search_distance = float(
            self.get_parameter('max_search_distance').value
        )
        self.pre_contact_speed = float(self.get_parameter('pre_contact_speed').value)
        self.retract_distance = float(self.get_parameter('retract_distance').value)

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if self.pre_contact_speed < 0.0:
            raise ValueError('pre_contact_speed must be >= 0')

        path_file = self._resolve_path_file(str(self.get_parameter('path_file').value))
        raw_waypoints = parse_path_map_file(path_file)
        self.path_points = resample_waypoints(
            raw_waypoints,
            max_linear_step=self.max_path_linear_step,
            max_angular_step=self.max_path_angular_step,
        )

        # 当前状态缓存：current_pose 来自 TF 状态节点，latest_wrench 来自补偿后力传感器。
        self.current_pose: PoseState | None = None
        self.latest_wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.latest_wrench_frame = self.control_wrench_frame
        self.state = ContactScanState.APPROACH

        # delta_c 是预接触阶段记录的法向搜索距离，用于把 P0 修正为 P2。
        self.delta_c = 0.0
        self.search_distance = 0.0
        self.contact_force_ref = 0.0
        self.control_wrench_ref = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.control_wrench_ramp_frame = self.control_wrench_frame
        self.contact_settle_pose: PoseState | None = None
        self.last_pre_contact_command_pose: PoseState | None = None
        self.last_published_desired_pose: PoseState | None = None
        self.contact_settle_stable_time = 0.0
        self.contact_path_points: list[PoseState] = []
        self.fault_hold_pose: PoseState | None = None

        # PathMapTrajectory 只负责两点间时间参数化和插值；阶段切换由本节点控制。
        self.trajectory: PathMapTrajectory | None = None
        self.last_time: float | None = None

        self.pose_pub = self.create_publisher(Pose, self.desired_pose_topic, 10)
        self.twist_pub = self.create_publisher(Twist, self.desired_twist_topic, 10)
        self.accel_pub = self.create_publisher(Accel, self.desired_accel_topic, 10)
        self.control_wrench_pub = self.create_publisher(
            WrenchStamped,
            self.control_wrench_topic,
            10,
        )
        # 状态话题只在初始状态和状态切换时发布；TRANSIENT_LOCAL 让后启动的
        # `ros2 topic echo /contact_scan/state` 也能收到最近一次状态。
        state_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, state_qos)

        self.create_subscription(Pose, self.current_pose_topic, self._on_current_pose, 10)
        self.create_subscription(
            WrenchStamped,
            self.compensated_wrench_topic,
            self._on_wrench,
            10,
        )
        self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'Contact scan trajectory node started: '
            f'path_file={path_file}, waypoints={len(self.path_points)}, '
            f'state={self.state.value}, state_topic={self.state_topic}'
        )
        self._publish_state()

    @staticmethod
    def _resolve_path_file(path_file_param: str) -> Path:
        """解析 path-map 文件路径."""
        if path_file_param:
            return Path(path_file_param).expanduser()
        package_share = Path(get_package_share_directory('robot_trajectory_planner'))
        return package_share / 'data' / 'path_map.txt'

    def _on_current_pose(self, msg: Pose) -> None:
        """保存最新受控末端 `asm_ee_site` 位姿."""
        self.current_pose = PoseState(
            position=(
                float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
            ),
            orientation_xyzw=(
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ),
        )

    def _on_wrench(self, msg: WrenchStamped) -> None:
        """
        保存补偿后的六维力数据.

        这里直接读取消息值。坐标系符号由上游补偿和本节点参数共同保证。
        """
        self.latest_wrench_frame = msg.header.frame_id or self.control_wrench_frame
        self.latest_wrench = (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )

    def _on_timer(self) -> None:
        """
        主状态机循环.

        每个周期只处于一个阶段：接近、预接触、接触扫查、撤离或故障。
        这样避免“未接触时导纳积分”和“接触后恒力调节”混在一起。
        """
        now = time.monotonic()
        dt = 0.0 if self.last_time is None else max(0.0, now - self.last_time)
        self.last_time = now

        if self.current_pose is None:
            self.get_logger().info(
                'Waiting for current EE pose...',
                throttle_duration_sec=2.0,
            )
            return

        fn = normal_force(
            self.latest_wrench,
            axis=self.force_axis,
            force_axis_sign=self.force_axis_sign,
        )
        if should_fault_by_force(fn, max_contact_force=self.max_contact_force):
            self._set_state(
                ContactScanState.FAULT,
                reason=f'normal_force={fn:.3f} exceeds max_contact_force',
            )

        if self.state == ContactScanState.APPROACH:
            # 接近阶段还不做恒力导纳。发布当前实测 wrench 作为控制
            # wrench，抵消导纳方程中的外力项，使执行链近似退化为位置跟踪。
            self._publish_measured_control_wrench()
            self._ensure_trajectory([self.path_points[0]], loop_path=False)
            command = self._advance_trajectory(dt)
            if command.finished:
                self._set_state(
                    ContactScanState.PRE_CONTACT,
                    reason='approach trajectory finished',
                )
                self.trajectory = None
            self._publish_command(command)
            return

        if self.state == ContactScanState.PRE_CONTACT:
            # 预接触阶段仍按位置搜索，不让导纳把接触力主动调回 0。
            # 这里用实测 wrench 抵消导纳外力项，直到真正进入 CONTACT_SETTLE。
            self._publish_measured_control_wrench()
            if should_enter_contact(
                fn,
                contact_force_threshold=self.contact_force_threshold,
            ):
                approach = compute_approach_axis(
                    self.path_points[0].orientation_xyzw,
                    approach_axis_sign=self.approach_axis_sign,
                )
                start = self.path_points[0].position
                contact = self.current_pose.position
                # delta_c 使用实际接触位姿相对粗路径起点的接近轴投影，
                # 避免用理论 search_distance 估计真实曲面偏置。
                self.delta_c = (
                    (contact[0] - start[0]) * approach[0]
                    + (contact[1] - start[1]) * approach[1]
                    + (contact[2] - start[2]) * approach[2]
                )
                # CONTACT_SETTLE 从当前实测接触力开始，再按斜坡逼近目标力。
                # 这样避免从 0N 目标力开始导致导纳主动卸力。
                self.contact_force_ref = min(
                    max(fn, 0.0),
                    self.target_contact_force,
                )
                # CONTACT_SETTLE 的 control_wrench 从 PRE_CONTACT 最后一帧
                # 完整实测 wrench 开始，避免力/力矩命令结构突变。
                self.control_wrench_ref = self.latest_wrench
                self.control_wrench_ramp_frame = self.latest_wrench_frame
                # 切换到 CONTACT_SETTLE 时锁住上一帧已经发布出去的
                # /scan/desired_pose，避免因状态切换改用其他缓存造成突变。
                self.contact_settle_pose = (
                    self.last_published_desired_pose
                    or self.last_pre_contact_command_pose
                    or self.current_pose
                )
                self.contact_settle_stable_time = 0.0
                contact_path = apply_contact_offset(
                    [
                        (point.position, point.orientation_xyzw)
                        for point in self.path_points
                    ],
                    delta_c=self.delta_c,
                    approach_axis_sign=self.approach_axis_sign,
                )
                self.contact_path_points = [
                    PoseState(position=position, orientation_xyzw=orientation)
                    for position, orientation in contact_path
                ]
                self._set_state(
                    ContactScanState.CONTACT_SETTLE,
                    reason=f'contact established, delta_c={self.delta_c:.4f} m',
                )
                self.get_logger().info(
                    f'Contact established: delta_c={self.delta_c:.4f} m'
                )
                return

            self.search_distance += self.pre_contact_speed * dt
            if should_fault_by_search_distance(
                self.search_distance,
                max_search_distance=self.max_search_distance,
            ):
                self._set_state(
                    ContactScanState.FAULT,
                    reason='contact search exceeded max_search_distance',
                )
                self.get_logger().error('Contact search exceeded max_search_distance.')
                return

            # 预接触运动不使用 PathMapTrajectory，直接沿起点工具法向推进。
            approach = compute_approach_axis(
                self.path_points[0].orientation_xyzw,
                approach_axis_sign=self.approach_axis_sign,
            )
            start = self.path_points[0].position
            pose = PoseState(
                position=(
                    start[0] + self.search_distance * approach[0],
                    start[1] + self.search_distance * approach[1],
                    start[2] + self.search_distance * approach[2],
                ),
                orientation_xyzw=self.path_points[0].orientation_xyzw,
            )
            # 记录直接位置搜索的最后一帧命令，CONTACT_SETTLE 用它保持
            # desired_pose 连续，而不是锁存有噪声/滞后的瞬时实际位姿。
            self.last_pre_contact_command_pose = pose
            self._publish_pose_twist_accel(
                pose,
                (
                    self.pre_contact_speed * approach[0],
                    self.pre_contact_speed * approach[1],
                    self.pre_contact_speed * approach[2],
                ),
                (0.0, 0.0, 0.0),
            )
            return

        if self.state == ContactScanState.CONTACT_SETTLE:
            # 稳定阶段先锁住接触瞬间位姿，不推进扫查路径。目标力从 0
            # 按斜坡爬升，并等待实测接触力在目标附近稳定一段时间。
            self._publish_target_control_wrench(dt)
            hold_pose = self.contact_settle_pose or self.current_pose
            self._publish_pose_twist_accel(
                hold_pose,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )

            target_reached = self.contact_force_ref >= self.target_contact_force
            force_stable = (
                self.contact_settle_force_tolerance <= 0.0
                or abs(fn - self.target_contact_force)
                <= self.contact_settle_force_tolerance
            )
            if target_reached and force_stable:
                self.contact_settle_stable_time += dt
            else:
                self.contact_settle_stable_time = 0.0

            if self.contact_settle_stable_time >= self.contact_settle_duration:
                scan_start_pose = (
                    self.last_published_desired_pose
                    or self.contact_settle_pose
                    or self.current_pose
                )
                aligned_path = align_contact_path_start(
                    [
                        (point.position, point.orientation_xyzw)
                        for point in self.contact_path_points
                    ],
                    stable_pose=(
                        scan_start_pose.position,
                        scan_start_pose.orientation_xyzw,
                    ),
                )
                self.contact_path_points = [
                    PoseState(position=position, orientation_xyzw=orientation)
                    for position, orientation in aligned_path
                ]
                self._ensure_trajectory(
                    self.contact_path_points,
                    loop_path=False,
                    start_pose=scan_start_pose,
                )
                self._set_state(
                    ContactScanState.CONTACT_SCAN,
                    reason='contact force settled',
                )
            return

        if self.state == ContactScanState.CONTACT_SCAN:
            # 接触扫查阶段才发布目标接触力，导纳控制器据此生成柔顺偏移。
            self._publish_target_control_wrench(dt)
            command = self._advance_trajectory(dt)
            self._publish_command(command)
            if command.finished:
                final_pose = command.pose
                retract = compute_retract_pose(
                    (final_pose.position, final_pose.orientation_xyzw),
                    retract_distance=self.retract_distance,
                    approach_axis_sign=self.approach_axis_sign,
                )
                self._ensure_trajectory([
                    PoseState(position=retract[0], orientation_xyzw=retract[1])
                ], loop_path=False)
                self._set_state(
                    ContactScanState.RETRACT,
                    reason='contact scan trajectory finished',
                )
            return

        if self.state == ContactScanState.RETRACT:
            # 撤离阶段卸载目标接触力，并沿最终接近方向反向退出。
            self._publish_zero_control_wrench()
            command = self._advance_trajectory(dt)
            self._publish_command(command)
            if command.finished:
                self._set_state(
                    ContactScanState.FINISHED,
                    reason='retract trajectory finished',
                )
            return

        if self.state == ContactScanState.FAULT:
            # 故障阶段保持当前位置并清零目标 wrench，等待人工处理。
            self._publish_zero_control_wrench()
            hold_pose = self.fault_hold_pose or self.current_pose
            self._publish_pose_twist_accel(
                hold_pose,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            return

        self._publish_zero_control_wrench()

    def _ensure_trajectory(
        self,
        waypoints: list[PoseState],
        *,
        loop_path: bool,
        start_pose: PoseState | None = None,
    ) -> None:
        """按当前真实位姿创建一段新的时间参数化轨迹."""
        if self.trajectory is not None:
            return
        assert self.current_pose is not None
        self.trajectory = PathMapTrajectory(
            waypoints=waypoints,
            start_pose=start_pose or self.current_pose,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            min_segment_duration=0.0,
            loop_path=loop_path,
        )

    def _advance_trajectory(self, dt: float):
        """推进当前 PathMapTrajectory."""
        assert self.trajectory is not None
        return self.trajectory.advance(dt)

    def _set_state(self, new_state: ContactScanState, *, reason: str) -> None:
        """切换状态并打印调试日志."""
        if new_state == self.state:
            return
        old_state = self.state
        self.state = new_state
        if new_state == ContactScanState.FAULT and self.current_pose is not None:
            # FAULT 只锁存第一次进入故障时的位姿。后续循环继续发布这个固定
            # desired_pose，避免“当前位姿 + 导纳偏移”形成持续逃逸运动。
            self.fault_hold_pose = self.current_pose
        self.get_logger().info(
            f'ContactScanState: {old_state.value} -> {new_state.value}; {reason}'
        )
        self._publish_state()

    def _publish_state(self) -> None:
        """发布当前 ContactScanState，便于 ros2 topic echo 调试."""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _publish_command(self, command) -> None:
        """发布 PathMapTrajectory 生成的 pose/twist/accel."""
        self._publish_pose_twist_accel(
            command.pose,
            command.twist_linear,
            command.twist_angular,
        )

    def _publish_pose_twist_accel(
        self,
        pose: PoseState,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float],
    ) -> None:
        """发布导纳控制器需要的名义位姿、速度和零加速度."""
        pose_msg = Pose()
        pose_msg.position.x = pose.position[0]
        pose_msg.position.y = pose.position[1]
        pose_msg.position.z = pose.position[2]
        pose_msg.orientation.x = pose.orientation_xyzw[0]
        pose_msg.orientation.y = pose.orientation_xyzw[1]
        pose_msg.orientation.z = pose.orientation_xyzw[2]
        pose_msg.orientation.w = pose.orientation_xyzw[3]
        self.pose_pub.publish(pose_msg)
        # 记录真正发布到 /scan/desired_pose 的最后一帧，状态切换时用它
        # 做保持位姿，避免理论命令缓存和实际发布值不一致。
        self.last_published_desired_pose = pose

        twist_msg = Twist()
        twist_msg.linear.x = linear[0]
        twist_msg.linear.y = linear[1]
        twist_msg.linear.z = linear[2]
        twist_msg.angular.x = angular[0]
        twist_msg.angular.y = angular[1]
        twist_msg.angular.z = angular[2]
        self.twist_pub.publish(twist_msg)

        self.accel_pub.publish(Accel())

    def _publish_zero_control_wrench(self) -> None:
        """发布零目标 wrench，用于撤离和故障阶段."""
        self.contact_force_ref = 0.0
        self._publish_control_wrench(0.0)

    def _publish_measured_control_wrench(self) -> None:
        """
        发布当前实测 wrench 作为控制 wrench，用于临时抵消导纳外力项.

        在 APPROACH/PRE_CONTACT 阶段，目标是几何位置搜索而不是恒力控制。
        让 wrench_control 近似等于 wrench_external，可避免导纳把接触力调回 0。
        """
        self.contact_force_ref = 0.0
        self.control_wrench_ref = self.latest_wrench
        self.control_wrench_ramp_frame = self.latest_wrench_frame
        self._publish_control_wrench_vector(
            self.latest_wrench,
            frame_id=self.latest_wrench_frame,
        )

    def _publish_target_control_wrench(self, dt: float) -> None:
        """按斜坡发布恒力扫查阶段的目标 wrench."""
        self.contact_force_ref = ramp_toward(
            self.contact_force_ref,
            self.target_contact_force,
            max_rate=self.force_ramp_rate,
            dt=dt,
        )
        signed_force = self.force_axis_sign * self.contact_force_ref
        target_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if self.force_axis == 'x':
            target_wrench[0] = signed_force
        elif self.force_axis == 'y':
            target_wrench[1] = signed_force
        else:
            target_wrench[2] = signed_force
        if self.zero_torque_rx_ry_enabled:
            target_wrench[3] = self.target_torque_rx
            target_wrench[4] = self.target_torque_ry

        # 从 PRE_CONTACT 的完整 wrench 平滑过渡到目标恒力 wrench。
        # 这样切换到 CONTACT_SETTLE 时，非目标轴的力/力矩不会瞬间清零。
        # 力和力矩单位不同，因此分别使用 N/s 和 Nm/s 的斜坡速度。
        ramp_rates = (
            self.force_ramp_rate,
            self.force_ramp_rate,
            self.force_ramp_rate,
            self.torque_ramp_rate,
            self.torque_ramp_rate,
            self.torque_ramp_rate,
        )
        self.control_wrench_ref = tuple(
            ramp_toward(
                current,
                target,
                max_rate=rate,
                dt=dt,
            )
            for current, target, rate in zip(
                self.control_wrench_ref,
                target_wrench,
                ramp_rates,
            )
        )
        self._publish_control_wrench_vector(
            self.control_wrench_ref,
            frame_id=self.control_wrench_ramp_frame,
        )

    def _publish_control_wrench(self, signed_force: float) -> None:
        """
        按配置轴向发布目标 wrench.

        导纳控制器会再把该 wrench 转换到 `admittance_frame` 中使用。
        """
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.control_wrench_frame
        if self.force_axis == 'x':
            msg.wrench.force.x = signed_force
        elif self.force_axis == 'y':
            msg.wrench.force.y = signed_force
        else:
            msg.wrench.force.z = signed_force
        self.control_wrench_pub.publish(msg)

    def _publish_control_wrench_vector(
        self,
        wrench: tuple[float, float, float, float, float, float],
        *,
        frame_id: str,
    ) -> None:
        """发布完整六维 wrench，保留原始坐标系供导纳控制器变换."""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.wrench.force.x = wrench[0]
        msg.wrench.force.y = wrench[1]
        msg.wrench.force.z = wrench[2]
        msg.wrench.torque.x = wrench[3]
        msg.wrench.torque.y = wrench[4]
        msg.wrench.torque.z = wrench[5]
        self.control_wrench_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContactScanTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
