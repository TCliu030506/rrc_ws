import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Accel, Pose, Twist, WrenchStamped
from rclpy.node import Node

from robot_trajectory_planner.path_map_trajectory_logic import (
    PathMapTrajectory,
    PoseState,
    parse_path_map_file,
    resample_waypoints,
)
from ultra_scanning_system.contact_scan_trajectory_logic import (
    ContactScanState,
    apply_contact_offset,
    compute_approach_axis,
    compute_retract_pose,
    normal_force,
    should_enter_contact,
    should_fault_by_force,
    should_fault_by_search_distance,
)


class ContactScanTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('contact_scan_trajectory_node')

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
        self.declare_parameter('control_wrench_frame', 'asm_ee_site')
        self.declare_parameter('path_file', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_linear_speed', 0.005)
        self.declare_parameter('max_angular_speed', 0.05)
        self.declare_parameter('max_path_linear_step', 0.001)
        self.declare_parameter('max_path_angular_step', 0.01)
        self.declare_parameter('approach_axis_sign', 1.0)
        self.declare_parameter('force_axis', 'z')
        self.declare_parameter('force_axis_sign', 1.0)
        self.declare_parameter('contact_force_threshold', 2.0)
        self.declare_parameter('target_contact_force', 6.0)
        self.declare_parameter('max_contact_force', 12.0)
        self.declare_parameter('max_search_distance', 0.04)
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

        self.current_pose: PoseState | None = None
        self.latest_wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.state = ContactScanState.APPROACH
        self.delta_c = 0.0
        self.search_distance = 0.0
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
            f'state={self.state.value}'
        )

    @staticmethod
    def _resolve_path_file(path_file_param: str) -> Path:
        if path_file_param:
            return Path(path_file_param).expanduser()
        package_share = Path(get_package_share_directory('robot_trajectory_planner'))
        return package_share / 'data' / 'path_map.txt'

    def _on_current_pose(self, msg: Pose) -> None:
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
        self.latest_wrench = (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )

    def _on_timer(self) -> None:
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
            self.state = ContactScanState.FAULT

        if self.state == ContactScanState.APPROACH:
            self._publish_zero_control_wrench()
            self._ensure_trajectory([self.path_points[0]], loop_path=False)
            command = self._advance_trajectory(dt)
            if command.finished:
                self.state = ContactScanState.PRE_CONTACT
                self.trajectory = None
            self._publish_command(command)
            return

        if self.state == ContactScanState.PRE_CONTACT:
            self._publish_zero_control_wrench()
            if should_enter_contact(
                fn,
                contact_force_threshold=self.contact_force_threshold,
            ):
                self.delta_c = self.search_distance
                contact_path = apply_contact_offset(
                    [
                        (point.position, point.orientation_xyzw)
                        for point in self.path_points
                    ],
                    delta_c=self.delta_c,
                    approach_axis_sign=self.approach_axis_sign,
                )
                self._ensure_trajectory([
                    PoseState(position=position, orientation_xyzw=orientation)
                    for position, orientation in contact_path
                ], loop_path=False)
                self.state = ContactScanState.CONTACT_SCAN
                self.get_logger().info(
                    f'Contact established: delta_c={self.delta_c:.4f} m'
                )
                return

            self.search_distance += self.pre_contact_speed * dt
            if should_fault_by_search_distance(
                self.search_distance,
                max_search_distance=self.max_search_distance,
            ):
                self.state = ContactScanState.FAULT
                self.get_logger().error('Contact search exceeded max_search_distance.')
                return

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

        if self.state == ContactScanState.CONTACT_SCAN:
            self._publish_target_control_wrench()
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
                self.state = ContactScanState.RETRACT
            return

        if self.state == ContactScanState.RETRACT:
            self._publish_zero_control_wrench()
            command = self._advance_trajectory(dt)
            self._publish_command(command)
            if command.finished:
                self.state = ContactScanState.FINISHED
            return

        if self.state == ContactScanState.FAULT:
            self._publish_zero_control_wrench()
            self._publish_pose_twist_accel(
                self.current_pose,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            return

        self._publish_zero_control_wrench()

    def _ensure_trajectory(self, waypoints: list[PoseState], *, loop_path: bool) -> None:
        if self.trajectory is not None:
            return
        assert self.current_pose is not None
        self.trajectory = PathMapTrajectory(
            waypoints=waypoints,
            start_pose=self.current_pose,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            min_segment_duration=0.0,
            loop_path=loop_path,
        )

    def _advance_trajectory(self, dt: float):
        assert self.trajectory is not None
        return self.trajectory.advance(dt)

    def _publish_command(self, command) -> None:
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
        pose_msg = Pose()
        pose_msg.position.x = pose.position[0]
        pose_msg.position.y = pose.position[1]
        pose_msg.position.z = pose.position[2]
        pose_msg.orientation.x = pose.orientation_xyzw[0]
        pose_msg.orientation.y = pose.orientation_xyzw[1]
        pose_msg.orientation.z = pose.orientation_xyzw[2]
        pose_msg.orientation.w = pose.orientation_xyzw[3]
        self.pose_pub.publish(pose_msg)

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
        self._publish_control_wrench(0.0)

    def _publish_target_control_wrench(self) -> None:
        signed_force = self.force_axis_sign * self.target_contact_force
        self._publish_control_wrench(signed_force)

    def _publish_control_wrench(self, signed_force: float) -> None:
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
