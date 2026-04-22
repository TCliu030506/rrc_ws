from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class AdmittancePoseToIkTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('admittance_pose_to_ik_trajectory_node')

        self.declare_parameter('input_pose_topic', '/admittance/cmd_pose')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('trajectory_action', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('ik_link_name', 'tool0')
        self.declare_parameter('planning_frame', 'base')
        self.declare_parameter('avoid_collisions', True)
        self.declare_parameter(
            'joint_names',
            [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint',
            ],
        )
        self.declare_parameter('ik_timeout_sec', 0.08)
        self.declare_parameter('execute_duration_sec', 0.12)
        self.declare_parameter('command_rate_hz', 20.0)

        self.input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.ik_service = str(self.get_parameter('ik_service').value)
        self.trajectory_action = str(self.get_parameter('trajectory_action').value)
        self.group_name = str(self.get_parameter('group_name').value)
        self.ik_link_name = str(self.get_parameter('ik_link_name').value)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.avoid_collisions = bool(self.get_parameter('avoid_collisions').value)
        self.joint_names = [str(x) for x in self.get_parameter('joint_names').value]
        self.ik_timeout_sec = float(self.get_parameter('ik_timeout_sec').value)
        self.execute_duration_sec = float(self.get_parameter('execute_duration_sec').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)

        if self.command_rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be > 0')

        self.latest_joint_map: Dict[str, float] = {}
        self.latest_pose: Optional[Pose] = None
        self.pose_seq = 0
        self.last_processed_pose_seq = 0
        self.ik_request_pending = False

        self.ik_client = self.create_client(GetPositionIK, self.ik_service)
        self.traj_client = ActionClient(self, FollowJointTrajectory, self.trajectory_action)

        self.create_subscription(Pose, self.input_pose_topic, self._pose_cb, 20)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 20)

        self.timer = self.create_timer(1.0 / self.command_rate_hz, self._tick)

        self.get_logger().info(
            'admittance->IK bridge started: '
            f'pose={self.input_pose_topic}, ik={self.ik_service}, action={self.trajectory_action}'
        )

    def _pose_cb(self, msg: Pose) -> None:
        self.latest_pose = msg
        self.pose_seq += 1

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_map[name] = float(pos)

    def _tick(self) -> None:
        if self.latest_pose is None:
            return
        if self.pose_seq == self.last_processed_pose_seq:
            return
        if self.ik_request_pending:
            return

        if not self.ik_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('IK service not ready: ' + self.ik_service, throttle_duration_sec=2.0)
            return

        if not self.traj_client.server_is_ready():
            self.get_logger().warn(
                'trajectory action not ready: ' + self.trajectory_action,
                throttle_duration_sec=2.0,
            )
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout = self._sec_to_duration(self.ik_timeout_sec)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = self.latest_pose
        req.ik_request.pose_stamped = pose_stamped

        if self.latest_joint_map:
            req.ik_request.robot_state.joint_state.name = list(self.latest_joint_map.keys())
            req.ik_request.robot_state.joint_state.position = [
                self.latest_joint_map[name] for name in self.latest_joint_map.keys()
            ]

        self.ik_request_pending = True
        req_pose_seq = self.pose_seq
        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f: self._on_ik_response(f, req_pose_seq))

    def _on_ik_response(self, future, req_pose_seq: int) -> None:
        self.ik_request_pending = False

        try:
            resp = future.result()
        except Exception as ex:
            self.get_logger().warn(f'IK call failed: {ex}')
            return

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f'IK failed, error_code={resp.error_code.val}',
                throttle_duration_sec=0.5,
            )
            return

        solution_map = {
            name: pos
            for name, pos in zip(
                resp.solution.joint_state.name,
                resp.solution.joint_state.position,
            )
        }

        target_positions: List[float] = []
        for name in self.joint_names:
            if name not in solution_map:
                self.get_logger().warn(f'IK solution missing joint: {name}')
                return
            target_positions.append(float(solution_map[name]))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = self._sec_to_duration(self.execute_duration_sec)
        goal.trajectory.points = [point]

        self.last_processed_pose_seq = req_pose_seq
        send_future = self.traj_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Failed to send trajectory goal: {ex}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal rejected.')

    @staticmethod
    def _sec_to_duration(sec: float) -> Duration:
        sec_i = int(sec)
        nsec_i = int((sec - sec_i) * 1e9)
        d = Duration()
        d.sec = sec_i
        d.nanosec = nsec_i
        return d


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittancePoseToIkTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
