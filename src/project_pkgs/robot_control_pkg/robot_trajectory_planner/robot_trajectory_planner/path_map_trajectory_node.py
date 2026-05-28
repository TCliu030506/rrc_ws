import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Accel, Pose, Twist
from rclpy.node import Node

from robot_trajectory_planner.path_map_trajectory_logic import (
    PathMapTrajectory,
    PoseState,
    parse_path_map_file,
    resample_waypoints,
)


class PathMapTrajectoryNode(Node):
    """从 path_map.txt 文件发布扫查期望轨迹。

    本节点只负责 ROS 输入输出。轨迹数学逻辑放在
    path_map_trajectory_logic.py 中，这样不需要运行 ROS 图也能测试。
    """

    def __init__(self) -> None:
        super().__init__('path_map_trajectory_node')

        # 话题参数与 ultra_scanning_system.launch.py 中导纳控制器的接口保持一致。
        self.declare_parameter('current_pose_topic', '/asm_ee_site/pose')
        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')

        # path_file 为空表示使用本包安装目录下的 data/path_map.txt。
        self.declare_parameter('path_file', '')

        # max_*_speed 用于轨迹时间参数化；max_path_*_step 用于在时间参数化前
        # 对文件中的稀疏路径点做加密重采样。
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_linear_speed', 0.005)
        self.declare_parameter('max_angular_speed', 0.05)
        self.declare_parameter('min_segment_duration', 0.0)
        self.declare_parameter('loop_path', False)
        self.declare_parameter('enable_path_resampling', True)
        self.declare_parameter('max_path_linear_step', 0.002)
        self.declare_parameter('max_path_angular_step', 0.02)

        current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        desired_pose_topic = str(self.get_parameter('topic_desired_pose').value)
        desired_twist_topic = str(self.get_parameter('topic_desired_twist').value)
        desired_accel_topic = str(self.get_parameter('topic_desired_accel').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self._min_segment_duration = float(
            self.get_parameter('min_segment_duration').value
        )
        self._loop_path = bool(self.get_parameter('loop_path').value)
        self._enable_path_resampling = bool(
            self.get_parameter('enable_path_resampling').value
        )
        self._max_path_linear_step = float(
            self.get_parameter('max_path_linear_step').value
        )
        self._max_path_angular_step = float(
            self.get_parameter('max_path_angular_step').value
        )

        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')

        path_file_param = str(self.get_parameter('path_file').value)
        self._path_file = self._resolve_path_file(path_file_param)
        raw_waypoints = parse_path_map_file(self._path_file)

        # 重采样是几何预处理步骤。运行时仍会在重采样后的路径点之间，
        # 按 timer 周期继续做插值发布。
        if self._enable_path_resampling:
            self._waypoints = resample_waypoints(
                raw_waypoints,
                max_linear_step=self._max_path_linear_step,
                max_angular_step=self._max_path_angular_step,
            )
        else:
            self._waypoints = raw_waypoints

        self._current_pose: PoseState | None = None
        self._trajectory: PathMapTrajectory | None = None
        self._last_time: float | None = None

        self._pose_pub = self.create_publisher(Pose, desired_pose_topic, 10)
        self._twist_pub = self.create_publisher(Twist, desired_twist_topic, 10)
        self._accel_pub = self.create_publisher(Accel, desired_accel_topic, 10)
        self.create_subscription(Pose, current_pose_topic, self._on_current_pose, 10)
        self.create_timer(1.0 / self._publish_rate, self._on_timer)

        self.get_logger().info(
            'Path-map trajectory node started: '
            f'path_file={self._path_file}, '
            f'raw_waypoints={len(raw_waypoints)}, waypoints={len(self._waypoints)}, '
            f'current_pose_topic={current_pose_topic}, '
            f'topics=({desired_pose_topic}, {desired_twist_topic}, {desired_accel_topic})'
        )

    @staticmethod
    def _resolve_path_file(path_file_param: str) -> Path:
        """解析显式路径；若为空则使用包内安装的默认路径文件。"""
        if path_file_param:
            return Path(path_file_param).expanduser()

        package_share = Path(get_package_share_directory('robot_trajectory_planner'))
        return package_share / 'data' / 'path_map.txt'

    def _on_current_pose(self, msg: Pose) -> None:
        """保存最新的受控末端实时位姿。

        轨迹会从这个位姿开始初始化，因此第一段运动是从真实当前位置
        平滑移动到路径文件的第一个目标点。
        """
        self._current_pose = PoseState(
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

    def _on_timer(self) -> None:
        if self._trajectory is None:
            if self._current_pose is None:
                self.get_logger().info(
                    'Waiting for current EE pose...',
                    throttle_duration_sec=2.0,
                )
                return

            # 等收到第一帧当前位姿后再创建轨迹。否则第一条指令可能从
            # 任意默认位姿跳到路径起点。
            self._trajectory = PathMapTrajectory(
                waypoints=self._waypoints,
                start_pose=self._current_pose,
                max_linear_speed=self._max_linear_speed,
                max_angular_speed=self._max_angular_speed,
                min_segment_duration=self._min_segment_duration,
                loop_path=self._loop_path,
            )
            self._last_time = time.monotonic()
            self.get_logger().info('Current EE pose received; path-map trajectory started.')

        now = time.monotonic()
        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time
        self._last_time = now

        command = self._trajectory.advance(dt)
        self._pose_pub.publish(_pose_state_to_msg(command.pose))
        self._twist_pub.publish(_twist_to_msg(command.twist_linear, command.twist_angular))
        self._accel_pub.publish(_accel_to_msg(command.accel_linear, command.accel_angular))


def _pose_state_to_msg(pose_state: PoseState) -> Pose:
    """将纯逻辑层的 PoseState 转换为 geometry_msgs/Pose。"""
    msg = Pose()
    msg.position.x = pose_state.position[0]
    msg.position.y = pose_state.position[1]
    msg.position.z = pose_state.position[2]
    msg.orientation.x = pose_state.orientation_xyzw[0]
    msg.orientation.y = pose_state.orientation_xyzw[1]
    msg.orientation.z = pose_state.orientation_xyzw[2]
    msg.orientation.w = pose_state.orientation_xyzw[3]
    return msg


def _twist_to_msg(linear: tuple[float, float, float], angular: tuple[float, float, float]) -> Twist:
    """将纯逻辑层的速度向量转换为 geometry_msgs/Twist。"""
    msg = Twist()
    msg.linear.x = linear[0]
    msg.linear.y = linear[1]
    msg.linear.z = linear[2]
    msg.angular.x = angular[0]
    msg.angular.y = angular[1]
    msg.angular.z = angular[2]
    return msg


def _accel_to_msg(linear: tuple[float, float, float], angular: tuple[float, float, float]) -> Accel:
    """将纯逻辑层的加速度向量转换为 geometry_msgs/Accel。"""
    msg = Accel()
    msg.linear.x = linear[0]
    msg.linear.y = linear[1]
    msg.linear.z = linear[2]
    msg.angular.x = angular[0]
    msg.angular.y = angular[1]
    msg.angular.z = angular[2]
    return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathMapTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
