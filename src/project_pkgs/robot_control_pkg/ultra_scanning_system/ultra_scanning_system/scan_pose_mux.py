import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_trajectory_planner.path_map_trajectory_logic import (
    PoseState,
    interpolate_pose,
)


def should_use_direct_pose(state_value: str, direct_states: set[str]) -> bool:
    """Return true when the current scan state should bypass admittance."""
    return state_value in direct_states


def blend_ratio(elapsed: float, duration: float) -> float:
    """Return a clamped 0..1 blend ratio."""
    if duration <= 0.0:
        return 1.0
    return min(1.0, max(0.0, elapsed / duration))


def pose_msg_to_state(msg: Pose) -> PoseState:
    """Convert geometry_msgs/Pose to the shared PoseState helper."""
    return PoseState(
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


def pose_state_to_msg(state: PoseState) -> Pose:
    """Convert PoseState back to geometry_msgs/Pose."""
    msg = Pose()
    msg.position.x = state.position[0]
    msg.position.y = state.position[1]
    msg.position.z = state.position[2]
    msg.orientation.x = state.orientation_xyzw[0]
    msg.orientation.y = state.orientation_xyzw[1]
    msg.orientation.z = state.orientation_xyzw[2]
    msg.orientation.w = state.orientation_xyzw[3]
    return msg


def interpolate_pose_msg(start: Pose, target: Pose, ratio: float) -> Pose:
    """Blend two Pose messages using the path-map pose interpolation logic."""
    return pose_state_to_msg(
        interpolate_pose(
            pose_msg_to_state(start),
            pose_msg_to_state(target),
            ratio,
        )
    )


class ScanPoseMux(Node):
    """
    Select the pose command sent to servoL.

    APPROACH/PRE_CONTACT 使用轨迹源的名义位姿直接控制机械臂；其他阶段
    使用导纳控制器输出的位姿。这样不需要运行时启动/停止导纳节点。
    """

    def __init__(self) -> None:
        super().__init__('scan_pose_mux')

        self.declare_parameter('direct_pose_topic', '/scan/desired_pose')
        self.declare_parameter('admittance_pose_topic', '/admittance/asm_ee_cmd_pose')
        self.declare_parameter('output_pose_topic', '/servo/asm_ee_cmd_pose')
        self.declare_parameter('state_topic', '/contact_scan/state')
        self.declare_parameter('direct_states', ['approach', 'pre_contact'])
        self.declare_parameter('admittance_blend_duration', 0.5)

        direct_pose_topic = str(self.get_parameter('direct_pose_topic').value)
        admittance_pose_topic = str(self.get_parameter('admittance_pose_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        self.direct_states = set(str(v) for v in self.get_parameter('direct_states').value)
        self.admittance_blend_duration = float(
            self.get_parameter('admittance_blend_duration').value
        )
        self.current_state = ''
        self.latest_direct_pose: Pose | None = None
        self.last_output_pose: Pose | None = None
        self.blend_start_pose: Pose | None = None
        self.blend_start_time: float | None = None

        self.pose_pub = self.create_publisher(Pose, output_pose_topic, 10)
        self.create_subscription(Pose, direct_pose_topic, self._on_direct_pose, 10)
        self.create_subscription(Pose, admittance_pose_topic, self._on_admittance_pose, 10)

        # contact_scan/state 是 transient local；这里也用 transient local，确保 mux
        # 后启动时能拿到最近一次状态。
        state_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, state_topic, self._on_state, state_qos)

        self.get_logger().info(
            'Scan pose mux started: '
            f'direct={direct_pose_topic}, admittance={admittance_pose_topic}, '
            f'output={output_pose_topic}, state={state_topic}, '
            f'direct_states={sorted(self.direct_states)}, '
            f'admittance_blend_duration={self.admittance_blend_duration:.3f}s'
        )

    def _on_state(self, msg: String) -> None:
        """Store latest contact scan state."""
        was_direct = should_use_direct_pose(self.current_state, self.direct_states)
        will_use_direct = should_use_direct_pose(msg.data, self.direct_states)
        self.current_state = msg.data
        if was_direct and not will_use_direct:
            self._start_admittance_blend()
        elif will_use_direct:
            self._clear_admittance_blend()

    def _on_direct_pose(self, msg: Pose) -> None:
        """Publish direct pose only during configured bypass states."""
        self.latest_direct_pose = msg
        if should_use_direct_pose(self.current_state, self.direct_states):
            self._publish_pose(msg)

    def _on_admittance_pose(self, msg: Pose) -> None:
        """Publish admittance pose when not bypassing admittance."""
        if not should_use_direct_pose(self.current_state, self.direct_states):
            self._publish_pose(self._blend_admittance_pose(msg))

    def _start_admittance_blend(self) -> None:
        """从直控切到导纳时，以上一帧输出为起点做短时间平滑接管."""
        if self.admittance_blend_duration <= 0.0:
            self._clear_admittance_blend()
            return
        self.blend_start_pose = self.last_output_pose or self.latest_direct_pose
        self.blend_start_time = time.monotonic() if self.blend_start_pose else None

    def _clear_admittance_blend(self) -> None:
        """清除导纳接管插值状态."""
        self.blend_start_pose = None
        self.blend_start_time = None

    def _blend_admittance_pose(self, msg: Pose) -> Pose:
        """必要时把导纳输出从上一帧 servo 目标平滑插值过去."""
        if self.blend_start_pose is None or self.blend_start_time is None:
            return msg
        ratio = blend_ratio(
            time.monotonic() - self.blend_start_time,
            self.admittance_blend_duration,
        )
        if ratio >= 1.0:
            self._clear_admittance_blend()
            return msg
        return interpolate_pose_msg(self.blend_start_pose, msg, ratio)

    def _publish_pose(self, msg: Pose) -> None:
        """发布 servo 目标，并缓存最后一帧输出供下一次接管使用."""
        self.pose_pub.publish(msg)
        self.last_output_pose = msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanPoseMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
