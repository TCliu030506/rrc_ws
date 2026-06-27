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
    """判断当前接触扫查状态是否应该绕过导纳控制器。"""
    return state_value in direct_states


def blend_ratio(elapsed: float, duration: float) -> float:
    """根据经过时间计算 0..1 的插值比例，防止越界。"""
    if duration <= 0.0:
        return 1.0
    return min(1.0, max(0.0, elapsed / duration))


def pose_msg_to_state(msg: Pose) -> PoseState:
    """把 ROS Pose 消息转换成轨迹工具函数共用的 PoseState。"""
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
    """把 PoseState 转回 ROS Pose 消息。"""
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
    """用路径轨迹模块的插值逻辑在两个 Pose 消息之间平滑过渡。"""
    return pose_state_to_msg(
        interpolate_pose(
            pose_msg_to_state(start),
            pose_msg_to_state(target),
            ratio,
        )
    )


class ScanPoseMux(Node):
    """
    选择最终发给 servoL 的末端位姿命令。

    节点有两个位姿输入：
    - direct_pose_topic: 接触轨迹节点发布的名义期望位姿 `/scan/desired_pose`
    - admittance_pose_topic: 导纳控制器输出的柔顺修正后位姿

    APPROACH/PRE_CONTACT 使用名义位姿直接控制机械臂；其他阶段使用导纳
    控制器输出的位姿。这样导纳节点可以一直运行，mux 只负责选择哪一路
    Pose 真正送到 servo 控制器。
    """

    def __init__(self) -> None:
        super().__init__('scan_pose_mux')

        self.declare_parameter('direct_pose_topic', '/scan/desired_pose')
        self.declare_parameter('admittance_pose_topic', '/admittance/asm_ee_cmd_pose')
        self.declare_parameter('output_pose_topic', '/servo/asm_ee_cmd_pose')
        self.declare_parameter('state_topic', '/contact_scan/state')
        self.declare_parameter('direct_states', ['approach', 'pre_contact'])
        self.declare_parameter('admittance_blend_duration', 0.5)
        self.declare_parameter('approach_blend_duration', 0.5)

        direct_pose_topic = str(self.get_parameter('direct_pose_topic').value)
        admittance_pose_topic = str(self.get_parameter('admittance_pose_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        self.direct_states = set(str(v) for v in self.get_parameter('direct_states').value)
        self.admittance_blend_duration = float(
            self.get_parameter('admittance_blend_duration').value
        )
        self.approach_blend_duration = float(
            self.get_parameter('approach_blend_duration').value
        )
        # current_state 由 /contact_scan/state 更新。初始为空时默认不直通，
        # 即在没有状态信息前优先使用导纳输出，避免误把轨迹源直接送给 servo。
        self.current_state = ''

        # latest_direct_pose 保存轨迹源最近一帧；last_output_pose 保存 mux 真正
        # 发给 servo 的最近一帧，切换到导纳时用作平滑接管起点。
        self.latest_direct_pose: Pose | None = None
        self.last_output_pose: Pose | None = None

        # blend_* 只在“直通 -> 导纳”切换后短时间有效，用于避免输出位姿突跳。
        self.blend_start_pose: Pose | None = None
        self.blend_start_time: float | None = None
        self.direct_blend_start_pose: Pose | None = None
        self.direct_blend_start_time: float | None = None

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
            f'admittance_blend_duration={self.admittance_blend_duration:.3f}s, '
            f'approach_blend_duration={self.approach_blend_duration:.3f}s'
        )

    def _on_state(self, msg: String) -> None:
        """接收接触扫查状态，并根据状态边沿启动或清除导纳接管插值。"""
        was_direct = should_use_direct_pose(self.current_state, self.direct_states)
        will_use_direct = should_use_direct_pose(msg.data, self.direct_states)
        will_enter_approach = msg.data == 'approach' and self.current_state != 'approach'
        self.current_state = msg.data
        # 只有从直通状态离开时才需要平滑接管；进入直通状态时直接使用轨迹源。
        if was_direct and not will_use_direct:
            self._start_admittance_blend()
        elif will_use_direct:
            self._clear_admittance_blend()
        if will_enter_approach:
            self._start_direct_blend()
        elif msg.data != 'approach':
            self._clear_direct_blend()

    def _on_direct_pose(self, msg: Pose) -> None:
        """接收轨迹源名义位姿；仅在配置的直通状态下转发到 servo。"""
        self.latest_direct_pose = msg
        if should_use_direct_pose(self.current_state, self.direct_states):
            self._publish_pose(self._blend_direct_pose(msg))

    def _on_admittance_pose(self, msg: Pose) -> None:
        """接收导纳输出位姿；非直通状态下转发，必要时先做平滑插值。"""
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

    def _start_direct_blend(self) -> None:
        """进入 approach 时，从上一帧 servo 目标平滑接到第一帧轨迹源."""
        if self.approach_blend_duration <= 0.0 or self.last_output_pose is None:
            self._clear_direct_blend()
            return
        self.direct_blend_start_pose = self.last_output_pose
        self.direct_blend_start_time = time.monotonic()

    def _clear_direct_blend(self) -> None:
        """清除进入 approach 时的直通接管插值状态."""
        self.direct_blend_start_pose = None
        self.direct_blend_start_time = None

    def _blend_direct_pose(self, msg: Pose) -> Pose:
        """必要时把第一段直通轨迹从上一帧 servo 目标平滑插值过去."""
        if self.direct_blend_start_pose is None or self.direct_blend_start_time is None:
            return msg
        ratio = blend_ratio(
            time.monotonic() - self.direct_blend_start_time,
            self.approach_blend_duration,
        )
        if ratio >= 1.0:
            self._clear_direct_blend()
            return msg
        return interpolate_pose_msg(self.direct_blend_start_pose, msg, ratio)

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
        # 位置线性插值，姿态使用 path_map_trajectory_logic.interpolate_pose()
        # 内部的四元数 slerp，避免姿态分量直接线性混合。
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
