import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def should_use_direct_pose(state_value: str, direct_states: set[str]) -> bool:
    """Return true when the current scan state should bypass admittance."""
    return state_value in direct_states


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

        direct_pose_topic = str(self.get_parameter('direct_pose_topic').value)
        admittance_pose_topic = str(self.get_parameter('admittance_pose_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        self.direct_states = set(str(v) for v in self.get_parameter('direct_states').value)
        self.current_state = ''

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
            f'direct_states={sorted(self.direct_states)}'
        )

    def _on_state(self, msg: String) -> None:
        """Store latest contact scan state."""
        self.current_state = msg.data

    def _on_direct_pose(self, msg: Pose) -> None:
        """Publish direct pose only during configured bypass states."""
        if should_use_direct_pose(self.current_state, self.direct_states):
            self.pose_pub.publish(msg)

    def _on_admittance_pose(self, msg: Pose) -> None:
        """Publish admittance pose when not bypassing admittance."""
        if not should_use_direct_pose(self.current_state, self.direct_states):
            self.pose_pub.publish(msg)


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
