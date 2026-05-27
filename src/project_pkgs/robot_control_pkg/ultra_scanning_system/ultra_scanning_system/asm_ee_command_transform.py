import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from ultra_scanning_system.asm_ee_command_math import (
    Transform,
    compute_base_to_tool_command,
    normalize_quat,
)


def pose_msg_to_transform(msg: Pose) -> Transform:
    return (
        (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        ),
        normalize_quat((
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )),
    )


def transform_to_pose_msg(transform: Transform) -> Pose:
    xyz, quat = transform
    msg = Pose()
    msg.position.x = xyz[0]
    msg.position.y = xyz[1]
    msg.position.z = xyz[2]
    msg.orientation.x = quat[0]
    msg.orientation.y = quat[1]
    msg.orientation.z = quat[2]
    msg.orientation.w = quat[3]
    return msg


def tf_msg_to_transform(tf_msg) -> Transform:
    translation = tf_msg.transform.translation
    rotation = tf_msg.transform.rotation
    return (
        (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        ),
        normalize_quat((
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )),
    )


class AsmEeCommandTransformNode(Node):
    def __init__(self) -> None:
        super().__init__('asm_ee_command_transform_node')

        self.declare_parameter(
            'input_pose_topic',
            '/admittance/asm_ee_cmd_pose',
        )
        self.declare_parameter(
            'output_pose_topic',
            '/arm_desired_pose_tool0',
        )
        self.declare_parameter('control_base_frame', 'base')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('controlled_frame', 'asm_ee_site')
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)

        input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self.control_base_frame = str(
            self.get_parameter('control_base_frame').value
        )
        self.tool_frame = str(self.get_parameter('tool_frame').value)
        self.controlled_frame = str(
            self.get_parameter('controlled_frame').value
        )
        self.tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Pose, output_pose_topic, 10)
        self.sub = self.create_subscription(
            Pose,
            input_pose_topic,
            self._on_pose,
            10,
        )

        self.get_logger().info(
            'ASM EE command transform started: '
            f'{input_pose_topic} ({self.control_base_frame}->'
            f'{self.controlled_frame}) -> {output_pose_topic} '
            f'({self.control_base_frame}->{self.tool_frame})'
        )

    def _on_pose(self, msg: Pose) -> None:
        try:
            tool_to_controlled_msg = self.tf_buffer.lookup_transform(
                self.tool_frame,
                self.controlled_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                'TF lookup failed while converting ASM EE command: '
                f'{exc}',
                throttle_duration_sec=1.0,
            )
            return

        base_to_controlled = pose_msg_to_transform(msg)
        tool_to_controlled = tf_msg_to_transform(tool_to_controlled_msg)
        base_to_tool = compute_base_to_tool_command(
            base_to_controlled,
            tool_to_controlled,
        )
        self.pub.publish(transform_to_pose_msg(base_to_tool))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AsmEeCommandTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
