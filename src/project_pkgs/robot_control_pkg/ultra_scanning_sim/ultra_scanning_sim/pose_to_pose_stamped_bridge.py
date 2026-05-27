import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def make_control_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class PoseToPoseStampedBridge(Node):
    def __init__(self) -> None:
        super().__init__('pose_to_pose_stamped_bridge')

        self.declare_parameter('input_pose_topic', '/admittance/cmd_pose')
        self.declare_parameter('output_pose_stamped_topic', '/cartesian_target_pose')
        self.declare_parameter('frame_id', 'world')

        input_topic = str(self.get_parameter('input_pose_topic').value)
        output_topic = str(self.get_parameter('output_pose_stamped_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        control_qos = make_control_qos(10)
        self.pub = self.create_publisher(PoseStamped, output_topic, control_qos)
        self.sub = self.create_subscription(Pose, input_topic, self._on_pose, 10)

        self.get_logger().info(
            f'Pose->PoseStamped bridge started: {input_topic} -> {output_topic}, frame_id={self.frame_id}'
        )

    def _on_pose(self, msg: Pose) -> None:
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.pose = msg
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseToPoseStampedBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
