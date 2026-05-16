import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class PoseStampedToPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('pose_stamped_to_pose_bridge')

        self.declare_parameter('input_pose_stamped_topic', '/mujoco/ee_pose_stamped')
        self.declare_parameter('output_pose_topic', '/mujoco/ee_pose')
        self.declare_parameter('output_twist_topic', '/mujoco/ee_twist')

        input_topic = str(self.get_parameter('input_pose_stamped_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        output_twist_topic = str(self.get_parameter('output_twist_topic').value)

        self.pose_pub = self.create_publisher(Pose, output_pose_topic, 10)
        self.twist_pub = self.create_publisher(Twist, output_twist_topic, 10)
        self.sub = self.create_subscription(PoseStamped, input_topic, self._on_pose_stamped, qos_profile=qos_profile_sensor_data)

        self.last_pose = None
        self.last_time = None

        self.get_logger().info(
            f'PoseStamped->Pose bridge started: {input_topic} -> {output_pose_topic}, {output_twist_topic}'
        )

    def _on_pose_stamped(self, msg: PoseStamped) -> None:
        current_time = self.get_clock().now()
        current_pose = msg.pose

        # Publish Pose
        self.pose_pub.publish(current_pose)

        # Calculate and publish Twist if previous pose exists
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            if dt > 0:
                twist = Twist()
                twist.linear.x = (current_pose.position.x - self.last_pose.position.x) / dt
                twist.linear.y = (current_pose.position.y - self.last_pose.position.y) / dt
                twist.linear.z = (current_pose.position.z - self.last_pose.position.z) / dt
                # Angular velocity calculation can be added if needed
                self.twist_pub.publish(twist)

        # Update last pose and time
        self.last_pose = current_pose
        self.last_time = current_time


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseStampedToPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()