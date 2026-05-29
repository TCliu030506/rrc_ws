import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from ur5_state_broadcaster.frame_motion_estimator_math import FrameMotionEstimator


class FrameMotionFromTfNode(Node):
    def __init__(self) -> None:
        super().__init__('frame_motion_from_tf_node')

        self.declare_parameter('source_frame', 'base')
        self.declare_parameter('target_frame', 'tool0')
        self.declare_parameter('output_pose_topic', '/tool0/pose')
        self.declare_parameter('output_twist_topic', '/tool0/twist')
        self.declare_parameter('output_accel_topic', '/tool0/accel')
        self.declare_parameter('publish_rate', 125.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.01)
        self.declare_parameter('express_in_target_frame', True)
        self.declare_parameter('min_dt', 1e-4)
        self.declare_parameter('max_dt', 0.2)
        self.declare_parameter('velocity_filter_tau', 0.002)
        self.declare_parameter('accel_filter_tau', 0.005)
        self.declare_parameter('max_linear_speed', 50.0)
        self.declare_parameter('max_angular_speed', 200.0)
        self.declare_parameter('max_linear_accel', 300.0)
        self.declare_parameter('max_angular_accel', 800.0)

        self.source_frame = str(self.get_parameter('source_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        output_twist_topic = str(self.get_parameter('output_twist_topic').value)
        output_accel_topic = str(self.get_parameter('output_accel_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self.express_in_target_frame = bool(
            self.get_parameter('express_in_target_frame').value
        )

        self.estimator = FrameMotionEstimator(
            min_dt=float(self.get_parameter('min_dt').value),
            max_dt=float(self.get_parameter('max_dt').value),
            velocity_filter_tau=float(self.get_parameter('velocity_filter_tau').value),
            accel_filter_tau=float(self.get_parameter('accel_filter_tau').value),
            max_linear_speed=float(self.get_parameter('max_linear_speed').value),
            max_angular_speed=float(self.get_parameter('max_angular_speed').value),
            max_linear_accel=float(self.get_parameter('max_linear_accel').value),
            max_angular_accel=float(self.get_parameter('max_angular_accel').value),
            express_in_target_frame=self.express_in_target_frame,
        )

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, output_pose_topic, 10)
        self.twist_pub = self.create_publisher(TwistStamped, output_twist_topic, 10)
        self.accel_pub = self.create_publisher(AccelStamped, output_accel_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        output_frame = self.target_frame if self.express_in_target_frame else self.source_frame
        self.get_logger().info(
            'Frame motion from TF started. '
            f'{self.source_frame}->{self.target_frame}, '
            f'pose={output_pose_topic}, twist={output_twist_topic}, '
            f'accel={output_accel_topic}, output_frame={output_frame}, '
            f'rate={publish_rate:.3f} Hz'
        )

    def _on_timer(self) -> None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF lookup failed: {exc}',
                throttle_duration_sec=1.0,
            )
            self.estimator.reset()
            return

        stamp_sec = (
            float(tf_msg.header.stamp.sec)
            + float(tf_msg.header.stamp.nanosec) * 1e-9
        )
        transform = tf_msg.transform
        position = (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        )
        orientation = (
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        )

        estimate = self.estimator.update(stamp_sec, position, orientation)
        self.pose_pub.publish(self._pose_msg(tf_msg))
        if estimate is None:
            return

        output_frame = self.target_frame if self.express_in_target_frame else self.source_frame
        self.twist_pub.publish(self._twist_msg(tf_msg.header.stamp, output_frame, estimate))
        self.accel_pub.publish(self._accel_msg(tf_msg.header.stamp, output_frame, estimate))

    def _pose_msg(self, tf_msg) -> PoseStamped:
        msg = PoseStamped()
        msg.header = tf_msg.header
        msg.pose.position.x = tf_msg.transform.translation.x
        msg.pose.position.y = tf_msg.transform.translation.y
        msg.pose.position.z = tf_msg.transform.translation.z
        msg.pose.orientation = tf_msg.transform.rotation
        return msg

    def _twist_msg(self, stamp, frame_id: str, estimate) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x = float(estimate.linear_velocity[0])
        msg.twist.linear.y = float(estimate.linear_velocity[1])
        msg.twist.linear.z = float(estimate.linear_velocity[2])
        msg.twist.angular.x = float(estimate.angular_velocity[0])
        msg.twist.angular.y = float(estimate.angular_velocity[1])
        msg.twist.angular.z = float(estimate.angular_velocity[2])
        return msg

    def _accel_msg(self, stamp, frame_id: str, estimate) -> AccelStamped:
        msg = AccelStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.accel.linear.x = float(estimate.linear_acceleration[0])
        msg.accel.linear.y = float(estimate.linear_acceleration[1])
        msg.accel.linear.z = float(estimate.linear_acceleration[2])
        msg.accel.angular.x = float(estimate.angular_acceleration[0])
        msg.accel.angular.y = float(estimate.angular_acceleration[1])
        msg.accel.angular.z = float(estimate.angular_acceleration[2])
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrameMotionFromTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
