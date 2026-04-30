from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


class Ur5PoseTargetServoBridge(Node):
    def __init__(self) -> None:
        super().__init__('ur5_pose_target_servo_bridge')

        self.declare_parameter('input_pose_topic', '/admittance/cmd_pose')
        self.declare_parameter('current_pose_topic', '/gazebo/ee_pose')
        self.declare_parameter('servo_twist_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('tf_wait_timeout_sec', 0.2)
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_linear_velocity', 0.15)
        self.declare_parameter('max_angular_velocity', 0.75)
        self.declare_parameter('position_deadband', 0.002)
        self.declare_parameter('orientation_deadband', 0.01)
        self.declare_parameter('publish_zero_when_idle', True)

        self.input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        self.current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        self.servo_twist_topic = str(self.get_parameter('servo_twist_topic').value)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.ee_frame = str(self.get_parameter('ee_frame').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.tf_wait_timeout_sec = float(self.get_parameter('tf_wait_timeout_sec').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)
        self.position_deadband = float(self.get_parameter('position_deadband').value)
        self.orientation_deadband = float(self.get_parameter('orientation_deadband').value)
        self.publish_zero_when_idle = bool(self.get_parameter('publish_zero_when_idle').value)

        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be > 0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(TwistStamped, self.servo_twist_topic, 10)
        self.pose_sub = self.create_subscription(Pose, self.input_pose_topic, self._pose_cb, 10)
        self.current_pose_sub = self.create_subscription(Pose, self.current_pose_topic, self._current_pose_cb, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._tick)

        self.latest_target_pose: Optional[Pose] = None
        self.latest_current_pose: Optional[Pose] = None
        self.last_target_time = self.get_clock().now()

        self.get_logger().info(
            'UR5 pose-to-Servo bridge started: '
            f'pose={self.input_pose_topic}, current_pose={self.current_pose_topic}, servo_twist={self.servo_twist_topic}, '
            f'frame={self.planning_frame}->{self.ee_frame}, rate={self.control_rate_hz}Hz'
        )

    def _pose_cb(self, msg: Pose) -> None:
        self.latest_target_pose = msg
        self.last_target_time = self.get_clock().now()

    def _current_pose_cb(self, msg: Pose) -> None:
        self.latest_current_pose = msg

    def _tick(self) -> None:
        if self.latest_target_pose is None:
            if self.publish_zero_when_idle:
                self._publish_zero_twist()
            return

        try:
            current_pose = self._get_current_pose()
        except TransformException as exc:
            self.get_logger().warn(
                f'Unable to read TF {self.planning_frame}->{self.ee_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            if self.publish_zero_when_idle:
                self._publish_zero_twist()
            return

        target_pose = self.latest_target_pose
        assert target_pose is not None

        linear_error = (
            float(target_pose.position.x) - float(current_pose.position.x),
            float(target_pose.position.y) - float(current_pose.position.y),
            float(target_pose.position.z) - float(current_pose.position.z),
        )

        target_quat = self._normalize_quaternion(
            float(target_pose.orientation.x),
            float(target_pose.orientation.y),
            float(target_pose.orientation.z),
            float(target_pose.orientation.w),
        )
        current_quat = self._normalize_quaternion(
            float(current_pose.orientation.x),
            float(current_pose.orientation.y),
            float(current_pose.orientation.z),
            float(current_pose.orientation.w),
        )

        quat_error = self._quat_multiply(target_quat, self._quat_conjugate(current_quat))
        if quat_error[3] < 0.0:
            quat_error = tuple(-value for value in quat_error)
        rot_error = self._quat_to_rotvec(quat_error)

        pos_error_norm = self._vector_norm(linear_error)
        rot_error_norm = self._vector_norm(rot_error)

        if pos_error_norm <= self.position_deadband and rot_error_norm <= self.orientation_deadband:
            self._publish_zero_twist()
            return

        linear_cmd = self._limit_vector(self._scale_vector(linear_error, self.linear_gain), self.max_linear_velocity)
        angular_cmd = self._limit_vector(self._scale_vector(rot_error, self.angular_gain), self.max_angular_velocity)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        twist.twist.linear.x = linear_cmd[0]
        twist.twist.linear.y = linear_cmd[1]
        twist.twist.linear.z = linear_cmd[2]
        twist.twist.angular.x = angular_cmd[0]
        twist.twist.angular.y = angular_cmd[1]
        twist.twist.angular.z = angular_cmd[2]
        self.twist_pub.publish(twist)

    def _get_current_pose(self) -> Pose:
        if self.latest_current_pose is not None:
            return self.latest_current_pose
        return self._lookup_current_pose()

    def _lookup_current_pose(self) -> Pose:
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            self.ee_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_wait_timeout_sec),
        )

        pose = Pose()
        pose.position.x = float(transform.transform.translation.x)
        pose.position.y = float(transform.transform.translation.y)
        pose.position.z = float(transform.transform.translation.z)
        pose.orientation.x = float(transform.transform.rotation.x)
        pose.orientation.y = float(transform.transform.rotation.y)
        pose.orientation.z = float(transform.transform.rotation.z)
        pose.orientation.w = float(transform.transform.rotation.w)
        return pose

    def _publish_zero_twist(self) -> None:
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        self.twist_pub.publish(twist)

    @staticmethod
    def _normalize_quaternion(x: float, y: float, z: float, w: float) -> Quaternion:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return x / norm, y / norm, z / norm, w / norm

    @staticmethod
    def _quat_conjugate(q: Quaternion) -> Quaternion:
        return -q[0], -q[1], -q[2], q[3]

    @staticmethod
    def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _quat_to_rotvec(q: Quaternion) -> Vector3:
        x, y, z, w = Ur5PoseTargetServoBridge._normalize_quaternion(*q)
        w = max(-1.0, min(1.0, w))
        angle = 2.0 * math.acos(w)
        sin_half = math.sqrt(max(0.0, 1.0 - w * w))
        if sin_half < 1e-9 or angle < 1e-9:
            return 0.0, 0.0, 0.0
        scale = angle / sin_half
        return x * scale, y * scale, z * scale

    @staticmethod
    def _scale_vector(vec: Vector3, scale: float) -> Vector3:
        return vec[0] * scale, vec[1] * scale, vec[2] * scale

    @staticmethod
    def _vector_norm(vec: Vector3) -> float:
        return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])

    @classmethod
    def _limit_vector(cls, vec: Vector3, max_norm: float) -> Vector3:
        norm = cls._vector_norm(vec)
        if norm <= max_norm or norm < 1e-12:
            return vec
        scale = max_norm / norm
        return vec[0] * scale, vec[1] * scale, vec[2] * scale


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur5PoseTargetServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
