import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy.node import Node


def _quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


class TcpTwistEstimator(Node):
    def __init__(self) -> None:
        super().__init__('tcp_twist_estimator')

        self.declare_parameter('input_pose_topic', '/tcp_pose_broadcaster/pose')
        self.declare_parameter('output_pose_topic', '/UR5/ee_pose')
        self.declare_parameter('output_twist_topic', '/UR5/ee_twist')
        self.declare_parameter('min_dt', 1e-4)
        self.declare_parameter('max_angular_speed', 10.0)

        input_pose_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        output_pose_topic = self.get_parameter('output_pose_topic').get_parameter_value().string_value
        output_twist_topic = self.get_parameter('output_twist_topic').get_parameter_value().string_value

        self.min_dt = self.get_parameter('min_dt').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.prev_time: Optional[float] = None
        self.prev_pos: Optional[Tuple[float, float, float]] = None
        self.prev_quat: Optional[Tuple[float, float, float, float]] = None

        self.pose_sub = self.create_subscription(PoseStamped, input_pose_topic, self.pose_callback, 50)
        self.pose_pub = self.create_publisher(Pose, output_pose_topic, 50)
        self.twist_pub = self.create_publisher(Twist, output_twist_topic, 50)

        self.get_logger().info(
            f'TCP state estimator started. input={input_pose_topic}, '
            f'output_pose={output_pose_topic}, output_twist={output_twist_topic}'
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        pose_msg = Pose()
        pose_msg.position.x = msg.pose.position.x
        pose_msg.position.y = msg.pose.position.y
        pose_msg.position.z = msg.pose.position.z
        pose_msg.orientation.x = msg.pose.orientation.x
        pose_msg.orientation.y = msg.pose.orientation.y
        pose_msg.orientation.z = msg.pose.orientation.z
        pose_msg.orientation.w = msg.pose.orientation.w
        self.pose_pub.publish(pose_msg)

        cur_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        cur_pos = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        cur_quat = _quat_normalize((
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ))

        if self.prev_time is None or self.prev_pos is None or self.prev_quat is None:
            self.prev_time = cur_time
            self.prev_pos = cur_pos
            self.prev_quat = cur_quat
            return

        dt = cur_time - self.prev_time
        if dt < self.min_dt:
            return

        vx = (cur_pos[0] - self.prev_pos[0]) / dt
        vy = (cur_pos[1] - self.prev_pos[1]) / dt
        vz = (cur_pos[2] - self.prev_pos[2]) / dt

        q_prev_inv = _quat_conjugate(self.prev_quat)
        dq = _quat_normalize(_quat_multiply(q_prev_inv, cur_quat))

        if dq[3] < 0.0:
            dq = (-dq[0], -dq[1], -dq[2], -dq[3])

        sin_half = math.sqrt(max(0.0, dq[0] * dq[0] + dq[1] * dq[1] + dq[2] * dq[2]))
        angle = 2.0 * math.atan2(sin_half, max(1e-12, dq[3]))

        wx = 0.0
        wy = 0.0
        wz = 0.0
        if sin_half > 1e-9 and angle > 1e-9:
            axis_x = dq[0] / sin_half
            axis_y = dq[1] / sin_half
            axis_z = dq[2] / sin_half
            wx = axis_x * angle / dt
            wy = axis_y * angle / dt
            wz = axis_z * angle / dt

        omega_norm = math.sqrt(wx * wx + wy * wy + wz * wz)
        if omega_norm > self.max_angular_speed > 0.0:
            scale = self.max_angular_speed / omega_norm
            wx *= scale
            wy *= scale
            wz *= scale

        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.linear.y = vy
        twist_msg.linear.z = vz
        twist_msg.angular.x = wx
        twist_msg.angular.y = wy
        twist_msg.angular.z = wz
        self.twist_pub.publish(twist_msg)

        self.prev_time = cur_time
        self.prev_pos = cur_pos
        self.prev_quat = cur_quat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpTwistEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
