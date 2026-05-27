import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
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


class EeStateFromTfNode(Node):
    def __init__(self) -> None:
        super().__init__('ee_state_from_tf_node')

        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'asm_force_sensor_link')
        self.declare_parameter('output_pose_topic', '/gazebo/ee_pose')
        self.declare_parameter('output_twist_topic', '/gazebo/ee_twist')
        self.declare_parameter('publish_rate', 125.0)
        self.declare_parameter('max_angular_speed', 10.0)

        self.source_frame = str(self.get_parameter('source_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self.output_twist_topic = str(self.get_parameter('output_twist_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(Pose, self.output_pose_topic, 20)
        self.twist_pub = self.create_publisher(Twist, self.output_twist_topic, 20)

        self.prev_time: Optional[float] = None
        self.prev_pos: Optional[Tuple[float, float, float]] = None
        self.prev_quat: Optional[Tuple[float, float, float, float]] = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'EE state from TF started. '
            f'{self.source_frame}->{self.target_frame}, '
            f'pose={self.output_pose_topic}, twist={self.output_twist_topic}'
        )

    def _on_timer(self) -> None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return

        stamp = tf_msg.header.stamp
        cur_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        cur_pos = (float(t.x), float(t.y), float(t.z))
        cur_quat = _quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w)))

        pose_msg = Pose()
        pose_msg.position.x = cur_pos[0]
        pose_msg.position.y = cur_pos[1]
        pose_msg.position.z = cur_pos[2]
        pose_msg.orientation.x = cur_quat[0]
        pose_msg.orientation.y = cur_quat[1]
        pose_msg.orientation.z = cur_quat[2]
        pose_msg.orientation.w = cur_quat[3]
        self.pose_pub.publish(pose_msg)

        if self.prev_time is None or self.prev_pos is None or self.prev_quat is None:
            self.prev_time = cur_time
            self.prev_pos = cur_pos
            self.prev_quat = cur_quat
            return

        dt = cur_time - self.prev_time
        if dt <= 1e-6:
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
    node = EeStateFromTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
