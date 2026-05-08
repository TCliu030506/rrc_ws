#!/usr/bin/env python3
"""Transform wrench topics from sensor frame to target frame.

Subscribes:
- /wrench
- /wrench_compensated

Publishes transformed wrench topics in base frame by default:
- /wrench_base_link
- /wrench_compensated_base_link
"""

from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WrenchFrameTransformNode(Node):
    def __init__(self) -> None:
        super().__init__('wrench_frame_transform_node')

        self.declare_parameter('input_wrench_topic', '/wrench')
        self.declare_parameter('input_wrench_comp_topic', '/wrench_compensated')
        self.declare_parameter('output_wrench_topic', '/wrench_base_link')
        self.declare_parameter('output_wrench_comp_topic', '/wrench_compensated_base_link')
        self.declare_parameter('source_frame', 'asm_force_sensor_link')
        self.declare_parameter('target_frame', 'base_link')

        self._input_wrench_topic = str(self.get_parameter('input_wrench_topic').value)
        self._input_wrench_comp_topic = str(self.get_parameter('input_wrench_comp_topic').value)
        self._output_wrench_topic = str(self.get_parameter('output_wrench_topic').value)
        self._output_wrench_comp_topic = str(self.get_parameter('output_wrench_comp_topic').value)
        self._source_frame = str(self.get_parameter('source_frame').value)
        self._target_frame = str(self.get_parameter('target_frame').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_wrench = self.create_publisher(WrenchStamped, self._output_wrench_topic, 20)
        self._pub_wrench_comp = self.create_publisher(WrenchStamped, self._output_wrench_comp_topic, 20)

        self._sub_wrench = self.create_subscription(
            WrenchStamped,
            self._input_wrench_topic,
            self._on_wrench,
            20,
        )
        self._sub_wrench_comp = self.create_subscription(
            WrenchStamped,
            self._input_wrench_comp_topic,
            self._on_wrench_comp,
            20,
        )

        self.get_logger().info(
            'Wrench transformer started: '
            f'{self._input_wrench_topic}->{self._output_wrench_topic}, '
            f'{self._input_wrench_comp_topic}->{self._output_wrench_comp_topic}, '
            f'frame {self._source_frame}->{self._target_frame}'
        )

    def _on_wrench(self, msg: WrenchStamped) -> None:
        out = self._transform_wrench(msg)
        if out is not None:
            self._pub_wrench.publish(out)

    def _on_wrench_comp(self, msg: WrenchStamped) -> None:
        out = self._transform_wrench(msg)
        if out is not None:
            self._pub_wrench_comp.publish(out)

    @staticmethod
    def _quat_to_rot(qx: float, qy: float, qz: float, qw: float):
        # Rotation matrix from quaternion.
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    @staticmethod
    def _mat_vec_mul(m, v):
        return [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]

    @staticmethod
    def _cross(a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    def _transform_wrench(self, msg: WrenchStamped) -> Optional[WrenchStamped]:
        source_frame = msg.header.frame_id if msg.header.frame_id else self._source_frame

        if source_frame == self._target_frame:
            out = WrenchStamped()
            out.header = msg.header
            out.header.frame_id = self._target_frame
            out.wrench = msg.wrench
            return out

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f'Wrench TF lookup failed ({source_frame}->{self._target_frame}): {ex}', throttle_duration_sec=2.0)
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        p = [float(t.x), float(t.y), float(t.z)]
        r = self._quat_to_rot(float(q.x), float(q.y), float(q.z), float(q.w))

        f_src = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
        ]
        tau_src = [
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ]

        # Wrench transform with translation coupling:
        # F_t = R * F_s
        # T_t = R * T_s + p x (R * F_s)
        f_t = self._mat_vec_mul(r, f_src)
        tau_rot = self._mat_vec_mul(r, tau_src)
        tau_t = [
            tau_rot[0],
            tau_rot[1],
            tau_rot[2],
        ]
        pxf = self._cross(p, f_t)
        tau_t[0] += pxf[0]
        tau_t[1] += pxf[1]
        tau_t[2] += pxf[2]

        out = WrenchStamped()
        out.header = msg.header
        out.header.frame_id = self._target_frame
        out.wrench.force.x = f_t[0]
        out.wrench.force.y = f_t[1]
        out.wrench.force.z = f_t[2]
        out.wrench.torque.x = tau_t[0]
        out.wrench.torque.y = tau_t[1]
        out.wrench.torque.z = tau_t[2]
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WrenchFrameTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
