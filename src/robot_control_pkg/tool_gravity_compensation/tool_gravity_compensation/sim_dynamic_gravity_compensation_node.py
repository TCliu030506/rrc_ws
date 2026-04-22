#!/usr/bin/env python3

import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import quat_to_rot


class SimDynamicGravityCompensationNode(Node):
    def __init__(self):
        super().__init__('sim_dynamic_gravity_compensation_node')

        self.declare_parameter('wrench_in_topic', '/wrench')
        self.declare_parameter('wrench_out_topic', '/wrench_compensated')
        self.declare_parameter('gravity_wrench_topic', '/wrench_gravity_model')
        self.declare_parameter('calibration_file', '/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_sim_dynamic.json')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('link_frames', ['asm_force_sensor_link', 'asm_tool_base_link', 'asm_tool_link1', 'asm_tool_link2'])
        self.declare_parameter('link_masses', [0.01707, 0.065358, 0.099171, 0.31471])
        self.declare_parameter('link_com_x', [-0.00034904, 3.1503e-15, -0.00057962, -0.00023235])
        self.declare_parameter('link_com_y', [0.00034904, 0.0, -0.00051221, 0.0001851])
        self.declare_parameter('link_com_z', [-6.9694e-05, 0.01886, -0.0055231, -0.0026657])
        self.declare_parameter('force_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('torque_bias', [0.0, 0.0, 0.0])

        self.wrench_in_topic = str(self.get_parameter('wrench_in_topic').value)
        self.wrench_out_topic = str(self.get_parameter('wrench_out_topic').value)
        self.gravity_wrench_topic = str(self.get_parameter('gravity_wrench_topic').value)
        self.calibration_file = os.path.expanduser(str(self.get_parameter('calibration_file').value))
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)

        self.link_frames = [str(v) for v in self.get_parameter('link_frames').value]
        self.link_masses = [float(v) for v in self.get_parameter('link_masses').value]
        self.link_com_x = [float(v) for v in self.get_parameter('link_com_x').value]
        self.link_com_y = [float(v) for v in self.get_parameter('link_com_y').value]
        self.link_com_z = [float(v) for v in self.get_parameter('link_com_z').value]
        self.force_bias = np.asarray(self.get_parameter('force_bias').value, dtype=float)
        self.torque_bias = np.asarray(self.get_parameter('torque_bias').value, dtype=float)

        if not (
            len(self.link_frames)
            == len(self.link_masses)
            == len(self.link_com_x)
            == len(self.link_com_y)
            == len(self.link_com_z)
        ):
            raise ValueError('link_frames, link_masses, link_com_* lengths must be equal')

        self.link_coms = [
            np.array([x, y, z], dtype=float)
            for x, y, z in zip(self.link_com_x, self.link_com_y, self.link_com_z)
        ]

        self._load_calibration_file()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(WrenchStamped, self.wrench_in_topic, self.wrench_cb, 100)
        self.pub_comp = self.create_publisher(WrenchStamped, self.wrench_out_topic, 100)
        self.pub_model = self.create_publisher(WrenchStamped, self.gravity_wrench_topic, 100)

        self.get_logger().info(
            'Dynamic gravity compensation ready. '
            f'in={self.wrench_in_topic}, out={self.wrench_out_topic}, '
            f'model={self.gravity_wrench_topic}, world={self.world_frame}, sensor={self.sensor_frame}'
        )

    def _refresh_link_coms(self):
        self.link_coms = [
            np.array([x, y, z], dtype=float)
            for x, y, z in zip(self.link_com_x, self.link_com_y, self.link_com_z)
        ]

    def _load_calibration_file(self):
        if not self.calibration_file:
            return
        if not os.path.exists(self.calibration_file):
            self.get_logger().info(
                f'Calibration file not found, using parameter defaults: {self.calibration_file}'
            )
            return

        try:
            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f'Failed to load calibration file {self.calibration_file}: {ex}')
            return

        if 'link_frames' in data:
            self.link_frames = [str(v) for v in data['link_frames']]
        if 'link_masses' in data:
            self.link_masses = [float(v) for v in data['link_masses']]
        if 'link_com_x' in data and 'link_com_y' in data and 'link_com_z' in data:
            self.link_com_x = [float(v) for v in data['link_com_x']]
            self.link_com_y = [float(v) for v in data['link_com_y']]
            self.link_com_z = [float(v) for v in data['link_com_z']]
            self._refresh_link_coms()
        if 'force_bias' in data:
            self.force_bias = np.asarray(data['force_bias'], dtype=float)
        if 'torque_bias' in data:
            self.torque_bias = np.asarray(data['torque_bias'], dtype=float)

        if len(self.link_frames) != len(self.link_coms):
            raise ValueError('Loaded calibration file has inconsistent link parameter lengths')

        self.get_logger().info(
            'Loaded dynamic calibration file: '
            f'{self.calibration_file}, links={len(self.link_frames)}'
        )

    def _lookup_rot_trans(self, target_frame: str, source_frame: str):
        if target_frame == source_frame:
            return np.eye(3), np.zeros(3)

        tf_msg = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
        )
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        return rot, trans

    def _gravity_vector_sensor(self):
        tf_msg = self.tf_buffer.lookup_transform(
            self.world_frame,
            self.sensor_frame,
            rclpy.time.Time(),
        )
        q = tf_msg.transform.rotation
        rot_ws = quat_to_rot(q.x, q.y, q.z, q.w)
        g_world = np.array([0.0, 0.0, -self.gravity_norm], dtype=float)
        return rot_ws.T @ g_world

    def _link_com_in_sensor(self, link_frame: str, com_local: np.ndarray):
        rot, trans = self._lookup_rot_trans(self.sensor_frame, link_frame)
        return trans + rot @ com_local

    def wrench_cb(self, msg: WrenchStamped):
        try:
            g_sensor = self._gravity_vector_sensor()
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return

        force_model = self.force_bias.copy()
        torque_model = self.torque_bias.copy()

        for link_frame, mass, com_local in zip(self.link_frames, self.link_masses, self.link_coms):
            try:
                com_sensor = self._link_com_in_sensor(link_frame, com_local)
            except TransformException as ex:
                self.get_logger().warn(
                    f'TF lookup failed for link {link_frame}: {ex}',
                    throttle_duration_sec=2.0,
                )
                return

            force_i = mass * g_sensor
            torque_i = np.cross(com_sensor, force_i)
            force_model += force_i
            torque_model += torque_i

        comp = WrenchStamped()
        comp.header = msg.header
        comp.wrench.force.x = float(msg.wrench.force.x - force_model[0])
        comp.wrench.force.y = float(msg.wrench.force.y - force_model[1])
        comp.wrench.force.z = float(msg.wrench.force.z - force_model[2])
        comp.wrench.torque.x = float(msg.wrench.torque.x - torque_model[0])
        comp.wrench.torque.y = float(msg.wrench.torque.y - torque_model[1])
        comp.wrench.torque.z = float(msg.wrench.torque.z - torque_model[2])
        self.pub_comp.publish(comp)

        model = WrenchStamped()
        model.header = msg.header
        model.wrench.force.x = float(force_model[0])
        model.wrench.force.y = float(force_model[1])
        model.wrench.force.z = float(force_model[2])
        model.wrench.torque.x = float(torque_model[0])
        model.wrench.torque.y = float(torque_model[1])
        model.wrench.torque.z = float(torque_model[2])
        self.pub_model.publish(model)


def main(args=None):
    rclpy.init(args=args)
    node = SimDynamicGravityCompensationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
