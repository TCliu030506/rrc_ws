#!/usr/bin/env python3

import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import AccelStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.dynamic_gravity_model import (
    aggregate_rigid_body_params,
    predict_dynamic_gravity_wrench,
    predict_rigid_body_inertia_wrench,
)
from tool_gravity_compensation.gravity_model import quat_to_rot


class DynamicGravityCompensationNode(Node):
    def __init__(self):
        super().__init__('dynamic_gravity_compensation_node')

        self.declare_parameter('wrench_in_topic', '/external_force_torque_wrench')
        self.declare_parameter('wrench_out_topic', '/external_force_torque_wrench_compensated')
        self.declare_parameter('gravity_wrench_topic', '/external_gravity_compensation_wrench_model')
        self.declare_parameter('enable_inertia_compensation', False)
        self.declare_parameter('accel_topic', '/asm_force_sensor_link/accel')
        self.declare_parameter('inertia_wrench_topic', '/external_inertia_compensation_wrench_model')
        self.declare_parameter('max_inertia_force', 30.0)
        self.declare_parameter('max_inertia_torque', 5.0)
        self.declare_parameter('max_accel_age_sec', 0.2)
        self.declare_parameter(
            'calibration_file',
            '/home/liutiancheng/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_dynamic.json',
        )
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter(
            'link_frames',
            ['asm_tool_base_link', 'asm_tool_link1', 'asm_tool_link2'],
        )
        self.declare_parameter('link_masses', [0.0, 0.0, 0.0])
        self.declare_parameter('link_com_x', [0.0, 0.0, 0.0])
        self.declare_parameter('link_com_y', [0.0, 0.0, 0.0])
        self.declare_parameter('link_com_z', [0.0, 0.0, 0.0])
        self.declare_parameter('force_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('torque_bias', [0.0, 0.0, 0.0])

        self.wrench_in_topic = str(self.get_parameter('wrench_in_topic').value)
        self.wrench_out_topic = str(self.get_parameter('wrench_out_topic').value)
        self.gravity_wrench_topic = str(self.get_parameter('gravity_wrench_topic').value)
        self.enable_inertia_compensation = bool(
            self.get_parameter('enable_inertia_compensation').value
        )
        self.accel_topic = str(self.get_parameter('accel_topic').value)
        self.inertia_wrench_topic = str(self.get_parameter('inertia_wrench_topic').value)
        self.max_inertia_force = float(self.get_parameter('max_inertia_force').value)
        self.max_inertia_torque = float(self.get_parameter('max_inertia_torque').value)
        self.max_accel_age_sec = float(self.get_parameter('max_accel_age_sec').value)
        self.calibration_file = os.path.expanduser(str(self.get_parameter('calibration_file').value))
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.link_frames = [str(value) for value in self.get_parameter('link_frames').value]
        self.link_masses = [float(value) for value in self.get_parameter('link_masses').value]
        self.link_com_x = [float(value) for value in self.get_parameter('link_com_x').value]
        self.link_com_y = [float(value) for value in self.get_parameter('link_com_y').value]
        self.link_com_z = [float(value) for value in self.get_parameter('link_com_z').value]
        self.force_bias = np.asarray(self.get_parameter('force_bias').value, dtype=float)
        self.torque_bias = np.asarray(self.get_parameter('torque_bias').value, dtype=float)
        self._refresh_link_coms()
        self._load_calibration_file()
        self._validate_lengths()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_accel = None
        self.sub = self.create_subscription(WrenchStamped, self.wrench_in_topic, self.wrench_cb, 100)
        if self.enable_inertia_compensation:
            self.accel_sub = self.create_subscription(
                AccelStamped,
                self.accel_topic,
                self.accel_cb,
                100,
            )
        self.pub_comp = self.create_publisher(WrenchStamped, self.wrench_out_topic, 100)
        self.pub_model = self.create_publisher(WrenchStamped, self.gravity_wrench_topic, 100)
        self.pub_inertia_model = self.create_publisher(WrenchStamped, self.inertia_wrench_topic, 100)

        self.get_logger().info(
            'Dynamic gravity compensation ready. '
            f'in={self.wrench_in_topic}, out={self.wrench_out_topic}, '
            f'model={self.gravity_wrench_topic}, world={self.world_frame}, '
            f'sensor={self.sensor_frame}, links={self.link_frames}, '
            f'inertia_enabled={self.enable_inertia_compensation}, '
            f'accel={self.accel_topic}'
        )

    def _refresh_link_coms(self):
        self.link_coms = [
            np.array([x, y, z], dtype=float)
            for x, y, z in zip(self.link_com_x, self.link_com_y, self.link_com_z)
        ]

    def _validate_lengths(self):
        lengths = {
            len(self.link_frames),
            len(self.link_masses),
            len(self.link_com_x),
            len(self.link_com_y),
            len(self.link_com_z),
            len(self.link_coms),
        }
        if len(lengths) != 1:
            raise ValueError('link_frames, link_masses, and link_com_* lengths must match')
        if self.force_bias.shape != (3,) or self.torque_bias.shape != (3,):
            raise ValueError('force_bias and torque_bias must have length 3')

    def _load_calibration_file(self):
        if not self.calibration_file:
            return
        if not os.path.exists(self.calibration_file):
            self.get_logger().warn(
                f'Dynamic calibration file not found; using parameter defaults: {self.calibration_file}'
            )
            return

        try:
            with open(self.calibration_file, 'r', encoding='utf-8') as file_obj:
                data = json.load(file_obj)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to load dynamic calibration file: {exc}')
            return

        if 'link_frames' in data:
            self.link_frames = [str(value) for value in data['link_frames']]
        if 'link_masses' in data:
            self.link_masses = [float(value) for value in data['link_masses']]
        if all(key in data for key in ('link_com_x', 'link_com_y', 'link_com_z')):
            self.link_com_x = [float(value) for value in data['link_com_x']]
            self.link_com_y = [float(value) for value in data['link_com_y']]
            self.link_com_z = [float(value) for value in data['link_com_z']]
            self._refresh_link_coms()
        if 'force_bias' in data:
            self.force_bias = np.asarray(data['force_bias'], dtype=float)
        if 'torque_bias' in data:
            self.torque_bias = np.asarray(data['torque_bias'], dtype=float)

        self.get_logger().info(
            f'Loaded dynamic calibration file: {self.calibration_file}'
        )

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

    def _link_transform_snapshot(self):
        transforms = []
        for link_frame in self.link_frames:
            tf_msg = self.tf_buffer.lookup_transform(
                self.sensor_frame,
                link_frame,
                rclpy.time.Time(),
            )
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            transforms.append({
                'rot': quat_to_rot(q.x, q.y, q.z, q.w),
                'trans': np.array([t.x, t.y, t.z], dtype=float),
            })
        return transforms

    def _params(self):
        return {
            'link_masses': self.link_masses,
            'link_coms': self.link_coms,
            'force_bias': self.force_bias,
            'torque_bias': self.torque_bias,
        }

    def accel_cb(self, msg: AccelStamped):
        if msg.header.frame_id and msg.header.frame_id != self.sensor_frame:
            self.get_logger().warn(
                f'Ignoring accel frame {msg.header.frame_id}; expected {self.sensor_frame}',
                throttle_duration_sec=2.0,
            )
            return
        self.latest_accel = msg

    def _limit_vector(self, vector, max_norm):
        if max_norm <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm <= max_norm or norm <= 1e-12:
            return vector
        return vector * (max_norm / norm)

    def _inertia_wrench(self, transforms):
        if not self.enable_inertia_compensation or self.latest_accel is None:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)
        if self.max_accel_age_sec > 0.0:
            accel_age = (
                self.get_clock().now() - Time.from_msg(self.latest_accel.header.stamp)
            ).nanoseconds * 1e-9
            if accel_age > self.max_accel_age_sec:
                self.get_logger().warn(
                    f'Ignoring stale accel sample. age={accel_age:.3f}s',
                    throttle_duration_sec=2.0,
                )
                return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        total_mass, com_sensor = aggregate_rigid_body_params(
            self.link_masses,
            self.link_coms,
            transforms,
        )
        accel = self.latest_accel.accel.linear
        linear_accel_sensor = np.array([accel.x, accel.y, accel.z], dtype=float)
        force_model, torque_model = predict_rigid_body_inertia_wrench(
            total_mass,
            com_sensor,
            linear_accel_sensor,
        )
        force_model = self._limit_vector(force_model, self.max_inertia_force)
        torque_model = self._limit_vector(torque_model, self.max_inertia_torque)
        return force_model, torque_model

    def wrench_cb(self, msg: WrenchStamped):
        try:
            g_sensor = self._gravity_vector_sensor()
            transforms = self._link_transform_snapshot()
            gravity_force_model, gravity_torque_model = predict_dynamic_gravity_wrench(
                g_sensor,
                transforms,
                self._params(),
            )
            inertia_force_model, inertia_torque_model = self._inertia_wrench(transforms)
            force_model = gravity_force_model + inertia_force_model
            torque_model = gravity_torque_model + inertia_torque_model
        except TransformException as exc:
            self.get_logger().warn(f'Dynamic compensation TF lookup failed: {exc}', throttle_duration_sec=1.0)
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Dynamic compensation failed: {exc}', throttle_duration_sec=1.0)
            return

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
        model.wrench.force.x = float(gravity_force_model[0])
        model.wrench.force.y = float(gravity_force_model[1])
        model.wrench.force.z = float(gravity_force_model[2])
        model.wrench.torque.x = float(gravity_torque_model[0])
        model.wrench.torque.y = float(gravity_torque_model[1])
        model.wrench.torque.z = float(gravity_torque_model[2])
        self.pub_model.publish(model)

        inertia_model = WrenchStamped()
        inertia_model.header = msg.header
        inertia_model.wrench.force.x = float(inertia_force_model[0])
        inertia_model.wrench.force.y = float(inertia_force_model[1])
        inertia_model.wrench.force.z = float(inertia_force_model[2])
        inertia_model.wrench.torque.x = float(inertia_torque_model[0])
        inertia_model.wrench.torque.y = float(inertia_torque_model[1])
        inertia_model.wrench.torque.z = float(inertia_torque_model[2])
        self.pub_inertia_model.publish(inertia_model)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicGravityCompensationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
