#!/usr/bin/env python3

import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import predict_gravity_wrench, quat_to_rot


class GravityCompensationNode(Node):
    def __init__(self):
        super().__init__('gravity_compensation_node')

        self.declare_parameter('wrench_in_topic', '/ft_sensor/wrench_raw')
        self.declare_parameter('wrench_out_topic', '/ft_sensor/wrench_compensated')
        self.declare_parameter('gravity_wrench_topic', '/ft_sensor/wrench_gravity_model')
        self.declare_parameter('sensor_frame', 'sensor_frame')
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('calibration_file', '~/.ros/tool_gravity_calibration.json')

        self.wrench_in_topic = self.get_parameter('wrench_in_topic').value
        self.wrench_out_topic = self.get_parameter('wrench_out_topic').value
        self.gravity_wrench_topic = self.get_parameter('gravity_wrench_topic').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.calibration_file = os.path.expanduser(str(self.get_parameter('calibration_file').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.params = self.load_calibration(self.calibration_file)

        self.sub = self.create_subscription(WrenchStamped, self.wrench_in_topic, self.wrench_cb, 100)
        self.pub_comp = self.create_publisher(WrenchStamped, self.wrench_out_topic, 100)
        self.pub_model = self.create_publisher(WrenchStamped, self.gravity_wrench_topic, 100)

        self.get_logger().info(
            f'Compensation node ready. in={self.wrench_in_topic}, out={self.wrench_out_topic}, '
            f'model={self.gravity_wrench_topic}, world={self.world_frame}, sensor={self.sensor_frame}'
        )

    def load_calibration(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'Calibration file does not exist: {path}')

        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        required = ['mass', 'com', 'force_bias', 'torque_bias']
        for key in required:
            if key not in data:
                raise ValueError(f'Missing key in calibration file: {key}')

        params = {
            'mass': float(data['mass']),
            'com': np.asarray(data['com'], dtype=float),
            'force_bias': np.asarray(data['force_bias'], dtype=float),
            'torque_bias': np.asarray(data['torque_bias'], dtype=float),
        }
        self.get_logger().info(
            f'Loaded calibration: mass={params["mass"]:.6f}, '
            f'com=[{params["com"][0]:.4f}, {params["com"][1]:.4f}, {params["com"][2]:.4f}]'
        )
        return params

    def gravity_sensor_from_tf(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.sensor_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return None

        q = tf_msg.transform.rotation
        rot_ws = quat_to_rot(q.x, q.y, q.z, q.w)

        g_world = np.array([0.0, 0.0, -self.gravity_norm], dtype=float)
        g_sensor = rot_ws.T @ g_world
        return g_sensor

    def wrench_cb(self, msg: WrenchStamped):
        g_sensor = self.gravity_sensor_from_tf()
        if g_sensor is None:
            return

        force_g, torque_g = predict_gravity_wrench(g_sensor, self.params)

        comp = WrenchStamped()
        comp.header = msg.header
        comp.wrench.force.x = float(msg.wrench.force.x - force_g[0])
        comp.wrench.force.y = float(msg.wrench.force.y - force_g[1])
        comp.wrench.force.z = float(msg.wrench.force.z - force_g[2])
        comp.wrench.torque.x = float(msg.wrench.torque.x - torque_g[0])
        comp.wrench.torque.y = float(msg.wrench.torque.y - torque_g[1])
        comp.wrench.torque.z = float(msg.wrench.torque.z - torque_g[2])
        self.pub_comp.publish(comp)

        model = WrenchStamped()
        model.header = msg.header
        model.wrench.force.x = float(force_g[0])
        model.wrench.force.y = float(force_g[1])
        model.wrench.force.z = float(force_g[2])
        model.wrench.torque.x = float(torque_g[0])
        model.wrench.torque.y = float(torque_g[1])
        model.wrench.torque.z = float(torque_g[2])
        self.pub_model.publish(model)


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
