#!/usr/bin/env python3

import json
import os
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import quat_to_rot, solve_gravity_params


class GravityCalibrationNode(Node):
    def __init__(self):
        super().__init__('gravity_calibration_node')

        self.declare_parameter('wrench_topic', '/ft_sensor/wrench_raw')
        self.declare_parameter('sensor_frame', 'sensor_frame')
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('auto_collect', False)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('output_file', '~/.ros/tool_gravity_calibration.json')

        self.wrench_topic = self.get_parameter('wrench_topic').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.auto_collect = bool(self.get_parameter('auto_collect').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.output_file = os.path.expanduser(str(self.get_parameter('output_file').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples = []
        self.collecting = self.auto_collect
        self.latest_wrench = None
        self.last_sample_time_ns = 0

        self.sub = self.create_subscription(WrenchStamped, self.wrench_topic, self.wrench_cb, 50)

        self.start_srv = self.create_service(Trigger, 'start_collection', self.start_collection_cb)
        self.stop_srv = self.create_service(Trigger, 'stop_collection', self.stop_collection_cb)
        self.solve_srv = self.create_service(Trigger, 'solve_and_save', self.solve_and_save_cb)
        self.clear_srv = self.create_service(Trigger, 'clear_samples', self.clear_samples_cb)

        # 新增：采集一帧数据的服务
        self.collect_one_srv = self.create_service(Trigger, 'collect_one_sample', self.collect_one_sample_cb)
    def collect_one_sample_cb(self, _req, res):
        """采集一帧数据（仅采集一次）"""
        # 获取当前wrench和重力
        msg = self.latest_wrench
        if msg is None:
            res.success = False
            res.message = 'No wrench data received yet.'
            return res

        g_sensor = self.try_get_gravity_sensor()
        if g_sensor is None:
            res.success = False
            res.message = 'TF lookup failed.'
            return res

        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ], dtype=float)
        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float)

        self.samples.append(
            {
                'g_sensor': g_sensor,
                'force': force,
                'torque': torque,
            }
        )
        res.success = True
        res.message = f'Collected one sample. Total samples: {len(self.samples)}'
        self.get_logger().info(res.message)
        return res

        self.get_logger().info(
            f'Calibration node ready. wrench_topic={self.wrench_topic}, '
            f'world_frame={self.world_frame}, sensor_frame={self.sensor_frame}, '
            f'auto_collect={self.auto_collect}'
        )

    def wrench_cb(self, msg: WrenchStamped):
        self.latest_wrench = msg
        if not self.collecting:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.last_sample_time_ns != 0:
            dt = (now_ns - self.last_sample_time_ns) * 1e-9
            if dt < self.sample_period_sec:
                return

        g_sensor = self.try_get_gravity_sensor()
        if g_sensor is None:
            return

        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ], dtype=float)
        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float)

        self.samples.append(
            {
                'g_sensor': g_sensor,
                'force': force,
                'torque': torque,
            }
        )
        self.last_sample_time_ns = now_ns

        if len(self.samples) % 20 == 0:
            self.get_logger().info(f'Collected samples: {len(self.samples)}')

    def try_get_gravity_sensor(self):
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

    def start_collection_cb(self, _req, res):
        self.samples.clear()
        self.collecting = True
        self.last_sample_time_ns = 0
        res.success = True
        res.message = 'Collection started and old samples cleared.'
        self.get_logger().info(res.message)
        return res

    def stop_collection_cb(self, _req, res):
        self.collecting = False
        res.success = True
        res.message = f'Collection stopped. sample_count={len(self.samples)}'
        self.get_logger().info(res.message)
        return res

    def clear_samples_cb(self, _req, res):
        self.samples.clear()
        self.last_sample_time_ns = 0
        res.success = True
        res.message = 'Samples cleared.'
        self.get_logger().info(res.message)
        return res

    def solve_and_save_cb(self, _req, res):
        if len(self.samples) < 6:
            res.success = False
            res.message = f'Not enough samples. current={len(self.samples)}, need>=6'
            return res

        try:
            result = solve_gravity_params(self.samples)
        except Exception as ex:  # noqa: BLE001
            res.success = False
            res.message = f'Least-squares solve failed: {ex}'
            self.get_logger().error(res.message)
            return res

        data = {
            'estimated_at': datetime.now().isoformat(),
            'world_frame': self.world_frame,
            'sensor_frame': self.sensor_frame,
            'gravity_norm': self.gravity_norm,
            'sample_count': int(result['sample_count']),
            'rms_residual': float(result['rms_residual']),
            'rank': int(result['rank']),
            'mass': float(result['mass']),
            'com': [float(v) for v in result['com']],
            'force_bias': [float(v) for v in result['force_bias']],
            'torque_bias': [float(v) for v in result['torque_bias']],
        }

        try:
            out_dir = os.path.dirname(self.output_file)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except Exception as ex:  # noqa: BLE001
            res.success = False
            res.message = f'Failed to save file {self.output_file}: {ex}'
            self.get_logger().error(res.message)
            return res

        res.success = True
        res.message = (
            f'Solve done. mass={data["mass"]:.6f} kg, '
            f'com=[{data["com"][0]:.4f}, {data["com"][1]:.4f}, {data["com"][2]:.4f}] m, '
            f'rms={data["rms_residual"]:.6f}, file={self.output_file}'
        )
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = GravityCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
