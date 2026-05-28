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

from tool_gravity_compensation.dynamic_gravity_model import (
    solve_dynamic_gravity_params,
)
from tool_gravity_compensation.gravity_model import quat_to_rot


class DynamicGravityCalibrationNode(Node):
    def __init__(self):
        super().__init__('dynamic_gravity_calibration_node')

        self.declare_parameter('wrench_topic', '/external_force_torque_wrench')
        self.declare_parameter('world_frame', 'base')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('samples_per_capture', 20)
        self.declare_parameter(
            'link_frames',
            ['asm_tool_base_link', 'asm_tool_link1', 'asm_tool_link2'],
        )
        self.declare_parameter(
            'tool_joint_names',
            ['brt_encoder1_joint', 'brt_encoder2_joint'],
        )
        self.declare_parameter(
            'output_file',
            '/home/liutiancheng/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_dynamic.json',
        )

        self.wrench_topic = str(self.get_parameter('wrench_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.samples_per_capture = max(1, int(self.get_parameter('samples_per_capture').value))
        self.link_frames = [str(value) for value in self.get_parameter('link_frames').value]
        self.tool_joint_names = [
            str(value) for value in self.get_parameter('tool_joint_names').value
        ]
        self.output_file = os.path.expanduser(str(self.get_parameter('output_file').value))

        if self.sample_period_sec <= 0.0:
            raise ValueError('sample_period_sec must be > 0')
        if not self.link_frames:
            raise ValueError('link_frames must not be empty')

        self.samples = []
        self.latest_wrench = None
        self._capture_remaining = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(WrenchStamped, self.wrench_topic, self._wrench_cb, 100)
        self.create_service(Trigger, '~/clear_samples', self._clear_samples_cb)
        self.create_service(Trigger, '~/collect_current_pose', self._collect_current_pose_cb)
        self.create_service(Trigger, '~/solve_and_save', self._solve_and_save_cb)
        self.create_service(Trigger, '~/sample_count', self._sample_count_cb)
        self.create_timer(self.sample_period_sec, self._sample_timer_cb)

        self.get_logger().info(
            'Dynamic gravity calibration node ready. '
            f'wrench={self.wrench_topic}, world={self.world_frame}, '
            f'sensor={self.sensor_frame}, links={self.link_frames}, '
            f'out={self.output_file}'
        )

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.latest_wrench = msg

    def _gravity_in_sensor(self):
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

    def _collect_sample(self):
        if self.latest_wrench is None:
            return None

        g_sensor = self._gravity_in_sensor()
        transforms = self._link_transform_snapshot()
        msg = self.latest_wrench
        return {
            'g_sensor': g_sensor,
            'force': np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ], dtype=float),
            'torque': np.array([
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ], dtype=float),
            'transforms': transforms,
        }

    def _sample_timer_cb(self) -> None:
        if self._capture_remaining <= 0:
            return

        try:
            sample = self._collect_sample()
        except TransformException as exc:
            self.get_logger().warn(
                f'Dynamic calibration TF lookup failed: {exc}',
                throttle_duration_sec=1.0,
            )
            return

        if sample is None:
            self.get_logger().warn(
                'Waiting for wrench data before collecting dynamic calibration samples.',
                throttle_duration_sec=1.0,
            )
            return

        self.samples.append(sample)
        self._capture_remaining -= 1
        if self._capture_remaining == 0:
            self.get_logger().info(
                f'Capture complete. total_samples={len(self.samples)}'
            )

    def _clear_samples_cb(self, _request, response):
        self.samples.clear()
        self._capture_remaining = 0
        response.success = True
        response.message = 'Dynamic calibration samples cleared.'
        return response

    def _collect_current_pose_cb(self, _request, response):
        if self._capture_remaining > 0:
            response.success = False
            response.message = (
                f'Capture already in progress. remaining={self._capture_remaining}'
            )
            return response

        self._capture_remaining = self.samples_per_capture
        response.success = True
        response.message = (
            f'Started capture for current pose. requested={self.samples_per_capture}, '
            f'current_total={len(self.samples)}'
        )
        self.get_logger().info(response.message)
        return response

    def _sample_count_cb(self, _request, response):
        response.success = True
        response.message = (
            f'sample_count={len(self.samples)}, capture_remaining={self._capture_remaining}'
        )
        return response

    def _solve_and_save_cb(self, _request, response):
        if self._capture_remaining > 0:
            response.success = False
            response.message = (
                f'Cannot solve while capture is in progress. remaining={self._capture_remaining}'
            )
            return response

        unknown_count = 6 + 4 * len(self.link_frames)
        min_samples = max(3, (unknown_count + 5) // 6)
        if len(self.samples) < min_samples:
            response.success = False
            response.message = (
                f'Not enough samples. current={len(self.samples)}, need>={min_samples}'
            )
            return response

        try:
            result = solve_dynamic_gravity_params(
                self.samples,
                link_count=len(self.link_frames),
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Dynamic least-squares solve failed: {exc}'
            self.get_logger().error(response.message)
            return response

        masses = np.asarray(result['link_masses'], dtype=float)
        coms = np.asarray(result['link_coms'], dtype=float)
        non_positive = [index for index, mass in enumerate(masses) if mass <= 0.0]
        if non_positive:
            self.get_logger().warn(
                f'Estimated non-positive link masses at indices {non_positive}. '
                'Increase posture diversity and verify TF/encoder signs.'
            )

        data = {
            'estimated_at': datetime.now().isoformat(),
            'mode': 'hardware_dynamic_service',
            'world_frame': self.world_frame,
            'sensor_frame': self.sensor_frame,
            'gravity_norm': self.gravity_norm,
            'sample_count': int(result['sample_count']),
            'rank': int(result['rank']),
            'rms_residual': float(result['rms_residual']),
            'link_frames': list(self.link_frames),
            'link_masses': [float(value) for value in masses],
            'link_com_x': [float(value) for value in coms[:, 0]],
            'link_com_y': [float(value) for value in coms[:, 1]],
            'link_com_z': [float(value) for value in coms[:, 2]],
            'force_bias': [float(value) for value in result['force_bias']],
            'torque_bias': [float(value) for value in result['torque_bias']],
            'tool_joint_names': list(self.tool_joint_names),
        }

        try:
            out_dir = os.path.dirname(self.output_file)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as file_obj:
                json.dump(data, file_obj, indent=2)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Failed to save dynamic calibration file: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'Dynamic solve done. samples={data["sample_count"]}, '
            f'rank={data["rank"]}, rms={data["rms_residual"]:.6f}, '
            f'file={self.output_file}'
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DynamicGravityCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
