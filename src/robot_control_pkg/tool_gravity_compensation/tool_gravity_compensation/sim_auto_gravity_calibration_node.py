#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import quat_to_rot, solve_gravity_params


class SimAutoGravityCalibrationNode(Node):
    def __init__(self):
        super().__init__('sim_auto_gravity_calibration_node')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wrench_topic', '/wrench')
        self.declare_parameter('trajectory_action', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('samples_per_pose', 20)
        self.declare_parameter('move_time_sec', 1.2)
        self.declare_parameter('settle_time_sec', 0.8)
        self.declare_parameter('wrist_delta_rad', 0.35)
        self.declare_parameter('output_file', '/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_sim.json')

        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.wrench_topic = str(self.get_parameter('wrench_topic').value)
        self.trajectory_action = str(self.get_parameter('trajectory_action').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.samples_per_pose = int(self.get_parameter('samples_per_pose').value)
        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.wrist_delta_rad = float(self.get_parameter('wrist_delta_rad').value)
        self.output_file = os.path.expanduser(str(self.get_parameter('output_file').value))

        self.ur_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.latest_joint_map = {}
        self.latest_wrench = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_cb, 50)
        self.create_subscription(
            # geometry_msgs/msg/WrenchStamped
            __import__('geometry_msgs.msg', fromlist=['WrenchStamped']).WrenchStamped,
            self.wrench_topic,
            self._wrench_cb,
            100,
        )

        self.action_client = ActionClient(self, FollowJointTrajectory, self.trajectory_action)

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_map[name] = float(pos)

    def _wrench_cb(self, msg) -> None:
        self.latest_wrench = msg

    def _spin_wait(self, seconds: float) -> None:
        end_t = time.time() + max(0.0, seconds)
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_until_ready(self) -> None:
        self.get_logger().info('Waiting for trajectory action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'Action server unavailable: {self.trajectory_action}')

        self.get_logger().info('Waiting for joint states and wrench data...')
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            joints_ok = all(name in self.latest_joint_map for name in self.ur_joint_names)
            wrench_ok = self.latest_wrench is not None
            if joints_ok and wrench_ok:
                return
            if time.time() - t0 > 15.0:
                raise RuntimeError('Timeout waiting for /joint_states or wrench topic data')

    def _send_joint_goal_and_wait(self, target_positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.ur_joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target_positions]
        sec = int(self.move_time_sec)
        nsec = int((self.move_time_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
        goal.trajectory.points = [point]

        send_future = self.action_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Trajectory goal rejected')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _gravity_in_sensor(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.world_frame, self.sensor_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed during sample: {ex}', throttle_duration_sec=2.0)
            return None

        q = tf_msg.transform.rotation
        rot_ws = quat_to_rot(q.x, q.y, q.z, q.w)
        g_world = np.array([0.0, 0.0, -self.gravity_norm], dtype=float)
        return rot_ws.T @ g_world

    def _collect_samples_for_current_pose(self, samples):
        collected = 0
        timeout = time.time() + max(5.0, self.samples_per_pose * self.sample_period_sec * 3.0)
        while rclpy.ok() and collected < self.samples_per_pose and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=self.sample_period_sec)
            if self.latest_wrench is None:
                continue
            g_sensor = self._gravity_in_sensor()
            if g_sensor is None:
                continue

            msg = self.latest_wrench
            samples.append(
                {
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
                }
            )
            collected += 1

        if collected < self.samples_per_pose:
            raise RuntimeError(f'Insufficient samples at one pose: {collected}/{self.samples_per_pose}')

    def _generate_pose_targets(self, base):
        d = self.wrist_delta_rad
        # 基于当前位姿，只改变 wrist_1/2/3，减少碰撞风险。
        offsets = [
            (0.0, 0.0, 0.0),
            (+d, 0.0, 0.0),
            (-d, 0.0, 0.0),
            (0.0, +d, 0.0),
            (0.0, -d, 0.0),
            (0.0, 0.0, +d),
            (0.0, 0.0, -d),
            (+d, +d, 0.0),
            (+d, 0.0, +d),
            (0.0, +d, +d),
            (-d, -d, 0.0),
            (0.0, -d, -d),
        ]

        targets = []
        for o1, o2, o3 in offsets:
            p = list(base)
            p[3] = p[3] + o1
            p[4] = p[4] + o2
            p[5] = p[5] + o3
            targets.append(p)
        return targets

    def run(self):
        self._wait_until_ready()

        base = [self.latest_joint_map[name] for name in self.ur_joint_names]
        targets = self._generate_pose_targets(base)

        self.get_logger().info(f'Start auto calibration over {len(targets)} poses...')
        samples = []

        for i, tgt in enumerate(targets, start=1):
            self.get_logger().info(f'Move to pose {i}/{len(targets)}')
            self._send_joint_goal_and_wait(tgt)
            self._spin_wait(self.settle_time_sec)
            self._collect_samples_for_current_pose(samples)
            self.get_logger().info(f'Collected total samples: {len(samples)}')

        result = solve_gravity_params(samples)
        data = {
            'estimated_at': datetime.now().isoformat(),
            'mode': 'sim_auto',
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

        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(
            'Sim auto calibration done. '
            f'mass={data["mass"]:.6f}, '
            f'com=[{data["com"][0]:.4f}, {data["com"][1]:.4f}, {data["com"][2]:.4f}], '
            f'rms={data["rms_residual"]:.6f}, '
            f'file={self.output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimAutoGravityCalibrationNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
