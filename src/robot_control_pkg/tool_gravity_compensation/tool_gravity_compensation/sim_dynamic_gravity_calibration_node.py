#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import quat_to_rot


class SimDynamicGravityCalibrationNode(Node):
    def __init__(self):
        super().__init__('sim_dynamic_gravity_calibration_node')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wrench_topic', '/wrench')
        self.declare_parameter('trajectory_action', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('samples_per_pose', 20)
        self.declare_parameter('move_time_sec', 1.2)
        self.declare_parameter('settle_time_sec', 0.8)
        self.declare_parameter('wrist_delta_rad', 0.35)
        self.declare_parameter('tool_joint_delta_rad', 0.5)
        self.declare_parameter('wrist_pose_samples', 5)
        self.declare_parameter('tool_pose_samples', 5)
        self.declare_parameter(
            'link_frames',
            ['asm_force_sensor_link', 'asm_tool_base_link', 'asm_tool_link1', 'asm_tool_link2'],
        )
        self.declare_parameter('tool_joint_names', ['asm_tool_joint1', 'asm_tool_joint2'])
        self.declare_parameter(
            'output_file',
            '/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_sim_dynamic.json',
        )

        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.wrench_topic = str(self.get_parameter('wrench_topic').value)
        self.trajectory_action = str(self.get_parameter('trajectory_action').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.samples_per_pose = int(self.get_parameter('samples_per_pose').value)
        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.wrist_delta_rad = float(self.get_parameter('wrist_delta_rad').value)
        self.tool_joint_delta_rad = float(self.get_parameter('tool_joint_delta_rad').value)
        self.wrist_pose_samples = max(3, int(self.get_parameter('wrist_pose_samples').value))
        self.tool_pose_samples = max(3, int(self.get_parameter('tool_pose_samples').value))
        self.link_frames = [str(v) for v in self.get_parameter('link_frames').value]
        self.tool_joint_names = [str(v) for v in self.get_parameter('tool_joint_names').value]
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
        self.create_subscription(WrenchStamped, self.wrench_topic, self._wrench_cb, 100)

        # Separate action clients for UR and ASM
        self.ur_action_client = ActionClient(self, FollowJointTrajectory, self.trajectory_action)
        self.asm_action_client = ActionClient(
            self, FollowJointTrajectory, '/asm_arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info(
            'Dynamic calibration node ready. '
            f'link_count={len(self.link_frames)}, out={self.output_file}, '
            f'tool_joints={self.tool_joint_names}'
        )

    @staticmethod
    def _skew(vec):
        x, y, z = vec
        return np.array([
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ], dtype=float)

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_map[name] = float(pos)

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.latest_wrench = msg

    def _spin_wait(self, seconds: float) -> None:
        end_t = time.time() + max(0.0, seconds)
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_until_ready(self) -> None:
        self.get_logger().info('Waiting for UR trajectory action server...')
        if not self.ur_action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'UR action server unavailable: {self.trajectory_action}')
        
        self.get_logger().info('Waiting for ASM trajectory action server...')
        if not self.asm_action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('ASM action server unavailable: /asm_arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for joint states and wrench data...')
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            joints_ok = all(name in self.latest_joint_map for name in self.ur_joint_names)
            wrench_ok = self.latest_wrench is not None
            if joints_ok and wrench_ok:
                missing_tool = [name for name in self.tool_joint_names if name not in self.latest_joint_map]
                if missing_tool:
                    self.get_logger().warn(
                        f'Tool joints not found in /joint_states: {missing_tool}. '
                        'Calibration still runs via TF snapshots.'
                    )
                return
            if time.time() - t0 > 15.0:
                raise RuntimeError('Timeout waiting for /joint_states or wrench topic data')

    def _send_joint_goal_and_wait_with_tool(self, ur_positions, tool_positions):
        # Send UR goals
        ur_goal = FollowJointTrajectory.Goal()
        ur_goal.trajectory.joint_names = list(self.ur_joint_names)
        ur_point = JointTrajectoryPoint()
        ur_point.positions = [float(v) for v in ur_positions]
        sec = int(self.move_time_sec)
        nsec = int((self.move_time_sec - sec) * 1e9)
        ur_point.time_from_start.sec = sec
        ur_point.time_from_start.nanosec = nsec
        ur_goal.trajectory.points = [ur_point]

        # Send ASM goals
        asm_goal = FollowJointTrajectory.Goal()
        asm_goal.trajectory.joint_names = list(self.tool_joint_names)
        asm_point = JointTrajectoryPoint()
        asm_point.positions = [float(v) for v in tool_positions]
        asm_point.time_from_start.sec = sec
        asm_point.time_from_start.nanosec = nsec
        asm_goal.trajectory.points = [asm_point]

        # Send both goals asynchronously
        ur_send_future = self.ur_action_client.send_goal_async(ur_goal)
        asm_send_future = self.asm_action_client.send_goal_async(asm_goal)

        # Wait for both goals to be accepted
        while rclpy.ok() and (not ur_send_future.done() or not asm_send_future.done()):
            rclpy.spin_once(self, timeout_sec=0.05)

        ur_goal_handle = ur_send_future.result()
        asm_goal_handle = asm_send_future.result()

        if ur_goal_handle is None or not ur_goal_handle.accepted:
            raise RuntimeError('UR trajectory goal rejected')
        if asm_goal_handle is None or not asm_goal_handle.accepted:
            raise RuntimeError('ASM trajectory goal rejected')

        # Wait for both results
        ur_result_future = ur_goal_handle.get_result_async()
        asm_result_future = asm_goal_handle.get_result_async()
        while rclpy.ok() and (not ur_result_future.done() or not asm_result_future.done()):
            rclpy.spin_once(self, timeout_sec=0.05)

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

        send_future = self.ur_action_client.send_goal_async(goal)
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

    def _link_transform_snapshot(self):
        transforms = []
        for link_frame in self.link_frames:
            try:
                tf_msg = self.tf_buffer.lookup_transform(self.sensor_frame, link_frame, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f'TF lookup failed for link {link_frame}: {ex}',
                    throttle_duration_sec=2.0,
                )
                return None

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            rot = quat_to_rot(q.x, q.y, q.z, q.w)
            trans = np.array([t.x, t.y, t.z], dtype=float)
            transforms.append({
                'rot': rot,
                'trans': trans,
            })
        return transforms

    def _collect_sample(self):
        if self.latest_wrench is None:
            return None

        g_sensor = self._gravity_in_sensor()
        if g_sensor is None:
            return None

        transforms = self._link_transform_snapshot()
        if transforms is None:
            return None

        msg = self.latest_wrench
        tool_joint_positions = {
            name: float(self.latest_joint_map[name])
            for name in self.tool_joint_names
            if name in self.latest_joint_map
        }
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
            'tool_joint_positions': tool_joint_positions,
        }

    def _collect_samples_for_current_pose(self, samples):
        collected = 0
        timeout = time.time() + max(5.0, self.samples_per_pose * self.sample_period_sec * 3.0)
        while rclpy.ok() and collected < self.samples_per_pose and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=self.sample_period_sec)
            sample = self._collect_sample()
            if sample is None:
                continue
            samples.append(sample)
            collected += 1

        if collected < self.samples_per_pose:
            raise RuntimeError(f'Insufficient samples at one pose: {collected}/{self.samples_per_pose}')

    def _generate_pose_targets(self, base):
        d_wrist = self.wrist_delta_rad
        d_tool = self.tool_joint_delta_rad

        wrist_samples = np.linspace(-d_wrist, d_wrist, self.wrist_pose_samples)
        tool_samples = np.linspace(-d_tool, d_tool, self.tool_pose_samples)

        # Wrist pose offsets (for UR joints). Use multiple intermediate values per axis.
        wrist_offsets = [(0.0, 0.0, 0.0)]
        for axis in range(3):
            for value in wrist_samples:
                if abs(float(value)) < 1e-12:
                    continue
                offset = [0.0, 0.0, 0.0]
                offset[axis] = float(value)
                wrist_offsets.append(tuple(offset))

        # Tool joint offsets (for asm_tool_joint1, asm_tool_joint2). Use multiple intermediate values per joint.
        tool_offsets = [(0.0, 0.0)]
        for axis in range(2):
            for value in tool_samples:
                if abs(float(value)) < 1e-12:
                    continue
                offset = [0.0, 0.0]
                offset[axis] = float(value)
                tool_offsets.append(tuple(offset))

        targets = []
        for w_offset in wrist_offsets:
            for t_offset in tool_offsets:
                p = list(base)
                # Apply wrist offsets (to UR joints)
                p[3] = p[3] + w_offset[0]
                p[4] = p[4] + w_offset[1]
                p[5] = p[5] + w_offset[2]
                targets.append((p, t_offset))
        
        return targets

    def _solve_joint_mass_com_bias(self, samples):
        link_count = len(self.link_frames)
        # x = [bf(3), bt(3), m(link_count), c(link_count*3)]
        unknown_count = 6 + link_count + 3 * link_count
        a = np.zeros((6 * len(samples), unknown_count), dtype=float)
        y = np.zeros((6 * len(samples),), dtype=float)

        for i, sample in enumerate(samples):
            g_sensor = sample['g_sensor']
            skew_g = self._skew(g_sensor)
            row_force = 6 * i
            row_torque = row_force + 3

            # Force rows: f = bf + sum_i m_i * g
            a[row_force:row_force + 3, 0:3] = np.eye(3)
            y[row_force:row_force + 3] = sample['force']

            # Torque rows: tau = bt + sum_i( m_i * [t_i]_x g - [g]_x R_i c_i )
            a[row_torque:row_torque + 3, 3:6] = np.eye(3)
            y[row_torque:row_torque + 3] = sample['torque']

            for j, tf_item in enumerate(sample['transforms']):
                rot = tf_item['rot']
                trans = tf_item['trans']

                # m_i coefficients in force equation
                mass_col = 6 + j
                a[row_force:row_force + 3, mass_col] = g_sensor

                # m_i coefficients in torque equation
                a[row_torque:row_torque + 3, mass_col] = self._skew(trans) @ g_sensor

                # c_i coefficients in torque equation
                c_start = 6 + link_count + 3 * j
                a[row_torque:row_torque + 3, c_start:c_start + 3] = -skew_g @ rot

        x, residuals, rank, _ = np.linalg.lstsq(a, y, rcond=None)

        force_bias = x[0:3]
        torque_bias = x[3:6]
        masses = x[6:6 + link_count]
        c_vec = x[6 + link_count:]
        c_mat = c_vec.reshape(link_count, 3)

        coms = np.zeros((link_count, 3), dtype=float)
        for i in range(link_count):
            if abs(masses[i]) > 1e-8:
                coms[i] = c_mat[i] / masses[i]

        rms = 0.0
        if residuals.size > 0:
            rms = float(np.sqrt(residuals[0] / (6 * len(samples))))

        return {
            'force_bias': force_bias,
            'torque_bias': torque_bias,
            'masses': masses,
            'coms': coms,
            'rank': int(rank),
            'rms_residual': rms,
        }

    def run(self):
        self._wait_until_ready()

        base = [self.latest_joint_map[name] for name in self.ur_joint_names]
        targets = self._generate_pose_targets(base)

        self.get_logger().info(f'Start dynamic auto calibration over {len(targets)} poses...')
        self.get_logger().info('Both UR and ASM tool joints will be actively controlled during calibration.')
        samples = []

        for i, (ur_positions, tool_offsets) in enumerate(targets, start=1):
            self.get_logger().info(f'Move to pose {i}/{len(targets)} (UR + ASM joints)')
            
            # Prepare tool joint targets with offsets
            base_tool_pos = [
                float(self.latest_joint_map.get(name, 0.0))
                for name in self.tool_joint_names
            ]
            adjusted_tool_pos = [
                base_tool_pos[j] + tool_offsets[j]
                for j in range(len(self.tool_joint_names))
            ]
            
            # Send both UR and ASM goals
            self._send_joint_goal_and_wait_with_tool(ur_positions, adjusted_tool_pos)
            self._spin_wait(self.settle_time_sec)
            self._collect_samples_for_current_pose(samples)
            self.get_logger().info(f'Collected total samples: {len(samples)}')

        result = self._solve_joint_mass_com_bias(samples)
        est_masses = result['masses']
        est_coms = result['coms']
        negative_idx = [i for i, m in enumerate(est_masses) if m <= 0.0]
        if negative_idx:
            self.get_logger().warn(
                f'Estimated non-positive masses at indices {negative_idx}. '
                'Please increase posture diversity or sample count for better conditioning.'
            )

        tool_joint_snapshot = {
            name: float(self.latest_joint_map[name])
            for name in self.tool_joint_names
            if name in self.latest_joint_map
        }

        data = {
            'estimated_at': datetime.now().isoformat(),
            'mode': 'sim_dynamic_auto',
            'world_frame': self.world_frame,
            'sensor_frame': self.sensor_frame,
            'gravity_norm': self.gravity_norm,
            'sample_count': int(len(samples)),
            'rank': int(result['rank']),
            'rms_residual': float(result['rms_residual']),
            'link_frames': list(self.link_frames),
            'link_masses': [float(v) for v in est_masses],
            'link_com_x': [float(v) for v in est_coms[:, 0]],
            'link_com_y': [float(v) for v in est_coms[:, 1]],
            'link_com_z': [float(v) for v in est_coms[:, 2]],
            'force_bias': [float(v) for v in result['force_bias']],
            'torque_bias': [float(v) for v in result['torque_bias']],
            'tool_joint_names': list(self.tool_joint_names),
            'tool_joint_snapshot': tool_joint_snapshot,
        }

        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(
            'Dynamic calibration done. '
            f'rms={data["rms_residual"]:.6f}, '
            f'masses={[round(v, 6) for v in data["link_masses"]]}, '
            f'file={self.output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimDynamicGravityCalibrationNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()