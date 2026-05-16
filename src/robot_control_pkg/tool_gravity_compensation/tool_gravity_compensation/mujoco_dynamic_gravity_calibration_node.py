#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from tool_gravity_compensation.gravity_model import quat_to_rot
from rclpy.qos import qos_profile_sensor_data


class MujocoDynamicGravityCalibrationNode(Node):
    def __init__(self):
        super().__init__('mujoco_dynamic_gravity_calibration_node')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_command_topic', '/command_joint')
        self.declare_parameter('wrench_topic', '/sensors/wrench')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('sensor_frame', 'asm_force_sensor_link')
        self.declare_parameter('gravity_norm', 9.81)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('samples_per_pose', 20)
        self.declare_parameter('move_time_sec', 1.2)
        self.declare_parameter('stable_velocity_threshold', 0.02)
        self.declare_parameter('stable_duration_sec', 0.3)
        self.declare_parameter('stable_timeout_sec', 20.0)
        self.declare_parameter('wrist_delta_rad', 0.35)
        self.declare_parameter('wrist_pose_samples', 5)
        self.declare_parameter(
            'link_frames',
            ['asm_tool_base_link', 'asm_tool_link1', 'asm_tool_link2'],
        )
        self.declare_parameter('tool_joint_names', ['asm_tool_joint1', 'asm_tool_joint2'])
        self.declare_parameter(
            'output_file',
            '/home/liutiancheng/Lab_WS/rrc_ws/src/robot_control_pkg/tool_gravity_compensation/config/tool_gravity_calibration_mujoco_dynamic.json',
        )

        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.joint_command_topic = str(self.get_parameter('joint_command_topic').value)
        self.wrench_topic = str(self.get_parameter('wrench_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.gravity_norm = float(self.get_parameter('gravity_norm').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.samples_per_pose = int(self.get_parameter('samples_per_pose').value)
        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.stable_velocity_threshold = float(self.get_parameter('stable_velocity_threshold').value)
        self.stable_duration_sec = float(self.get_parameter('stable_duration_sec').value)
        self.stable_timeout_sec = float(self.get_parameter('stable_timeout_sec').value)
        self.wrist_delta_rad = float(self.get_parameter('wrist_delta_rad').value)
        self.wrist_pose_samples = max(3, int(self.get_parameter('wrist_pose_samples').value))
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
        self.latest_joint_velocity_map = {}
        self._last_joint_state_time = None
        self._last_joint_position_map = {}
        self.latest_wrench = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_cb, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, self.wrench_topic, self._wrench_cb, 10)
        self.joint_cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 50)

        self.get_logger().info(
            'MuJoCo dynamic calibration node ready. '
            f'joint_cmd={self.joint_command_topic}, wrench={self.wrench_topic}, '
            f'link_count={len(self.link_frames)}, out={self.output_file}'
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
        current_time = self.get_clock().now().nanoseconds * 1e-9

        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_map[name] = float(pos)

        if len(msg.velocity) >= len(msg.name):
            for name, vel in zip(msg.name, msg.velocity):
                self.latest_joint_velocity_map[name] = float(vel)
        else:
            if self._last_joint_state_time is not None:
                dt = current_time - self._last_joint_state_time
                if dt > 1e-6:
                    for name, pos in zip(msg.name, msg.position):
                        prev_pos = self._last_joint_position_map.get(name)
                        if prev_pos is None:
                            continue
                        self.latest_joint_velocity_map[name] = float((float(pos) - prev_pos) / dt)

        self._last_joint_state_time = current_time
        self._last_joint_position_map = {name: float(pos) for name, pos in zip(msg.name, msg.position)}

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.latest_wrench = msg

    def _spin_wait(self, seconds: float) -> None:
        end_t = time.time() + max(0.0, seconds)
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_until_ready(self) -> None:
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
            if time.time() - t0 > 20.0:
                raise RuntimeError('Timeout waiting for /joint_states or wrench topic data')

    # def _publish_joint_target_for_duration(self, ur_positions):
    #     names = list(self.ur_joint_names)
    #     positions = [float(v) for v in ur_positions]
    #     end_t = time.time() + max(0.0, self.move_time_sec)
    #     while rclpy.ok() and time.time() < end_t:
    #         msg = JointState()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.name = names
    #         msg.position = positions
    #         self.joint_cmd_pub.publish(msg)

    #         # 打印发布的指令进行调试
    #         formatted_positions = '[' + ', '.join([f'{v:.4f}' for v in positions]) + ']'
    #         self.get_logger().info(f'Publishing joint target: {formatted_positions}')

    #         rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_joint_target_for_duration(self, ur_positions):
        # 只发布一次即可
        names = list(self.ur_joint_names)
        positions = [float(v) for v in ur_positions]
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.joint_cmd_pub.publish(msg)
        
        self.get_logger().info(f'Published joint target once: {positions}')
        
        # 等待一小段时间让消息传递
        time.sleep(0.5)

    def _arm_is_stable(self) -> bool:
        for name in self.ur_joint_names:
            velocity = self.latest_joint_velocity_map.get(name)
            if velocity is None or abs(velocity) > self.stable_velocity_threshold:
                return False
        return True

    def _wait_until_stable(self) -> None:
        stable_since = None
        deadline = time.time() + max(0.0, self.stable_timeout_sec)

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._arm_is_stable():
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= self.stable_duration_sec:
                    return
            else:
                stable_since = None

        raise RuntimeError(
            'Timed out waiting for the arm to become stable '
            f'(threshold={self.stable_velocity_threshold}, duration={self.stable_duration_sec}s)'
        )

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

    def _generate_pose_targets(self, base_ur):
        d_wrist = self.wrist_delta_rad

        wrist_samples = np.linspace(-d_wrist, d_wrist, self.wrist_pose_samples)

        wrist_offsets = [(0.0, 0.0, 0.0)]
        for axis in range(3):
            for value in wrist_samples:
                if abs(float(value)) < 1e-12:
                    continue
                offset = [0.0, 0.0, 0.0]
                offset[axis] = float(value)
                wrist_offsets.append(tuple(offset))

        targets = []
        for w_offset in wrist_offsets:
            ur_pos = list(base_ur)
            ur_pos[3] += w_offset[0]
            ur_pos[4] += w_offset[1]
            ur_pos[5] += w_offset[2]    
            targets.append(ur_pos)
        # # 分行打印targets以便查看（四位小数）
        # for i, target in enumerate(targets, start=1):
        #     formatted_target = '[' + ', '.join([f'{v:.4f}' for v in target]) + ']'
        #     self.get_logger().info(f'Generated target {i}/{len(targets)}: {formatted_target}')
        return targets

    def _solve_joint_mass_com_bias(self, samples):
        link_count = len(self.link_frames)
        
        # 改进：直接估计质心 com_j，而不是 c_j = m_j × com_j
        # 未知量：[m_0, m_1, m_2, com_0x, com_0y, com_0z, com_1x, com_1y, com_1z, com_2x, com_2y, com_2z]
        unknown_count = link_count + 3 * link_count
        a = np.zeros((6 * len(samples), unknown_count), dtype=float)
        y = np.zeros((6 * len(samples),), dtype=float)

        for i, sample in enumerate(samples):
            g_sensor = sample['g_sensor']
            skew_g = self._skew(g_sensor)
            row_force = 6 * i
            row_torque = row_force + 3

            # 力方程：F_meas = Σ(m_j × g_sensor)
            y[row_force:row_force + 3] = sample['force']

            # 力矩方程：τ_meas = Σ[r_j × (m_j × g_sensor) + (R_j × (m_j × com_j)) × g_sensor]
            y[row_torque:row_torque + 3] = sample['torque']

            for j, tf_item in enumerate(sample['transforms']):
                rot = tf_item['rot']
                trans = tf_item['trans']

                mass_col = j
                # 力方程中的质量项
                a[row_force:row_force + 3, mass_col] = g_sensor
                
                # 力矩方程中的质量项：r_j × (m_j × g_sensor)
                a[row_torque:row_torque + 3, mass_col] = self._skew(trans) @ g_sensor

                # 力矩方程中的质心项：(R_j × (m_j × com_j)) × g_sensor = m_j × ((R_j × com_j) × g_sensor)
                com_start = link_count + 3 * j
                a[row_torque:row_torque + 3, com_start:com_start + 3] = -skew_g @ rot

        # 添加更强的 Tikhonov 正则化
        lambda_com = 200.0    # 增大质心正则化权重
        lambda_mass = 1.0     # 质量正则化权重
        
        reg_weights = np.ones(unknown_count)
        for j in range(link_count):
            reg_weights[j] = lambda_mass
            for k in range(3):
                reg_weights[link_count + 3*j + k] = lambda_com
        
        a_reg = np.vstack([a, np.diag(reg_weights)])
        y_reg = np.hstack([y, np.zeros(unknown_count)])

        x, residuals, rank, _ = np.linalg.lstsq(a_reg, y_reg, rcond=None)

        masses = x[:link_count]
        coms = x[link_count:].reshape(link_count, 3)

        # 确保质量为正
        masses = np.maximum(masses, 1e-6)

        # 限制质心范围（防止异常值）
        coms = np.clip(coms, -0.05, 0.05)

        rms = 0.0
        if residuals.size > 0:
            rms = float(np.sqrt(residuals[0] / (6 * len(samples))))

        return {
            'force_bias': np.zeros(3),
            'torque_bias': np.zeros(3),
            'masses': masses,
            'coms': coms,
            'rank': int(rank),
            'rms_residual': rms,
        }

    def run(self):
        self._wait_until_ready()

        base_ur = [self.latest_joint_map[name] for name in self.ur_joint_names]
        targets = self._generate_pose_targets(base_ur)

        self.get_logger().info(f'Start MuJoCo dynamic auto calibration over {len(targets)} poses...')
        samples = []

        for i, ur_positions in enumerate(targets, start=1):
            self.get_logger().info(f'Move to pose {i}/{len(targets)}')
            self._publish_joint_target_for_duration(ur_positions)
            time.sleep(2) #等待机器人开始运动后再判定是否到达目标位置
            self._wait_until_stable()
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

        data = {
            'estimated_at': datetime.now().isoformat(),
            'mode': 'mujoco_dynamic_auto',
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
        }

        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(
            'MuJoCo dynamic calibration done. '
            f'rms={data["rms_residual"]:.6f}, '
            f'masses={[round(v, 6) for v in data["link_masses"]]}, '
            f'file={self.output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MujocoDynamicGravityCalibrationNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()