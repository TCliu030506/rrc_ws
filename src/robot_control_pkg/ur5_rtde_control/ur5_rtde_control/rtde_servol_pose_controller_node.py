#!/usr/bin/env python3
"""
基于rtde_control的UR机器人位置控制节点。
订阅geometry_msgs/Pose指令，并通过servoL下发到机械臂。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import rtde_control
import rtde_receive


class URServoLPoseControllerNode(Node):
    def __init__(self):
        super().__init__('ur_servol_pose_controller_node')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('topic_cmd_pose', '/ur_cmd_pose')
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('acceleration', 0.1)
        self.declare_parameter('lookahead_time', 0.1)
        self.declare_parameter('gain', 300.0)
        self.declare_parameter('tcp_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('enable_servo_verify', False)
        self.declare_parameter('verify_log_interval_sec', 0.5)
        self.declare_parameter('verify_topic', '/ur5/servol_verify')
        self.declare_parameter('verify_pos_err_warn_m', 0.02)
        self.declare_parameter('verify_rot_err_warn_rad', 0.15)

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.topic_cmd_pose = self.get_parameter('topic_cmd_pose').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        self.lookahead_time = self.get_parameter('lookahead_time').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.tcp_offset = [float(v) for v in self.get_parameter('tcp_offset').value]
        self.enable_servo_verify = self.get_parameter('enable_servo_verify').get_parameter_value().bool_value
        self.verify_log_interval_sec = self.get_parameter('verify_log_interval_sec').get_parameter_value().double_value
        self.verify_topic = self.get_parameter('verify_topic').get_parameter_value().string_value
        self.verify_pos_err_warn_m = self.get_parameter('verify_pos_err_warn_m').get_parameter_value().double_value
        self.verify_rot_err_warn_rad = self.get_parameter('verify_rot_err_warn_rad').get_parameter_value().double_value
        self.dt = 1.0 / 125.0

        if len(self.tcp_offset) != 6:
            raise ValueError('tcp_offset must be length 6')

        self.get_logger().info(f'连接UR机器人: {self.robot_ip}')
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        if not self.rtde_c.isConnected() or not self.rtde_r.isConnected():
            self.get_logger().error('RTDE接口连接失败')
            raise RuntimeError('RTDE接口连接失败')

        try:
            self.rtde_c.setTcp(self.tcp_offset)
            self.get_logger().info(f'已设置TCP偏移: {self.tcp_offset}')
        except Exception as exc:
            self.get_logger().error(f'setTcp设置失败: {exc}')
            raise

        self.get_logger().info('RTDE接口连接成功')

        self.pose_sub = self.create_subscription(
            Pose,
            self.topic_cmd_pose,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'订阅位置指令话题: {self.topic_cmd_pose}, servoL参数: '
            f'speed={self.speed}, acc={self.acceleration}, dt={self.dt}, '
            f'lookahead_time={self.lookahead_time}, gain={self.gain}'
        )
        if self.enable_servo_verify:
            self._verify_pub = self.create_publisher(Float64MultiArray, self.verify_topic, 10)
            self.get_logger().info(
                f'servoL执行验证已开启，发布话题: {self.verify_topic}, interval={self.verify_log_interval_sec}s'
            )
        else:
            self._verify_pub = None

        self._lock = threading.Lock()
        self._target_pose = None
        self._first_frame_aligned = False
        self._servo_cmd_count = 0
        self._last_verify_log_time = 0.0
        self._running = True
        self._send_thread = threading.Thread(target=self._servo_send_loop, daemon=True)
        self._send_thread.start()

    @staticmethod
    def quaternion_to_rotvec(qx: float, qy: float, qz: float, qw: float):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12:
            return [0.0, 0.0, 0.0]

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
        s = math.sqrt(max(0.0, 1.0 - qw * qw))

        if s < 1e-9:
            return [0.0, 0.0, 0.0]

        ax = qx / s
        ay = qy / s
        az = qz / s

        return [ax * angle, ay * angle, az * angle]

    def pose_callback(self, msg: Pose):
        rx, ry, rz = self.quaternion_to_rotvec(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        target_pose = [
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
            float(rx),
            float(ry),
            float(rz),
        ]

        with self._lock:
            self._target_pose = target_pose

    def _servo_send_loop(self):
        period = self.dt
        while self._running:
            with self._lock:
                target_pose = None if self._target_pose is None else list(self._target_pose)

            if target_pose is not None:
                if not self._first_frame_aligned:
                    try:
                        current_tcp_pose = self.rtde_r.getActualTCPPose()
                        if current_tcp_pose is not None and len(current_tcp_pose) == 6:
                            target_pose = [float(v) for v in current_tcp_pose]
                            self._first_frame_aligned = True
                            self.get_logger().info('首帧已对齐当前TCP位姿，下一周期开始跟踪外部目标位姿')
                    except Exception as exc:
                        self.get_logger().warn(f'首帧TCP对齐失败，继续使用外部目标位姿: {exc}')

                try:
                    t0 = time.perf_counter()
                    servo_ret = self.rtde_c.servoL(
                        target_pose,
                        self.speed,
                        self.acceleration,
                        self.dt,
                        self.lookahead_time,
                        self.gain,
                    )
                    elapsed_ms = (time.perf_counter() - t0) * 1000.0
                    self._servo_cmd_count += 1

                    if self.enable_servo_verify:
                        now = time.time()
                        if now - self._last_verify_log_time >= self.verify_log_interval_sec:
                            self._last_verify_log_time = now
                            pos_err = float('nan')
                            rot_err = float('nan')
                            try:
                                actual_pose = self.rtde_r.getActualTCPPose()
                                if actual_pose is not None and len(actual_pose) == 6:
                                    pos_err = math.sqrt(
                                        (float(actual_pose[0]) - target_pose[0]) ** 2
                                        + (float(actual_pose[1]) - target_pose[1]) ** 2
                                        + (float(actual_pose[2]) - target_pose[2]) ** 2
                                    )
                                    rot_err = math.sqrt(
                                        (float(actual_pose[3]) - target_pose[3]) ** 2
                                        + (float(actual_pose[4]) - target_pose[4]) ** 2
                                        + (float(actual_pose[5]) - target_pose[5]) ** 2
                                    )
                            except Exception as exc:
                                self.get_logger().warn(f'servoL验证读取当前TCP失败: {exc}')

                            ret_code = 0.0 if servo_ret is False else 1.0
                            verify_msg = Float64MultiArray()
                            verify_msg.data = [
                                float(self._servo_cmd_count),
                                ret_code,
                                float(elapsed_ms),
                                float(pos_err),
                                float(rot_err),
                            ]
                            if self._verify_pub is not None:
                                self._verify_pub.publish(verify_msg)

                            if (not math.isnan(pos_err)) and pos_err > self.verify_pos_err_warn_m:
                                self.get_logger().warn(
                                    f'servoL位置误差偏大: {pos_err:.4f}m > {self.verify_pos_err_warn_m:.4f}m'
                                )
                            if (not math.isnan(rot_err)) and rot_err > self.verify_rot_err_warn_rad:
                                self.get_logger().warn(
                                    f'servoL姿态误差偏大: {rot_err:.4f}rad > {self.verify_rot_err_warn_rad:.4f}rad'
                                )
                except Exception as exc:
                    self.get_logger().error(f'servoL下发失败: {exc}')

            time.sleep(period)

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_send_thread') and self._send_thread.is_alive():
            self._send_thread.join(timeout=1.0)
        if self.rtde_c:
            try:
                self.rtde_c.stopScript()
            except Exception:
                pass
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()
        self.get_logger().info('已断开UR RTDE连接')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = URServoLPoseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
