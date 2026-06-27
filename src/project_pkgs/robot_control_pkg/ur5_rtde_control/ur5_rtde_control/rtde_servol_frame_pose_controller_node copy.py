#!/usr/bin/env python3
"""
带 TF 坐标转换的 UR servoL 位姿控制节点。

节点接收 controlled_frame 的目标位姿，先转换成 base_frame 下 tool_frame
的目标位姿，再通过 RTDE servoL 发送给机械臂。这样可以省掉外部
asm_ee_command_transform_node 的中间话题传递。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

import rtde_control
import rtde_receive

from ur5_rtde_control.rtde_servol_frame_pose_math import (
    Transform,
    compute_base_to_tool_command_from_frame,
    finite_transform,
    normalize_quat,
)
from ur5_rtde_control.rtde_servol_pose_controller_node import (
    URServoLPoseControllerNode,
)


def pose_msg_to_transform(msg: Pose) -> Transform:
    """将 Pose 消息转换成内部 Transform，并归一化四元数。"""
    return (
        (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        ),
        normalize_quat((
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )),
    )


def tf_msg_to_transform(tf_msg) -> Transform:
    """将 TF 查询结果转换为内部 Transform。"""
    translation = tf_msg.transform.translation
    rotation = tf_msg.transform.rotation
    return (
        (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        ),
        normalize_quat((
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )),
    )


def transform_to_pose_msg(transform: Transform) -> Pose:
    """将转换后的 base->tool0 目标位姿发布为 Pose，便于调试观察。"""
    position, orientation = transform
    msg = Pose()
    msg.position.x = position[0]
    msg.position.y = position[1]
    msg.position.z = position[2]
    msg.orientation.x = orientation[0]
    msg.orientation.y = orientation[1]
    msg.orientation.z = orientation[2]
    msg.orientation.w = orientation[3]
    return msg


class URServoLFramePoseControllerNode(Node):
    def __init__(self):
        super().__init__('rtde_servol_frame_pose_controller_node')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('topic_cmd_pose', '/admittance/asm_ee_cmd_pose')
        self.declare_parameter('input_pose_is_stamped', False)
        self.declare_parameter('input_pose_frame', 'base')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('controlled_frame', 'asm_ee_site')
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('enable_debug_pose_publish', True)
        self.declare_parameter('debug_pose_topic', '/arm_desired_pose_tool0')
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
        self.input_pose_is_stamped = (
            self.get_parameter('input_pose_is_stamped').get_parameter_value().bool_value
        )
        self.input_pose_frame = self.get_parameter('input_pose_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
        self.controlled_frame = (
            self.get_parameter('controlled_frame').get_parameter_value().string_value
        )
        self.tf_lookup_timeout_sec = (
            self.get_parameter('tf_lookup_timeout_sec').get_parameter_value().double_value
        )
        self.enable_debug_pose_publish = (
            self.get_parameter('enable_debug_pose_publish').get_parameter_value().bool_value
        )
        self.debug_pose_topic = (
            self.get_parameter('debug_pose_topic').get_parameter_value().string_value
        )
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        self.lookahead_time = self.get_parameter('lookahead_time').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.tcp_offset = [float(v) for v in self.get_parameter('tcp_offset').value]
        self.enable_servo_verify = (
            self.get_parameter('enable_servo_verify').get_parameter_value().bool_value
        )
        self.verify_log_interval_sec = (
            self.get_parameter('verify_log_interval_sec').get_parameter_value().double_value
        )
        self.verify_topic = self.get_parameter('verify_topic').get_parameter_value().string_value
        self.verify_pos_err_warn_m = (
            self.get_parameter('verify_pos_err_warn_m').get_parameter_value().double_value
        )
        self.verify_rot_err_warn_rad = (
            self.get_parameter('verify_rot_err_warn_rad').get_parameter_value().double_value
        )
        self.dt = 1.0 / 500.0

        if len(self.tcp_offset) != 6:
            raise ValueError('tcp_offset must be length 6')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        msg_type = PoseStamped if self.input_pose_is_stamped else Pose
        self.pose_sub = self.create_subscription(
            msg_type,
            self.topic_cmd_pose,
            self.pose_callback,
            10,
        )
        if self.enable_debug_pose_publish:
            self.debug_pose_pub = self.create_publisher(Pose, self.debug_pose_topic, 10)
        else:
            self.debug_pose_pub = None

        self.get_logger().info(
            '订阅位姿指令话题: '
            f'{self.topic_cmd_pose}, stamped={self.input_pose_is_stamped}, '
            f'input_pose_frame={self.input_pose_frame}, '
            f'controlled_frame={self.controlled_frame}, tool_frame={self.tool_frame}, '
            f'debug_pose_publish={self.enable_debug_pose_publish}, '
            f'debug_pose_topic={self.debug_pose_topic}, '
            f'servoL speed={self.speed}, acc={self.acceleration}, dt={self.dt}, '
            f'lookahead_time={self.lookahead_time}, gain={self.gain}'
        )

        if self.enable_servo_verify:
            self._verify_pub = self.create_publisher(Float64MultiArray, self.verify_topic, 10)
            self.get_logger().info(
                f'servoL执行验证已开启，发布话题: {self.verify_topic}, '
                f'interval={self.verify_log_interval_sec}s'
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

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Transform:
        tf_msg = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
        )
        return tf_msg_to_transform(tf_msg)

    def pose_callback(self, msg):
        # Pose 没有 header，使用 input_pose_frame 参数；PoseStamped 优先使用 header.frame_id。
        if self.input_pose_is_stamped:
            command_frame = msg.header.frame_id or self.input_pose_frame
            pose_msg = msg.pose
        else:
            command_frame = self.input_pose_frame
            pose_msg = msg

        command_to_controlled = pose_msg_to_transform(pose_msg)
        if not finite_transform(command_to_controlled):
            self.get_logger().error(
                '收到非有限位姿指令，已丢弃，防止 NaN/Inf 进入 servoL。'
            )
            return

        try:
            base_to_tool = compute_base_to_tool_command_from_frame(
                command_to_controlled=command_to_controlled,
                command_frame=command_frame,
                base_frame=self.base_frame,
                controlled_frame=self.controlled_frame,
                tool_frame=self.tool_frame,
                lookup_transform=self._lookup_transform,
            )
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(
                f'位姿坐标转换失败，已跳过本帧: {exc}',
                throttle_duration_sec=1.0,
            )
            return

        position, orientation = base_to_tool
        if self.debug_pose_pub is not None:
            self.debug_pose_pub.publish(transform_to_pose_msg(base_to_tool))

        rx, ry, rz = URServoLPoseControllerNode.quaternion_to_rotvec(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        )
        target_pose = [
            float(position[0]),
            float(position[1]),
            float(position[2]),
            float(rx),
            float(ry),
            float(rz),
        ]
        if not all(math.isfinite(value) for value in target_pose):
            self.get_logger().error(
                '转换后的 servoL 目标位姿包含 NaN/Inf，已丢弃。'
            )
            return

        with self._lock:
            self._target_pose = target_pose

    def _servo_send_loop(self):
        period = self.dt
        while self._running:
            with self._lock:
                target_pose = None if self._target_pose is None else list(self._target_pose)

            if target_pose is not None:
                if not all(math.isfinite(value) for value in target_pose):
                    self.get_logger().error(
                        'servoL 发送线程检测到非法目标位姿，已丢弃。'
                    )
                    time.sleep(period)
                    continue

                if not self._first_frame_aligned:
                    try:
                        current_tcp_pose = self.rtde_r.getActualTCPPose()
                        if current_tcp_pose is not None and len(current_tcp_pose) == 6:
                            target_pose = [float(v) for v in current_tcp_pose]
                            self._first_frame_aligned = True
                            self.get_logger().info(
                                '首帧已对齐当前TCP位姿，下一周期开始跟踪外部目标位姿'
                            )
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
                        self._publish_verify(servo_ret, elapsed_ms, target_pose)
                except Exception as exc:
                    self.get_logger().error(f'servoL下发失败: {exc}')

            time.sleep(period)

    def _publish_verify(self, servo_ret, elapsed_ms: float, target_pose):
        now = time.time()
        if now - self._last_verify_log_time < self.verify_log_interval_sec:
            return
        self._last_verify_log_time = now

        pos_err = float('nan')
        rot_err = float('nan')
        try:
            actual_pose = self.rtde_r.getActualTCPPose()
            if actual_pose is not None and len(actual_pose) == 6:
                pos_err, rot_err = URServoLPoseControllerNode.compute_pose_errors(
                    actual_pose,
                    target_pose,
                )
        except Exception as exc:
            self.get_logger().warn(f'servoL验证读取当前TCP失败: {exc}')

        verify_msg = Float64MultiArray()
        verify_msg.data = [
            float(self._servo_cmd_count),
            0.0 if servo_ret is False else 1.0,
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
    node = URServoLFramePoseControllerNode()
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
