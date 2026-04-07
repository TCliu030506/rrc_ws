#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Vector3
from omni_msgs.msg import OmniFeedback

class TeleoperationFeedbackNode(Node):

    def __init__(self):
        super().__init__('teleoperation_feedback_node')
        # 参数可根据需要修改
        # self.declare_parameter('wrench_topic', '/external_force_torque_wrench_compensated')
        self.declare_parameter('wrench_topic', '/external_force_torque_wrench')
        self.declare_parameter('feedback_topic', '/phantom/force_feedback')
        self.declare_parameter('position', [0.0, 0.0, 0.0])  # 默认力反馈位置为0
        self.declare_parameter('force_scale', 1.0)  # 力缩放系数
        self.declare_parameter('publish_frequency', 1000.0)  # 发布频率，单位Hz

        # 力反馈预处理参数（去噪/抑振）
        self.declare_parameter('enable_force_filter', True)
        self.declare_parameter('deadband_force', 0.25)      # N，死区阈值
        self.declare_parameter('lowpass_cutoff_hz', 25.0)   # Hz，一阶低通截止频率
        self.declare_parameter('max_force_abs', 8.0)        # N，输出限幅
        self.declare_parameter('max_force_rate', 120.0)     # N/s，限斜率

        self.wrench_topic = self.get_parameter('wrench_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.position = self.get_parameter('position').value
        self.force_scale = self.get_parameter('force_scale').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        self.enable_force_filter = bool(self.get_parameter('enable_force_filter').value)
        self.deadband_force = float(self.get_parameter('deadband_force').value)
        self.lowpass_cutoff_hz = float(self.get_parameter('lowpass_cutoff_hz').value)
        self.max_force_abs = float(self.get_parameter('max_force_abs').value)
        self.max_force_rate = float(self.get_parameter('max_force_rate').value)

        if self.publish_frequency <= 0.0:
            self.publish_frequency = 100.0
            self.get_logger().warn('publish_frequency <= 0, fallback to 100.0 Hz')

        self._dt = 1.0 / self.publish_frequency
        self._alpha = self._compute_lpf_alpha(self.lowpass_cutoff_hz, self._dt)

        self._filt_force = [0.0, 0.0, 0.0]  # 一阶低通状态
        self._prev_out_force = [0.0, 0.0, 0.0]  # 限斜率状态

        self.subscription = self.create_subscription(
            WrenchStamped,
            self.wrench_topic,
            self.wrench_callback,
            10
        )
        self.publisher = self.create_publisher(OmniFeedback, self.feedback_topic, 10)
        self.get_logger().info(f'Subscribing to: {self.wrench_topic}, publishing to: {self.feedback_topic}')
        self.get_logger().info(
            f'Force preprocess: enable={self.enable_force_filter}, '
            f'deadband={self.deadband_force:.3f}N, fc={self.lowpass_cutoff_hz:.1f}Hz, '
            f'max_abs={self.max_force_abs:.2f}N, max_rate={self.max_force_rate:.1f}N/s'
        )

        self.latest_wrench = None
        timer_period = 1.0 / self.publish_frequency if self.publish_frequency > 0 else 0.01
        self.timer = self.create_timer(timer_period, self.publish_feedback)

    def _compute_lpf_alpha(self, cutoff_hz: float, dt: float) -> float:
        if cutoff_hz <= 0.0 or dt <= 0.0:
            return 1.0
        wdt = 2.0 * math.pi * cutoff_hz * dt
        return wdt / (1.0 + wdt)

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def _apply_force_preprocess(self, fx: float, fy: float, fz: float):
        """按轴执行：死区 -> 低通 -> 限幅 -> 限斜率。"""
        raw = [fx, fy, fz]

        # 1) 死区阈值
        dead = []
        th = max(0.0, self.deadband_force)
        for v in raw:
            dead.append(0.0 if abs(v) < th else v)

        # 2) 一阶低通
        lpf = [0.0, 0.0, 0.0]
        a = self._alpha
        for i in range(3):
            self._filt_force[i] = self._filt_force[i] + a * (dead[i] - self._filt_force[i])
            lpf[i] = self._filt_force[i]

        # 3) 限幅
        if self.max_force_abs > 0.0:
            for i in range(3):
                lpf[i] = self._clamp(lpf[i], -self.max_force_abs, self.max_force_abs)

        # 4) 限斜率
        out = [0.0, 0.0, 0.0]
        if self.max_force_rate > 0.0 and self._dt > 0.0:
            max_step = self.max_force_rate * self._dt
            for i in range(3):
                dv = lpf[i] - self._prev_out_force[i]
                dv = self._clamp(dv, -max_step, max_step)
                out[i] = self._prev_out_force[i] + dv
                self._prev_out_force[i] = out[i]
        else:
            out = lpf
            self._prev_out_force = list(out)

        return out[0], out[1], out[2]

    def wrench_callback(self, msg: WrenchStamped):
        # 只存储最新的wrench信息
        self.latest_wrench = msg

    def publish_feedback(self):
        if self.latest_wrench is None:
            return
        msg = self.latest_wrench
        feedback = OmniFeedback()
        feedback.force = Vector3()
        # 力缩放
        fx = msg.wrench.force.x * self.force_scale *-1.0  # 注意坐标系差异，反向力反馈
        fy = msg.wrench.force.y * self.force_scale *-1.0  # 注意坐标系差异，反向力反馈
        fz = msg.wrench.force.z * self.force_scale *-1.0  # 注意坐标系差异，反向力反馈

        if self.enable_force_filter:
            fx, fy, fz = self._apply_force_preprocess(fx, fy, fz)

        feedback.force.x = fx
        feedback.force.y = fy
        feedback.force.z = fz
        feedback.position = Vector3()
        feedback.position.x = self.position[0]
        feedback.position.y = self.position[1]
        feedback.position.z = self.position[2]
        self.publisher.publish(feedback)
        # 高频节点避免每帧打印日志，以减少抖动和CPU开销




def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationFeedbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
