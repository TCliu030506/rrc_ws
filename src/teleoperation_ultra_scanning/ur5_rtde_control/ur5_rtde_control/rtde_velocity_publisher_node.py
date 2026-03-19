#!/usr/bin/env python3
"""
基于rtde_control实现的UR机器人速度控制节点。
订阅geometry_msgs/Twist类型的速度指令，并通过rtde接口发送给UR机器人。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
import rtde_control
import rtde_receive
import threading
import time

class URVelocityControlNode(Node):
    def __init__(self):
        super().__init__('ur_velocity_control_node')
        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('topic_cmd_vel', '/ur_cmd_vel')
        self.declare_parameter('adaptive_acc_min', 0.3)
        self.declare_parameter('adaptive_acc_max', 3.0)
        self.declare_parameter('adaptive_acc_scale', 1.0)
        self.declare_parameter('enable_debug_output', True)
        self.declare_parameter('topic_debug_sent_velocity', '/UR5/debug/sent_velocity')
        self.declare_parameter('topic_debug_sent_acceleration', '/UR5/debug/sent_acceleration')
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        topic_cmd_vel = self.get_parameter('topic_cmd_vel').get_parameter_value().string_value
        self.adaptive_acc_min = self.get_parameter('adaptive_acc_min').get_parameter_value().double_value
        self.adaptive_acc_max = self.get_parameter('adaptive_acc_max').get_parameter_value().double_value
        self.adaptive_acc_scale = self.get_parameter('adaptive_acc_scale').get_parameter_value().double_value
        self.enable_debug_output = self.get_parameter('enable_debug_output').get_parameter_value().bool_value
        topic_debug_sent_velocity = self.get_parameter('topic_debug_sent_velocity').get_parameter_value().string_value
        topic_debug_sent_acceleration = self.get_parameter('topic_debug_sent_acceleration').get_parameter_value().string_value

        self.get_logger().info(f'连接UR机器人: {robot_ip}')
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not self.rtde_c.isConnected() or not self.rtde_r.isConnected():
            self.get_logger().error('RTDE接口连接失败！')
            raise RuntimeError('RTDE接口连接失败')
        self.get_logger().info('RTDE接口连接成功')

        self.subscription = self.create_subscription(
            Twist,
            topic_cmd_vel,
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info(f'订阅速度指令话题: {topic_cmd_vel}')

        self.debug_sent_velocity_pub = None
        self.debug_sent_acceleration_pub = None
        if self.enable_debug_output:
            self.debug_sent_velocity_pub = self.create_publisher(Float64MultiArray, topic_debug_sent_velocity, 10)
            self.debug_sent_acceleration_pub = self.create_publisher(Float64, topic_debug_sent_acceleration, 10)
            self.get_logger().info(
                f'调试输出已开启: velocity->{topic_debug_sent_velocity}, acceleration->{topic_debug_sent_acceleration}'
            )

        # 控制参数
        self.dt = 1.0/125.0

        # 共享速度变量（由订阅回调更新，发送线程读取）
        self._lock = threading.Lock()
        self._velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._last_sent_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._running = True

        # 启动独立发送线程，以指定频率下发速度指令
        self._send_thread = threading.Thread(target=self._speed_send_loop, daemon=True)
        self._send_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        # 仅更新共享速度变量，不在回调里直接下发rtde
        with self._lock:
            self._velocity = [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ]
        self.get_logger().debug(f'接收到速度指令: {self._velocity}')

    def _speed_send_loop(self):
        period = self.dt if self.dt > 0.0 else 1.0 / 125.0
        while self._running:
            with self._lock:
                velocity = list(self._velocity)

            delta_velocity = [velocity[i] - self._last_sent_velocity[i] for i in range(6)]
            max_delta = max(abs(value) for value in delta_velocity)
            required_acc = (max_delta / period) * self.adaptive_acc_scale if period > 0.0 else self.adaptive_acc_max
            adaptive_acc = max(self.adaptive_acc_min, min(self.adaptive_acc_max, required_acc))

            try:
                self.rtde_c.speedL(velocity, adaptive_acc, self.dt)
                self._last_sent_velocity = velocity
                if self.enable_debug_output:
                    velocity_msg = Float64MultiArray()
                    velocity_msg.data = velocity
                    acceleration_msg = Float64()
                    acceleration_msg.data = adaptive_acc
                    self.debug_sent_velocity_pub.publish(velocity_msg)
                    self.debug_sent_acceleration_pub.publish(acceleration_msg)
            except Exception as e:
                self.get_logger().error(f'发送速度指令失败: {e}')
            time.sleep(period)

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_send_thread') and self._send_thread.is_alive():
            self._send_thread.join(timeout=1.0)
        self.rtde_c.stopScript()
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        self.get_logger().info('已断开UR RTDE连接')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = URVelocityControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
