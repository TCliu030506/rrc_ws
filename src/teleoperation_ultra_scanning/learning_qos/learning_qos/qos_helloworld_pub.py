#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 QoS示例-发布“Hello World”话题
"""

import json
import statistics
import time

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # ROS2 QoS类

"""
创建一个发布者节点
"""
class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        
        qos_profile = QoSProfile(                                 # 创建一个QoS原则
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(String, "chatter", qos_profile)   # 创建发布者对象（消息类型、话题名、QoS原则）
        self.ack_sub = self.create_subscription(
            String, "chatter_ack", self.ack_callback, qos_profile
        )

        self.declare_parameter('period_sec', 0.05)
        self.declare_parameter('stats_window', 200)
        self._period_sec = float(self.get_parameter('period_sec').value)
        self._stats_window = int(self.get_parameter('stats_window').value)

        self._seq = 0
        self._pending_send_ns = {}
        self._rtt_samples_ms = []

        self.timer = self.create_timer(self._period_sec, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        
    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        msg = String()                                            # 创建一个String类型的消息对象
        send_ns = time.time_ns()
        payload = {
            'seq': self._seq,
            'send_ns': send_ns,
            'msg': 'Hello World',
        }
        msg.data = json.dumps(payload, separators=(',', ':'))

        self._pending_send_ns[self._seq] = send_ns
        self._seq += 1

        # 防止在订阅端掉线时等待字典无限增长
        if len(self._pending_send_ns) > (self._stats_window * 5):
            oldest_seq = min(self._pending_send_ns.keys())
            self._pending_send_ns.pop(oldest_seq, None)

        self.pub.publish(msg)                                     # 发布话题消息

    def _percentile(self, values, p):
        if not values:
            return 0.0
        data = sorted(values)
        k = max(0, min(len(data) - 1, int(round((len(data) - 1) * p))))
        return data[k]

    def ack_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            seq = int(payload['seq'])
            recv_ack_ns = time.time_ns()
        except Exception:
            self.get_logger().warn('收到无法解析的ACK消息，已忽略。')
            return

        send_ns = self._pending_send_ns.pop(seq, None)
        if send_ns is None:
            return

        rtt_ms = (recv_ack_ns - send_ns) / 1e6
        self._rtt_samples_ms.append(rtt_ms)
        if len(self._rtt_samples_ms) > self._stats_window:
            self._rtt_samples_ms.pop(0)

        avg = statistics.fmean(self._rtt_samples_ms)
        p95 = self._percentile(self._rtt_samples_ms, 0.95)
        p99 = self._percentile(self._rtt_samples_ms, 0.99)
        self.get_logger().info(
            f'RTT seq={seq}, rtt={rtt_ms:.3f}ms, avg={avg:.3f}ms, p95={p95:.3f}ms, p99={p99:.3f}ms, n={len(self._rtt_samples_ms)}'
        )
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("qos_helloworld_pub")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
