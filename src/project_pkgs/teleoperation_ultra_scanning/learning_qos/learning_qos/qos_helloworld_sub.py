#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 QoS示例-订阅“Hello World”话题消息
"""

import json
import time

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from std_msgs.msg import String                  # ROS2标准定义的String消息
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # ROS2 QoS类

"""
创建一个订阅者节点
"""
class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        
        qos_profile = QoSProfile(                                 # 创建一个QoS原则
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, qos_profile) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、QoS原则）
        self.ack_pub = self.create_publisher(String, "chatter_ack", qos_profile)

    def listener_callback(self, msg):                               # 创建回调函数，执行收到话题消息后对数据的处理
        now_ns = time.time_ns()
        try:
            payload = json.loads(msg.data)
            seq = int(payload['seq'])
            send_ns = int(payload['send_ns'])
            text = str(payload.get('msg', ''))
        except Exception:
            self.get_logger().warn('收到非延时测试格式消息，已忽略。')
            return

        one_way_ms = (now_ns - send_ns) / 1e6
        self.get_logger().info(
            f'I heard seq={seq}, msg="{text}", one-way={one_way_ms:.3f}ms (双机需NTP/PTP同步才准确)'
        )

        ack = String()
        ack.data = json.dumps({'seq': seq}, separators=(',', ':'))
        self.ack_pub.publish(ack)
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("qos_helloworld_sub")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
