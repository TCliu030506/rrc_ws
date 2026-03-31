#!/usr/bin/env python3

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

        self.wrench_topic = self.get_parameter('wrench_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.position = self.get_parameter('position').value
        self.force_scale = self.get_parameter('force_scale').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        self.subscription = self.create_subscription(
            WrenchStamped,
            self.wrench_topic,
            self.wrench_callback,
            10
        )
        self.publisher = self.create_publisher(OmniFeedback, self.feedback_topic, 10)
        self.get_logger().info(f'Subscribing to: {self.wrench_topic}, publishing to: {self.feedback_topic}')

        self.latest_wrench = None
        timer_period = 1.0 / self.publish_frequency if self.publish_frequency > 0 else 0.01
        self.timer = self.create_timer(timer_period, self.publish_feedback)

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
        feedback.force.x = msg.wrench.force.x * self.force_scale
        feedback.force.y = msg.wrench.force.y * self.force_scale
        feedback.force.z = msg.wrench.force.z * self.force_scale
        feedback.position = Vector3()
        feedback.position.x = self.position[0]
        feedback.position.y = self.position[1]
        feedback.position.z = self.position[2]
        self.publisher.publish(feedback)
        self.get_logger().info(f'Published feedback: {feedback}')




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
