#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math

class FlangeToSensorStaticTF(Node):
    def __init__(self):
        super().__init__('flange_to_sensor_static_tf')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'tool0_controller'           # 父坐标系
        t.child_frame_id = 'sensor_frame'      # 子坐标系
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.010      # 沿Z轴正向移动0.010m
        # 绕Z轴正向旋转60度
        theta = math.radians(60)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta/2)
        t.transform.rotation.w = math.cos(theta/2)
        self.broadcaster.sendTransform(t)
        self.get_logger().info('Static transform flange -> sensor_frame published.')

        # 保持节点运行
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 需要周期性发布，否则部分工具可能收不到
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'tool0_controller'
        t.child_frame_id = 'sensor_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.010
        theta = math.radians(60)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta/2)
        t.transform.rotation.w = math.cos(theta/2)
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FlangeToSensorStaticTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
