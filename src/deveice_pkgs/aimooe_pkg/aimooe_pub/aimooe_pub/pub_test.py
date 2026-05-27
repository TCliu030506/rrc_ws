# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from aimooe_sdk.msg import AimCoord
import rclpy.clock
import random
import math

# topic pub cycle
class Time:
    freq = 0.5 # seconds
    
class Publisher(Node):

    def __init__(self):
        super().__init__('Aimooe_tracker_test'+str(rclpy.clock.Clock().now().nanoseconds))
        self.declare_parameter('frame', 'fake_tool')
        self.tool_frame = self.get_parameter('frame').get_parameter_value().string_value
        self.get_logger().info(f'pub tool_frame: {self.tool_frame}')
        self.msg = AimCoord()
        self.publisher_ = self.create_publisher(AimCoord, 'aimooe_tracker', 10)
        timer_period = Time.freq  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pub_msg()

    def pub_msg(self):
        # read data
        self.msg.header.frame_id = self.tool_frame
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position.x = random.uniform(-0.1,0.1)
        self.msg.position.y = random.uniform(-0.1,0.1)
        self.msg.position.z = random.uniform(-0.1,0.1)
        self.msg.orientation.x = 0.0
        self.msg.orientation.y = 0.0
        self.msg.orientation.z = math.pi*random.uniform(0.0,2.0)
        self.msg.mean_error = random.uniform(-1.0,1.0)
        self.publisher_.publish(self.msg)
                
def main(args=None):
    rclpy.init(args=args)

    # python_interpreter_path = sys.executable
    # print(f"The Python interpreter is located at: {python_interpreter_path}")

    optical_pub = Publisher()

    rclpy.spin(optical_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optical_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()