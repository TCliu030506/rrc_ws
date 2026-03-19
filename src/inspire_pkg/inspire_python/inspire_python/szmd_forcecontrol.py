#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import rclpy.clock
from rclpy.node import Node
import time

from inspire_python.inspire_client import Subscriber
from . import inspire_client
import rclpy.parameter

from coordinate.msg import ArrayInt16
     
def main(args=None):
    rclpy.init(args=args)

    motor4 = inspire_client.Client(4)
    motor5 = inspire_client.Client(5)

    echo = Node('Inspire_actuator_echo'+str(rclpy.clock.Clock().now().nanoseconds))
    echo.declare_parameter('id', 0x04)
    id = echo.get_parameter('id').get_parameter_value().integer_value
    frame = "inspire_actuator_id" + str(id)
    echo.get_logger().info(f'Listening tool_frame: {frame}')

    sub = Subscriber(frame)
    
    while rclpy.ok():
        force = sub.msg_listen_force_only()
        print(force)  
        motor5.move_fce(force)
        time.sleep(1)

if __name__ == '__main__':
    
    main()