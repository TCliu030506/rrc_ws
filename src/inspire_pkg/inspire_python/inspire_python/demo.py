#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from . import inspire_client

def main(args=None):
    rclpy.init(args=args)

    motor = inspire_client.Client(3)
    
    while rclpy.ok():
        user_input = input("Enter a value: [500 ~ 1500]deg ")  # 读取用户输入
        motor.move_vel(user_input,100)

    rclpy.shutdown()

if __name__ == '__main__':
    
    main()