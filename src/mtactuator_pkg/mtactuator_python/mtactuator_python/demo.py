#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from . import mtactuator_client

def main(args=None):
    rclpy.init(args=args)

    motor = mtactuator_client.Client(1)                        # id = 1
    
    # while rclpy.ok():
        # user_input = input("Enter a value: [500 ~ 1500]deg ")  # 读取用户输入
    # motor.move_angle_add(100, 1000)                 # maxspeed & pos
    # while rclpy.ok():
    # motor.move_speed(-1000)                 # speed
        
    motor.close()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()