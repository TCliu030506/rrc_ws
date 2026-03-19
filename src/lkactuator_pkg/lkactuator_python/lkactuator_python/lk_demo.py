#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import time
from .lk_client import LK_Motor
from rclpy.node import Node
from msg_codec_cpp.srv import Script

class LK_clients(Node):
    def __init__(self, id: int) -> None:
        super().__init__('lkactuator_'+str(id))
        self.motor = LK_Motor(id)
        self.client = self.create_client(Script, 'lk_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/lk_service: waiting...')

    def request(self, request: Script.Request):
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_angle_add(self, maxspeed: int, pos_: int):
        req = self.motor.move_angle_add(maxspeed, pos_)
        self.request(req)

    def move_speed(self, maxspeed: int):
        req = self.motor.move_speed(maxspeed)
        self.request(req)

    def move_angle_absolute(self, maxspeed: int, pos: int):
        req: Script.Request = self.motor.move_angle_absolute(maxspeed, pos)
        self.request(req)

    def lock(self):
        req = self.motor.lock()
        self.request(req)

    def unlock(self):
        req = self.motor.unlock()
        self.request(req)

    def stop(self):
        req = self.motor.stop()
        self.request(req)

    def read_angle(self):
        # 读取多圈绝对角度
        req = self.motor.read_angle()
        self.request(req)

    def read_control_parameter(self, number: int):
        # 读取控制参数
        req = self.motor.read_control_parameter(number)
        self.request(req)

def main(args=None):
    rclpy.init(args=args)
    #示例

    # motor1 = LK_clients(5)
    # motor2 = LK_clients(4)
    # motor3 = LK_clients(3)
    # motor4 = LK_clients(2)
    # motor5 = LK_clients(1)
    # motor6 = LK_clients(6)
    motor1 = LK_clients(1)
    motor2 = LK_clients(2)
    motor3 = LK_clients(3)
    motor4 = LK_clients(4)
    motor5 = LK_clients(5)
    motor6 = LK_clients(6)
  
    try:
        while rclpy.ok():
            try:
                mode = input("请输入命令: ")

                if mode == "move":
                    user_input: int = int(input())  # 读取用户输入
                    user_input2 = int(input())  # 转换为所需单位
                    my_motor = LK_clients(user_input)  # 选择要操作的电机
                    my_motor.move_angle_absolute(360,user_input2)
                    # for i in range(100):
                    #     motor1.move_angle_absolute(360,user_input)
                    #     user_input += 1000
                    #     time.sleep(0.01)
                    # motor1.move_angle_absolute(360,user_input)
                    # # # motor1.move_angle_absolute(3600,0)
                    # # # motor1.move_angle_absolute(3600,user_input)
                    # # # motor1.move_angle_absolute(3600,0)
                    # # # motor1.move_angle_absolute(3600,user_input)
                    # # # motor1.move_angle_absolute(3600,0)
                    # motor2.move_angle_absolute(360,user_input)
                    # motor3.move_angle_absolute(360,user_input)
                    # motor4.move_angle_absolute(360,user_input)
                    # motor5.move_angle_absolute(360,user_input)
                    # motor6.move_angle_absolute(360,user_input)
                    # motor1.read_control_parameter(user_input)
                    # motor1.lock()
                    # motor1.move_speed(user_input)
                    # motor1.unlock()
                    # motor2.unlock()
                    # motor3.unlock()
                    # motor4.unlock()
                    # motor5.unlock()
                    # motor6.unlock()
                elif mode == "moveall":    
                    user_input: int = int(input())  # 读取用户输入
                    motor1.move_angle_absolute(360,user_input)
                    motor2.move_angle_absolute(360,user_input)
                    motor3.move_angle_absolute(360,user_input)
                    motor4.move_angle_absolute(360,user_input)
                    motor5.move_angle_absolute(360,user_input)
                    motor6.move_angle_absolute(360,user_input)

                elif mode == "lock":
                    motor1.lock()
                    motor2.lock()
                    motor3.lock()
                    motor4.lock()
                    motor5.lock()
                    motor6.lock()
            
                    # maxspeed & pos
                elif mode == "read":
                    user_input: int = int(input())  # 读取用户输入
                    my_motor = LK_clients(user_input)  # 选择要操作的电机
                    my_motor.read_angle()
                    # motor1.read_angle()
                    # motor2.read_angle()
                    # motor3.read_angle()
                    # motor4.read_angle()
                    # motor5.read_angle()
                    # motor6.read_angle()

                # elif mode == "":
                    # motor1.stop()
                    # motor2.stop()
                    # motor3.stop()
                    # motor4.stop()
                    # motor5.stop()
                    # motor6.stop()
            
            except KeyboardInterrupt:
                print("\n接收到中断信号")
                break
            except Exception as e:
                print(f"命令执行出错: {e}")
                # 继续循环，不退出
                continue
    #pos单位：1000=1度 speed单位：度/10s 建议360 即10秒一圈
    finally:
        my_motor.stop()
        # motor1.destroy_node()
        # motor2.destroy_node()
        # motor3.destroy_node()
        # motor4.destroy_node()
        # motor5.destroy_node()
        # motor6.destroy_node()
        rclpy.shutdown()

       

if __name__ == '__main__':
    
    main()