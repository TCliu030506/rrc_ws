#!/usr/bin/env python3

import rclpy
from .cybergear_client import CybergearClient

MENU = """
请选择操作:
 1) 串口唤醒 (wake)
 2) 电机使能 (enable)
 3) 电机失能 (disable)
 4) 目标位置运动 (move_pos)
 5) 设置当前位置为零位 (set_current_zero)
 6) 回零位 (return_to_zero)
 7) 读取电机状态 (read_state)
 8) 力矩控制 (move_torque_control)
 9) 退出 (exit)
输入编号并回车: """

def main():
    rclpy.init()
    motor_id = input("请输入电机ID (整数): ")
    try:
        motor_id = int(motor_id)
    except ValueError:
        print("无效的ID，使用默认ID 1")
        motor_id = 1

    client = CybergearClient(motor_id)

    while rclpy.ok():
        choice = input(MENU).strip()
        if choice == '1':
            client.wake()
        elif choice == '2':
            client.enable()
        elif choice == '3':
            client.disable()
        elif choice == '4':
            speed = float(input("输入速度 (float): "))
            pos = float(input("输入位置 (float): "))
            # 若需要字节打包，可在client内部处理，或传入bytes列表
            client.move_pos(speed, pos)
        elif choice == '5':
            client.set_current_zero()
        elif choice == '6':
            client.return_to_zero()
        elif choice == '7':
            client.read_state()
            # 等待一下，让订阅回调打印状态
            rclpy.spin_once(client, timeout_sec=0.1)
        elif choice == '8':
            torque = float(input("输入力矩 (float): "))
            position = float(input("输入位置 (float): "))
            speed = float(input("输入速度 (float): "))
            kp = float(input("输入Kp (float): "))
            kd = float(input("输入Kd (float): "))
            client.move_torque_control(torque, position, speed, kp, kd)
        elif choice == '9' or choice.lower() == 'exit':
            print("退出程序")
            break
        else:
            print("无效的选择，请重试。")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
