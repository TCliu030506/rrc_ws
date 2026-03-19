#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

import numpy as np
from mtactuator_python import mtactuator_client
from mtactuator_msg.msg import MtPosition
from dg_python.dg_kinematic import DGKinematic
from dg_msg.msg import DgState
import threading
import time
from rclpy.executors import MultiThreadedExecutor

class DGController(Node):
    """
    ROS2 node for controlling DG actuator
    """

    def __init__(self):
        super().__init__('dg_controller')

        print("请输入初始丝杠长度 lo (mm)")
        while True:
            try:
                self.lo = float(input("lo = "))
                break
            except ValueError:
                print("输入无效，请输入数字。")

        # 初始化电机对象（ID=1）
        self.motor = mtactuator_client.Client(1)

        # 初始化DG运动学
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        txt_path = os.path.join(script_dir, "gamma_theta_polyfit.txt")
        self.kin = DGKinematic(txt_path=txt_path)

        # 机械参数设置（根据实际情况修改）
        self.lead = 2.0        # 丝杠导程 (mm/rev)
        # self.lo = 42.3         # 初始长度 (mm) 43.004--138deg
        self.dir = 1           # 电机旋转方向 (+1 or -1)
        self.max_speed = 4000    # 电机最大速度
        self.theta_sw = 100.0  # 模式切换角 (deg)

        # 状态变量
        self.current_motor_angle = 0.0   # 当前角度 (deg)
        self.current_lin = self.lo       # 当前丝杠长度 l (mm)
        self.last_motor_angle = None     # 电机上一次角度 (deg)

        # ROS 订阅者：获取电机位置
        self.pos_sub = self.create_subscription(
            MtPosition,
            '/mt/position',
            self.motor_pos_callback,
            10
        )

        # ROS 发布者：输出当前状态
        # self.timer = self.create_timer(0.1, self.publish_state)
        self.state_pub = self.create_publisher(DgState, '/dg/state', 10)
        self.publish_rate = 100.0  # Hz
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()  # 启动独立线程持续发布　初始化时启动

        self.get_logger().info("DG Controller node initialized.")

    def motor_pos_callback(self, msg: MtPosition):
        """
        每当接收到电机位置反馈时：
        1. 计算丝杠长度 l；
        2. 根据 DGKinematic 计算 n, m, γ；
        3. 发布 DgState。
        """
        self.current_motor_angle = msg.position  # 电机角度（多圈，单位：deg）

        if self.last_motor_angle is None:
            self.last_motor_angle = self.current_motor_angle
            return

        delta_angle = (self.current_motor_angle - self.last_motor_angle) * self.dir
        delta_l = (delta_angle / 360.0) * self.lead
        self.current_lin = self.lo + delta_l

        # self.get_logger().debug(
        #     f"State | l={self.current_lin:.2f} mm, θ={theta_deg:.2f}°, γ={gamma_deg:.2f}°, n={n:.2f}, m={m:.2f}"
        # )

    def publish_loop(self):
        """持续发布当前状态（独立线程）"""
        period = 1.0 / self.publish_rate
        while rclpy.ok():
            try:
                self.publish_state()
            except Exception as e:
                self.get_logger().error(f"Publish thread error: {e}")
            time.sleep(period)

    def publish_state(self):
        """周期性计算并发布 DG 状态"""
        # 若电机角度还没更新过，则跳过
        if self.last_motor_angle is None:
            return
        
        # === 正运动学求解 ===
        theta_deg = self.kin.theta_from_lin(self.current_lin)
        gamma_deg = self.kin.gamma_from_theta(theta_deg)
        n, m = self.kin._nm_from_theta(theta_deg)

        # === 构造并发布消息 ===
        msg_state = DgState()
        msg_state.header = Header()
        msg_state.header.stamp = self.get_clock().now().to_msg()
        msg_state.n = n
        msg_state.m = m
        msg_state.gamma = gamma_deg
        msg_state.lin = self.current_lin

        self.state_pub.publish(msg_state)

        # self.get_logger().debug(   # 改成 debug 可防止日志刷屏
        #     f"[Timer] l={self.current_lin:.2f} mm, θ={theta_deg:.2f}°, γ={gamma_deg:.2f}°, n={n:.2f}, m={m:.2f}"
        # )

    # 主函数：处理输入并控制电机
    def control_loop(self):
        while rclpy.ok():
            mode = input("\nSelect control mode (n(16~56) / m(22.1~105) / gamma(85~270) / theta(30~138) / lin(43.271~116.190) / close): ").strip().lower()

            if mode == 'close':
                print("Closing motor...")
                try:
                    # 调用电机关闭命令（模式 5）
                    self.motor.close()
                except AttributeError:
                    self.get_logger().warning("Motor close function not implemented in mtactuator_client.")
                print("Motor closed. Exiting control loop.")
                break

            if mode not in ['n', 'm', 'gamma', 'lin', 'theta']:
                print("Invalid input, please enter n / m / gamma / lin / theta.")
                continue

            try:
                target_val = float(input(f"Enter target {mode} value: "))
            except ValueError:
                print("Invalid number.")
                continue

            # ---- 根据输入模式求 lin ----
            if mode == 'n':
                lin = self.kin.inverse_from_n(target_val)
            elif mode == 'm':
                lin = self.kin.inverse_from_m(target_val)
            elif mode == 'theta':
                lin = self.kin.lin_from_theta(target_val)
            elif mode == 'lin':
                lin = target_val
            else:
                lin = self.kin.inverse_from_gamma(target_val)

            print(f"Target linear position lin = {lin:.3f} mm")

            # ---- 控制电机运动 ----
            self.move_to_lin(lin)

    # 控制电机函数
    def move_to_lin(self, lin: float):
        """
        将目标线性位移转换为电机目标角度并发送指令。
        假设电机驱动中 move_angle_add(maxspeed, pos) 的单位是“角度”。
        """
        # 计算目标位移相对初始点
        delta_l = lin - self.current_lin

        # 计算电机转动角度 (deg)
        turns = delta_l / self.lead     # 丝杠转多少圈
        motor_angle = self.dir * turns * 360.0  # 转换为度

        self.get_logger().info(
            f"Moving motor: Δl={delta_l:.3f} mm -> {motor_angle:.3f} deg"
        )

        # 发送运动命令
        self.motor.move_angle_add(self.max_speed, motor_angle * 100)

def main(args=None):
    rclpy.init(args=args)
    node = DGController()

    # 启动控制输入线程　独立线程
    control_thread = threading.Thread(target=node.control_loop, daemon=True)
    control_thread.start()

    # 使用多线程执行器运行ROS　主线程
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
