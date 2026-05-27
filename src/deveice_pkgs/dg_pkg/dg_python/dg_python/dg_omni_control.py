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
from omni_msgs.msg import OmniButtonEvent
import threading
import time
from rclpy.executors import MultiThreadedExecutor

class DGButtonController(Node):
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
        self.dir = 1
        self.lead = 2.0        # 丝杠导程 (mm/rev)
        # self.lo = 42.3         # 初始长度 (mm) 43.004--138deg

        # 设置限位范围（单位：mm），根据实际需要调整
        self.min_lin = 13.0  # 最小丝杠长度
        self.max_lin = 60.0  # 最大丝杠长度


        print("请输入速度大小 speed (dps)，直接回车使用默认值")
        self.default_speed_dps = 1000.0  # 默认速度(单位 dps)，你可改
        while True:
            s = input(f"speed_dps = (default {self.default_speed_dps}) ").strip()
            if s == "":
                self.speed_dps = self.default_speed_dps
                break
            try:
                self.speed_dps = float(s)
                break
            except ValueError:
                print("输入无效，请输入数字或直接回车。")

        # 安全限幅（可选）
        self.speed_dps = max(0.0, min(self.speed_dps, 3000.0))


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
        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        # ROS 发布者：输出当前状态
        # self.timer = self.create_timer(0.1, self.publish_state)
        self.state_pub = self.create_publisher(DgState, '/dg/state', 10)
        self.publish_rate = 100.0  # Hz
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()  # 启动独立线程持续发布　初始化时启动

        self.get_logger().info("DG Controller node initialized.")


    def button_callback(self, msg: OmniButtonEvent):
        """
        仅根据按钮消息控制电机：
        - grey_button
        - white_button
        - 都未按下 / 同时按下：停止
        """
        grey_pressed = msg.grey_button
        white_pressed = msg.white_button
        self.get_logger().info(f"Button grey {grey_pressed} Button white {white_pressed}")

        if self.current_lin > self.min_lin and self.current_lin < self.max_lin:
            if grey_pressed == 0 and  white_pressed == 1:
                cmd = int(round((+self.speed_dps * self.dir) * 100.0))  # 0.01 dps
                self.get_logger().info(f"Button grey -> +speed {self.speed_dps:.2f} dps (cmd={cmd})")
                self.motor.move_speed(cmd)

            elif white_pressed == 0 and grey_pressed == 1:
                cmd = int(round((-self.speed_dps * self.dir) * 100.0))  # 0.01 dps
                self.get_logger().info(f"Button white -> -speed {self.speed_dps:.2f} dps (cmd={cmd})")
                self.motor.move_speed(cmd)
            else:
                self.get_logger().info("Buttons released/conflict -> stop")
                self.motor.stop()

        elif self.current_lin < self.min_lin and grey_pressed == 0 and  white_pressed == 1:
            cmd = int(round((+self.speed_dps * self.dir) * 100.0))  # 0.01 dps
            self.get_logger().info(f"Button grey -> +speed {self.speed_dps:.2f} dps (cmd={cmd})")
            self.motor.move_speed(cmd)

        elif self.current_lin > self.max_lin and white_pressed == 0 and grey_pressed == 1:
            cmd = int(round((-self.speed_dps * self.dir) * 100.0))  # 0.01 dps
            self.get_logger().info(f"Button white -> -speed {self.speed_dps:.2f} dps (cmd={cmd})")
            self.motor.move_speed(cmd)
        else:
            self.get_logger().info(f"Limit reached (l={self.current_lin:.2f} mm), stopping motor.")
            self.motor.stop()

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


def main(args=None):
    rclpy.init(args=args)
    node = DGButtonController()

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
