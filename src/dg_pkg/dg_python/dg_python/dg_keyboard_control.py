#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import sys
import select
import termios
import tty

import numpy as np
from mtactuator_python import mtactuator_client
from mtactuator_msg.msg import MtPosition
from dg_python.dg_kinematic import DGKinematic
from dg_msg.msg import DgState
import threading
import time
from rclpy.executors import MultiThreadedExecutor

class DGKeyController(Node):
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
        print("请输入速度大小 speed (dps)，直接回车使用默认值")
        self.default_speed_dps = 100.0  # 默认速度(单位 dps)，你可改
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
        self.speed_dps = max(0.0, min(self.speed_dps, 4000.0))


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
    # def control_loop(self):
    #     """
    #     键盘速度控制：
    #     W : 正转（+speed）
    #     S : 反转（-speed）
    #     Space/X : 停止（0）
    #     +/- : 增/减速度大小
    #     C : close 并退出
    #     H : 帮助
    #     """
    #     help_text = (
    #         "\nKeyboard speed control:\n"
    #         f"  W       : +speed ({self.speed_dps:.2f} dps)\n"
    #         f"  S       : -speed ({self.speed_dps:.2f} dps)\n"
    #         "  Space/X : stop\n"
    #         "  + / -   : increase/decrease speed magnitude\n"
    #         "  H       : help\n"
    #         "  C       : close motor and exit\n"
    #         "Note: speed command unit sent to driver is 0.01 dps\n"
    #     )
    #     print(help_text)

    #     fd = sys.stdin.fileno()
    #     old_settings = termios.tcgetattr(fd)
    #     tty.setcbreak(fd)

    #     current_cmd = None  # 记录上一条速度命令（0.01dps 的 int），避免重复刷服务

    #     def send_speed(dps: float):
    #         nonlocal current_cmd
    #         # dps -> 0.01 dps
    #         cmd = int(round(dps * 100.0))
    #         if cmd == current_cmd:
    #             return
    #         current_cmd = cmd
    #         self.get_logger().info(f"Set speed: {dps:.2f} dps (cmd={cmd} *0.01dps)")
    #         self.motor.move_speed(cmd)

    #     try:
    #         # 启动时默认先停住（可选）
    #         # send_speed(0.0)

    #         while rclpy.ok():
    #             rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    #             if not rlist:
    #                 continue

    #             ch = sys.stdin.read(1)
    #             if not ch:
    #                 continue
    #             key = ch.lower()

    #             if key == 'h':
    #                 print(help_text)
    #                 continue

    #             if key == 'c':
    #                 print("Closing motor...")
    #                 try:
    #                     self.motor.close()
    #                 except AttributeError:
    #                     self.get_logger().warning("Motor close function not implemented in mtactuator_client.")
    #                 print("Motor closed. Exiting control loop.")
    #                 break

    #             if key == 'w':
    #                 # dir 决定正方向
    #                 send_speed(+self.speed_dps * self.dir)
    #             elif key == 's':
    #                 send_speed(-self.speed_dps * self.dir)
    #             elif key == ' ' or key == 'x':
    #                 send_speed(0.0)
    #             elif key == '+':
    #                 self.speed_dps = min(self.speed_dps * 1.25, 100.0)
    #                 print(f"speed_dps = {self.speed_dps:.2f}")
    #                 # 不自动下发，等你按 w/s 再下发（如果你希望立即更新当前方向速度，我也可以给）
    #             elif key == '-':
    #                 self.speed_dps = max(self.speed_dps / 1.25, 0.1)
    #                 print(f"speed_dps = {self.speed_dps:.2f}")
    #             else:
    #                 continue

    #     finally:
    #         # 退出前停住（更安全）
    #         try:
    #             self.motor.move_speed(0)
    #         except Exception:
    #             pass
    #         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


    def control_loop(self):
        """
        终端遥控手感（无 key-up 的替代方案）：
        - W：正转（按下/重复触发；若 timeout 秒内无输入 -> 自动停）
        - S：反转
        - Space/X：立刻停
        - + / -：调速
        - H：帮助
        - C：关闭电机并退出
        """
        help_text = (
            "\nRC-like speed control (state-driven, auto-stop timeout):\n"
            f"  W       : +speed ({self.speed_dps:.2f} dps)\n"
            f"  S       : -speed ({self.speed_dps:.2f} dps)\n"
            "  Space/X : stop immediately\n"
            "  + / -   : increase/decrease speed magnitude\n"
            "  H       : help\n"
            "  C       : close motor and exit\n"
        )
        print(help_text)

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        # === 遥控关键参数 ===
        auto_stop_timeout = 0.5      # 秒：越小越像“松手即停”
        poll_period = 0.01           # 秒：仅用于轮询/超时检查，不用于周期发指令

        last_key_time = time.time()

        # 用“命令整数”(0.01dps)做状态机：更稳、不受 float 误差影响
        desired_cmd = 0              # 期望命令（0.01dps）
        last_sent_cmd = None         # 上次已下发命令（0.01dps），None 表示尚未下发过

        def dps_to_cmd(dps: float) -> int:
            return int(round(dps * 100.0))  # dps -> 0.01dps

        def send_cmd_if_needed():
            """只在 desired_cmd 变化时下发。0 用 stop()。"""
            nonlocal last_sent_cmd
            if desired_cmd == last_sent_cmd:
                return

            last_sent_cmd = desired_cmd

            if desired_cmd == 0:
                self.get_logger().info("Stop motor (cmd=0)")
                self.motor.stop()
            else:
                dps = desired_cmd / 100.0
                self.get_logger().info(f"Set speed: {dps:.2f} dps (cmd={desired_cmd})")
                self.motor.move_speed(desired_cmd)

        try:
            # 启动先停（并同步状态）
            self.motor.stop()
            desired_cmd = 0
            last_sent_cmd = 0

            while rclpy.ok():
                now = time.time()

                # 1) 非阻塞读键盘（poll_period 只是为了让循环醒来检查 timeout）
                rlist, _, _ = select.select([sys.stdin], [], [], poll_period)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch:
                        key = ch.lower()
                        last_key_time = now

                        if key == 'h':
                            print(help_text)
                            continue

                        if key == 'c':
                            print("Closing motor...")
                            try:
                                self.motor.close()
                            except AttributeError:
                                self.get_logger().warning("Motor close function not implemented.")
                            print("Motor closed. Exiting control loop.")
                            break

                        if key == 'w':
                            desired_cmd = dps_to_cmd(+self.speed_dps * self.dir)
                        elif key == 's':
                            desired_cmd = dps_to_cmd(-self.speed_dps * self.dir)
                        elif key == 'x' or key == ' ':
                            desired_cmd = 0  # 立刻停：交给 send_cmd_if_needed() 发一次 stop
                        elif key == '+':
                            self.speed_dps = min(self.speed_dps * 1.25, 4000.0)
                            print(f"speed_dps = {self.speed_dps:.2f}")
                            # 若当前在动（desired_cmd != 0），按原方向更新速度
                            if desired_cmd > 0:
                                desired_cmd = dps_to_cmd(+self.speed_dps * self.dir)
                            elif desired_cmd < 0:
                                desired_cmd = dps_to_cmd(-self.speed_dps * self.dir)
                        elif key == '-':
                            self.speed_dps = max(self.speed_dps / 1.25, 0.1)
                            print(f"speed_dps = {self.speed_dps:.2f}")
                            if desired_cmd > 0:
                                desired_cmd = dps_to_cmd(+self.speed_dps * self.dir)
                            elif desired_cmd < 0:
                                desired_cmd = dps_to_cmd(-self.speed_dps * self.dir)

                        # 有输入后，立刻按需下发（无周期发送）
                        send_cmd_if_needed()

                # 2) 超时自动停（模拟松手）
                # 条件：超过 timeout 且当前期望不是 0（也就是“在动/想动”）
                if (now - last_key_time) > auto_stop_timeout and desired_cmd != 0:
                    desired_cmd = 0
                    send_cmd_if_needed()


        finally:
            try:
                self.motor.stop()
            except Exception:
                pass
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)




def main(args=None):
    rclpy.init(args=args)
    node = DGKeyController()

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
