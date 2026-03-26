#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import sys
import os
import math

# 导入UR5控制接口
from ur5_rtde_control.ur5_rtde_control import URCONTROL

PI = math.pi

WAYPOINTS = [
    # 零姿态基准
    [0.25, -0.35, 0.50, 0.0, 0.0, 0.0],

    # 单一轴旋转：roll 各种角度
    [0.26, -0.34, 0.51,  PI/6,  0.0, 0.0],
    [0.24, -0.36, 0.49,  PI/4,  0.0, 0.0],
    [0.27, -0.33, 0.52,  PI/3,  0.0, 0.0],

    # 单一轴旋转：pitch 各种角度
    [0.25, -0.35, 0.50, 0.0,  PI/6,  0.0],
    [0.25, -0.35, 0.50, 0.0,  PI/4,  0.0],
    [0.25, -0.35, 0.50, 0.0,  PI/3,  0.0],

    # 单一轴旋转：yaw 各种角度
    [0.25, -0.35, 0.50, 0.0, 0.0,  PI/6],
    [0.25, -0.35, 0.50, 0.0, 0.0,  PI/4],
    [0.25, -0.35, 0.50, 0.0, 0.0,  PI/3],
    [0.25, -0.35, 0.50, 0.0, 0.0,  PI/2],
    [0.25, -0.35, 0.50, 0.0, 0.0,  PI],
    [0.25, -0.35, 0.50, 0.0, 0.0, -PI/6],
    [0.25, -0.35, 0.50, 0.0, 0.0, -PI/4],
    [0.25, -0.35, 0.50, 0.0, 0.0, 0.0],

]

class AutoGravityCalibrationNode(Node):
    def wait_until_stable(self, threshold=0.001, duration=1.0, check_period=0.05):
        """
        等待机械臂末端速度低于阈值持续一段时间，判定为静止。
        threshold: 速度阈值(m/s)
        duration: 需持续静止的时间(s)
        check_period: 检查周期(s)
        """
        stable_time = 0.0
        while True:
            tcp_speed = self.ur.rtde_r.getActualTCPSpeed()  # 返回6维速度
            linear_speed = (tcp_speed[0]**2 + tcp_speed[1]**2 + tcp_speed[2]**2) ** 0.5
            if linear_speed < threshold:
                stable_time += check_period
                if stable_time >= duration:
                    break
            else:
                stable_time = 0.0
            time.sleep(check_period)
            
    def __init__(self):
        super().__init__('auto_gravity_calibration_node')
        self.ur = URCONTROL('192.168.1.102')  # 修改为你的UR5 IP
        self.collect_srv = self.create_client(Trigger, 'collect_one_sample')
        self.stop_srv = self.create_client(Trigger, 'stop_collection')
        self.solve_srv = self.create_client(Trigger, 'solve_and_save')
        self.get_logger().info('自动重力标定节点已启动')

    def wait_for_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'等待服务 {client.srv_name} 可用...')

    def call_service(self, client):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run(self):
        self.wait_for_service(self.collect_srv)
        self.wait_for_service(self.stop_srv)
        self.wait_for_service(self.solve_srv)

        # 先清空历史数据
        clear_srv = self.create_client(Trigger, 'clear_samples')
        self.wait_for_service(clear_srv)
        self.get_logger().info('清空历史采样...')
        self.call_service(clear_srv)

        for idx, pose in enumerate(WAYPOINTS):
            self.get_logger().info(f'移动到第{idx+1}个位姿: {pose}')
            self.ur.movel(pose, 0.05, 0.1)  # 慢速移动到位
            self.wait_until_stable()
            self.get_logger().info('采集当前位姿数据...')
            self.call_service(self.collect_srv)
            time.sleep(0.5)

        self.get_logger().info('采集结束，停止采集...')
        self.call_service(self.stop_srv)
        self.get_logger().info('开始求解并保存标定结果...')
        self.call_service(self.solve_srv)
        self.get_logger().info('自动重力标定流程完成！')


def main(args=None):
    rclpy.init(args=args)
    node = AutoGravityCalibrationNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
