#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import sys
import os

# 导入UR5控制接口
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../teleoperation_ultra_scanning/ur5_rtde_control/ur5_rtde_control'))
from ur5_rtde_control import URCONTROL

# 你可以根据实际情况自定义采样位姿
WAYPOINTS = [
    [0.3, -0.2, 0.5, 0, 3.14, 0],
    [0.3, 0.2, 0.5, 0, 3.14, 0],
    [0.5, 0.0, 0.4, 0, 3.14, 0],
    [0.4, -0.3, 0.6, 0, 3.14, 0],
    [0.4, 0.3, 0.6, 0, 3.14, 0],
]

class AutoGravityCalibrationNode(Node):
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
            self.ur.movel(pose)
            time.sleep(2.5)  # 等待机械臂稳定
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
