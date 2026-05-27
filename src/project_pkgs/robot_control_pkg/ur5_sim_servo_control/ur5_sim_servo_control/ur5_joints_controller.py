#!/usr/bin/env python3
"""
简化版 UR5 关节轨迹控制节点

功能：
1. 订阅 /ur5_target_joints 话题接收目标关节位置
2. 向 joint_trajectory_controller 发送轨迹目标
"""

from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Ur5JointController(Node):
    def __init__(self) -> None:
        super().__init__('ur5_joint_controller')
        
        # UR5 关节名称
        self._joint_names: List[str] = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        # 运动时间（秒）
        self._goal_time_sec = 0.01
        
        # 创建 Action 客户端
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 订阅目标关节位置话题
        self._subscriber = self.create_subscription(
            Float64MultiArray,
            '/ur5_target_joints',
            self._target_joints_callback,
            10
        )
        
        self.get_logger().info('UR5 Joint Controller node started.')
        self.get_logger().info('Waiting for action server...')

    def _target_joints_callback(self, msg: Float64MultiArray) -> None:
        """接收到目标关节位置后的回调函数"""
        if len(msg.data) != len(self._joint_names):
            self.get_logger().error(
                f'Invalid joint count: expected {len(self._joint_names)}, got {len(msg.data)}'
            )
            return
        
        self.get_logger().info(
            f'Received target joints: {[round(v, 3) for v in msg.data]}'
        )
        
        self._send_goal(self._joint_names, msg.data)

    def _send_goal(self, joint_names: List[str], targets: List[float]) -> None:
        """向控制器发送轨迹目标"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        # 构造单点轨迹
        from trajectory_msgs.msg import JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = list(targets)
        sec = int(self._goal_time_sec)
        nanosec = int((self._goal_time_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        goal_msg.trajectory.points = [point]
        
        # 异步发送目标
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        """目标发送结果回调"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by controller')
        else:
            self.get_logger().warn('Goal rejected by controller')

    def wait_for_server(self) -> bool:
        """等待动作服务器就绪"""
        if self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().info('Action server ready.')
            return True
        self.get_logger().error('Action server not available')
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur5JointController()
    
    if not node.wait_for_server():
        node.destroy_node()
        rclpy.shutdown()
        return
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()