#!/usr/bin/env python3
"""
简化版 UR5 IK 求解节点

功能：
1. 订阅 /admittance/cmd_pose 话题接收位姿
2. 调用 MoveIt IK 服务进行逆运动学求解
3. 将求解结果发布到 /ur5_target_joints 话题
"""

from typing import List, Optional, Dict

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Ur5IkNode(Node):
    def __init__(self) -> None:
        super().__init__('ur5_ik_node')
        
        # UR5 关节名称（固定顺序）
        self._joint_names: List[str] = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        # 缓存最新关节状态
        self._latest_joint_state: Optional[Dict[str, float]] = None
        
        # 声明参数
        self.declare_parameter('input_pose_topic', '/admittance/cmd_pose')
        self.declare_parameter('output_joints_topic', '/ur5_target_joints')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('ik_link_name', 'tool0')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('ik_timeout_sec', 1.0)
        
        # 获取参数值
        input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        output_joints_topic = str(self.get_parameter('output_joints_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        ik_service = str(self.get_parameter('ik_service').value)
        self._group_name = str(self.get_parameter('group_name').value)
        self._ik_link_name = str(self.get_parameter('ik_link_name').value)
        self._planning_frame = str(self.get_parameter('planning_frame').value)
        self._ik_timeout_sec = float(self.get_parameter('ik_timeout_sec').value)
        
        # 创建 IK 服务客户端
        self._ik_client = self.create_client(GetPositionIK, ik_service)
        
        # 创建发布者（发布求解后的关节角度）
        self._joints_publisher = self.create_publisher(
            Float64MultiArray,
            output_joints_topic,
            10
        )
        
        # 创建订阅者（订阅导纳输出的位姿）
        self._pose_subscriber = self.create_subscription(
            Pose,
            input_pose_topic,
            self._pose_callback,
            10
        )
        
        # 创建关节状态订阅者（关键：获取初始状态）
        self._joint_state_subscriber = self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            10
        )
        
        # 等待 IK 服务
        self.get_logger().info(f'Waiting for IK service: {ik_service}')
        if self._ik_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().info('IK service ready.')
        else:
            self.get_logger().error('IK service not available!')
        
        self.get_logger().info(f'UR5 IK node started: {input_pose_topic} -> {output_joints_topic}')
    
    def _joint_state_callback(self, msg: JointState) -> None:
        """缓存最新关节状态"""
        self._latest_joint_state = {
            name: pos for name, pos in zip(msg.name, msg.position)
        }
        # self.get_logger().debug(f'Updated joint state: {list(self._latest_joint_state.keys())}')
    
    def _pose_callback(self, msg: Pose) -> None:
        """接收到位姿消息后的回调"""
        # 检查关节状态是否可用
        if self._latest_joint_state is None:
            self.get_logger().warn('No joint state available yet, skipping IK')
            return
        
        self._solve_ik(msg)
    
    def _solve_ik(self, pose: Pose) -> None:
        """执行逆运动学求解"""
        # 构建 IK 请求
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group_name
        req.ik_request.ik_link_name = self._ik_link_name
        req.ik_request.avoid_collisions = False
        
        # 设置超时时间
        req.ik_request.timeout.sec = int(self._ik_timeout_sec)
        req.ik_request.timeout.nanosec = int((self._ik_timeout_sec - int(self._ik_timeout_sec)) * 1e9)
        
        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        req.ik_request.pose_stamped = pose_stamped

        # 添加初始关节状态（关键！提高 IK 求解成功率）
        if self._latest_joint_state:
            ur_joint_names = []
            ur_joint_positions = []
            for name in self._joint_names:
                if name in self._latest_joint_state:
                    ur_joint_names.append(name)
                    ur_joint_positions.append(self._latest_joint_state[name])
            
            req.ik_request.robot_state.joint_state.name = ur_joint_names
            req.ik_request.robot_state.joint_state.position = ur_joint_positions
            # self.get_logger().info(f'Using initial joint state for IK')

        # 发送异步请求
        future = self._ik_client.call_async(req)
        future.add_done_callback(self._ik_response_callback)
        
    def _ik_response_callback(self, future) -> None:
        """处理 IK 服务响应"""
        try:
            resp = future.result()
        except Exception as ex:
            self.get_logger().warn(f'IK service call failed: {ex}')
            return
        
        # 检查求解结果
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'IK failed, error_code={resp.error_code.val}')
            return
        
        # 从响应中提取关节角度
        solution_map = {
            name: pos
            for name, pos in zip(
                resp.solution.joint_state.name,
                resp.solution.joint_state.position,
            )
        }
        
        # 按固定顺序提取 UR5 关节角度
        joint_angles = []
        for joint_name in self._joint_names:
            if joint_name in solution_map:
                joint_angles.append(solution_map[joint_name])
            else:
                self.get_logger().warn(f'Missing joint in IK solution: {joint_name}')
                return
        
        # 发布关节角度
        msg = Float64MultiArray()
        msg.data = joint_angles
        self._joints_publisher.publish(msg)
        
        self.get_logger().debug(f'IK solved: {[round(a, 3) for a in joint_angles]}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur5IkNode()
    
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()