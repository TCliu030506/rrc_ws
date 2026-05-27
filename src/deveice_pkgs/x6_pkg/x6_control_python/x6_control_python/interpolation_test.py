import rclpy
import numpy as np
import ast
import time
import math
import yaml
import random
import os
import threading
from pathlib import Path
from x6_msg.msg import X6Jointstime
from x6_msg.msg import X6Position
from rclpy.node import Node
import x6_kinematics.kinematics_X6 as X6

"""
----------------------------------------
x6_service.py
----------------------------------------
version 1.1.1: x6_service.py 2026-1-23

新的改动：
- 增加了绳驱方案标0公式（与磁耦合和直驱方案相反）
- 修复了部分注释错误
----------------------------------------
version 1.1.0: x6_service.py 2026-1-5
author: Yuchen Hong
email:  12425014@zju.edu.cn

新的改动：
- 摇头轨迹插值点增加到300个以降低运动速度
- 摇头轨迹起始点修正（原来和轨迹起点不重合）
- 摇头轨迹的位置改动（40->45）；偏转角度减小（原运动轨迹中杆件容易干涉断裂）
- 摇头轨迹的实际偏转角度似乎比设定的大的多？（待解决）

----------------------------------------
"""

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # 轨迹参数
        self.center = np.array([0.0, 0.0, 40.0])  # 圆心坐标 (x, y, z)
        self.radius = 5.0  # 圆的半径，可以手动修改
        self.num_points = 100  # 插值点数
        self.target_joints = np.zeros(6)  # 目标电机角度初始化
        self.start_point = np.array([self.radius, 0.0, 40.0, 0.0, 0.0, 0.0])  # 起始点 (r, 0, 40)
        package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.param_path = Path(os.path.join(package_path, 'param', 'x6param.yaml'))
        self.angle_zero = self._read_x6pose_from_yaml()

        # 螺旋轨迹参数
        self.num_points_spiral = 300  # 总插值点数
        self.center_spiral = [0.0, 0.0, 30.0]  # 起始中心点 (z=30)
        # 半径因轨迹而异 不写在轨迹参数中
        self.z_start_spiral = 45.0  # 起始高度
        self.z_end_spiral = 39.0  # 结束高度
        self.num_circles_spiral = 3  # 圈数

        # 摇头轨迹参数
        self.num_points_shake = 300  # 总插值点数

        # 使用线程锁保护共享数据
        self.lock = threading.Lock()
        self.x6joints = np.zeros(6)  # 当前电机角度初始化
        self.frequency = 300.0  # 发布频率，单位：Hz 注意：频率不能太慢 1/frequency必须小于cost_time 不然会导致轨迹不连贯

        # 创建闭环控制发布器
        self.x6theo_pub = self.create_publisher(X6Position, 'x6theo', 10)
        self.close_move_pub = self.create_publisher(X6Jointstime, 'x6tracking', 10)
        self.X6_Kine = X6.X6_solver()
        
        self.get_logger().info("节点已启动，等待用户输入...")

    def input_read(self, x6_input: str):
        """
        从终端读取 [x,y,z,roll,pitch,yaw] 格式的数据
        返回pose (list of float)
        """
        values = x6_input.strip("[]").split(",")
        values = [float(v.strip()) for v in values]

        pose = values[0:]
        pose = np.array(pose).T
        return pose

    def check_input(self, x6_input: str):
        """
        检查输入是否符合要求：[int/float,int/float,...] 共6个元素
        返回合法的list，否则返回 None
        """
        try:
            values = ast.literal_eval(x6_input)
        except Exception:
            return None

        if not (isinstance(values, list) and len(values) == 6 and all(isinstance(v, (int, float)) for v in values)):
            return None
        
        return values

    def _send_waypoint_fixed(self, fixed_point: np.ndarray):
        """发送路径点的方法(定点模式)"""
        target_pose = fixed_point
        target_angles = self.pose2angles(target_pose)
        if target_angles is False:
            self.get_logger().error("无法计算目标角度，姿态无解")
            return
        
        msg = X6Jointstime()
        msg.mode = 1
        msg.angles = target_angles.tolist()  # 转为消息单位
        msg.cost_time = 1.0
        self.close_move_pub.publish(msg)
        # self.get_logger().info(f"发布定点目标: 角度={target_angles}, 时间={msg.cost_time:.2f}秒")
    
    def _send_waypoint_tracking(self, x, y, z, rx, ry, rz):
        """发送路径点的方法(长轨迹)"""
        target_pose = [x, y, z, rx, ry, rz]
        target_angles = self.pose2angles(target_pose)
        if target_angles is False:
            self.get_logger().error("无法计算目标角度，姿态无解")
            return
        
        msg = X6Jointstime()
        msg.mode = 2
        msg.angles = target_angles.tolist()  # 转为消息单位
        msg.cost_time = 0.08
        self.close_move_pub.publish(msg)
        time.sleep(1/self.frequency)

    def send_start_order(self):
        """发送开始运动指令"""
        msg = X6Jointstime()
        msg.mode = 0
        self.close_move_pub.publish(msg)

    def pose2angles(self, target_pose):
        pose_rp = np.array(target_pose)
        # 判断是否有解
        if not self.X6_Kine.invKine_pose_rp(pose_rp[0:3], np.deg2rad(pose_rp[3:6])):
            self.get_logger().error(f"逆运动学无解，请重新输入")
            return False
        else:
            q6: np.array = np.rad2deg(self.X6_Kine.solution) 
            self.target_joints = q6 #运动学中的关节角度
            # 发布理论轨迹
            # x6theo = X6Position()
            # x6theo.joints = self.target_joints.tolist()
            # x6theo.pose_rp = pose_rp.tolist()
            # self.x6theo_pub.publish(x6theo)
            
            target_angles = self.joints2angles(self.target_joints* 1000)  # 单位：度
            return target_angles
        
    def joints2angles(self, joints):
        """将关节角度转换为电机角度"""
        # angles = - joints + self.angle_zero #直驱和磁耦合方案/电机绝对位置增大连杆角度减小（逆时针）
        # angles = joints + self.angle_zero #绳驱方案/电机绝对位置增大连杆角度增加（顺时针）
        angles = 2*joints + self.angle_zero #绳驱方案 传动比2：1
        return angles
    
    def random_pose(self):
        # 第一部分：生成球体内的随机位置（球心(0,0,40)，半径5）
        # 使用球坐标生成均匀分布的随机点
        radius = 5
        center = np.array([0, 0, 40])
        
        # 生成随机球坐标
        phi = random.uniform(0, 2 * math.pi)  # 方位角
        theta = random.uniform(0, math.pi)    # 极角
        r = random.uniform(0, radius)         # 半径
        
        # 球坐标转直角坐标
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta)
        
        # 平移到球心位置
        position = np.array([x, y, z]) + center
        
        # 第二部分：生成随机姿态
        rx = random.uniform(-15, 15)  # 绕x轴旋转角度（度）
        ry = random.uniform(-15, 15)  # 绕y轴旋转角度（度）
        rz = random.uniform(-30, 0)   # 绕z轴旋转角度（度）
        
        # 组合成6元素数组
        pose_array = np.concatenate([position, [rx, ry, rz]])

        return pose_array
    
    def move_repeat(self):
        """
        重复发送随机生成的姿态
        """
        random_pose = self.random_pose()
        # 发布理论轨迹
        x6theo = X6Position()
        x6theo.pose_rp = random_pose.tolist()
        self.x6theo_pub.publish(x6theo)
        center_point = np.array([0.0, 0.0, 40.0, 0.0, 0.0, 0.0])
        self._send_waypoint_fixed(center_point)
        time.sleep(2)
        print(random_pose)

        for _ in range(10):
            self._send_waypoint_fixed(random_pose)
            time.sleep(5)
            self.get_logger().info("5秒记录时间")
            self._send_waypoint_fixed(center_point)
            time.sleep(1.5)

    def move_circle(self):
        """
        生成平行于XY平面的圆形轨迹
        逆时针方向，从(r, 0, 40)开始
        """
        self._send_waypoint_fixed(self.start_point)
        time.sleep(2)

        for i in range(self.num_points):
            # 计算角度 (0 到 2π)
            angle = 2 * math.pi * i / self.num_points
            
            # 计算圆上的点坐标
            x = self.center[0] + self.radius * math.cos(angle)
            y = self.center[1] + self.radius * math.sin(angle)
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            
            # 发送路径点
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            
            # # 显示进度
            # if i % 10 == 0:
            #     self.get_logger().info(f"圆形轨迹进度: {i+1}/{self.num_points}")
        #self._send_waypoint_fixed(self.start_point)
        self.get_logger().info("圆形轨迹点发送完成")
        self.send_start_order()
        
    def move_square(self):
        """
        生成平行于XY平面的正方形轨迹
        逆时针方向，从(r, 0, 40)开始
        """
        points_per_side = self.num_points // 4  
        first_half_points = points_per_side // 2  

        self._send_waypoint_fixed(self.start_point)
        time.sleep(2)

        point_index = 0
        
        # 第一段: 从(5, 0, 40)到(5, 5, 40) - 向上（前半条边）
        for i in range(first_half_points):
            t = i / first_half_points
            x = self.radius
            y = t * self.radius  # 0 → 5
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 第二条边: 从(5, 5, 40)到(-5, 5, 40) - 向左
        for i in range(points_per_side):
            t = i / points_per_side
            x = self.radius - t * (2 * self.radius)  # 5 → -5
            y = self.radius
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 第三条边: 从(-5, 5, 40)到(-5, -5, 40) - 向下
        for i in range(points_per_side):
            t = i / points_per_side
            x = -self.radius
            y = self.radius - t * (2 * self.radius)  # 5 → -5
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 第四条边: 从(-5, -5, 40)到(5, -5, 40) - 向右
        for i in range(points_per_side):
            t = i / points_per_side
            x = -self.radius + t * (2 * self.radius)  # -5 → 5
            y = -self.radius
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 最后一段: 从(5, -5, 40)到(5, 0, 40) - 向上（后半条边）
        remaining_points = self.num_points - point_index  # 13个点
        for i in range(remaining_points):
            t = i / max(remaining_points, 1)
            x = self.radius
            y = -self.radius + t * self.radius  # -5 → 0
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        #self._send_waypoint_fixed(self.start_point)
        self.get_logger().info("正方形轨迹点发送完成")
        self.send_start_order()
    
    def move_triangle(self):
        """
        生成平行于XY平面的等边三角形轨迹
        逆时针方向，起始点为三角形的第一个顶点
        """
        side_length = 10.0  # 三角形边长
        
        # 计算三角形顶点 (等边三角形)
        # 顶点1: 起始点 (5, 0, 40)
        A_x = 5.0
        A_y = 0.0
        
        # 顶点2: 基于边长计算
        # 等边三角形内角60°，顶点2相对于顶点1旋转120°
        B_x = A_x + side_length * math.cos(2 * math.pi / 3)  # cos(120°) = -0.5
        B_y = A_y + side_length * math.sin(2 * math.pi / 3)  # sin(120°) = √3/2
        
        # 顶点3: 相对于顶点2旋转120°
        C_x = B_x + side_length * math.cos(4 * math.pi / 3)  # cos(240°) = -0.5
        C_y = B_y + side_length * math.sin(4 * math.pi / 3)  # sin(240°) = -√3/2
        
        # 设置总点数并分配
        points_per_side = round(self.num_points // 3) # 每条边分配33个点
        
        # 发送起始点
        self._send_waypoint_fixed([A_x, A_y, self.center[2], 0.0, 0.0, 0.0])
        time.sleep(2)
        
        point_index = 0
        
        # 第一条边: 从A(5, 0, 40)到B
        for i in range(points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = A_x + t * (B_x - A_x)
            y = A_y + t * (B_y - A_y)
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 第二条边: 从B到C
        for i in range(points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = B_x + t * (C_x - B_x)
            y = B_y + t * (C_y - B_y)
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 第三条边: 从C回到A
        for i in range(points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = C_x + t * (A_x - C_x)
            y = C_y + t * (A_y - C_y)
            z = self.center[2]
            rx, ry, rz = 0.0, 0.0, 0.0
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
            point_index += 1
        
        # 最后回到起点（确保精确闭合）
        #self._send_waypoint_fixed([A_x, A_y, self.center[2], 0.0, 0.0, 0.0])
        self.get_logger().info(f"等边三角形轨迹点发送完成，边长={side_length}")
        self.send_start_order()

    def move_circle_spiral(self):
        """
        生成平行于XY平面的螺旋下降圆形轨迹
        逆时针方向绕三圈，从(r, 0, 45)开始，到(r, 0, 36)结束
        """
        # 发送起始点
        radius_spiral = 3.0
        self._send_waypoint_fixed([radius_spiral, 0.0, self.z_start_spiral, 0.0, 0.0, 0.0])
        time.sleep(2)
        
        for i in range(self.num_points_spiral):
            # 计算全局归一化参数 (0 到 1)
            t_global = i / (self.num_points_spiral - 1) if self.num_points_spiral > 1 else 0
            
            # 计算角度 (0 到 6π，即三圈)
            angle = 2 * math.pi * self.num_circles_spiral * t_global
            
            # 计算圆上的点坐标
            x = radius_spiral * math.cos(angle)      # 中心为(0,0)
            y = radius_spiral * math.sin(angle)
            
            # Z从45线性下降到36（使用t_global确保全局线性）
            z = self.z_start_spiral + t_global * (self.z_end_spiral - self.z_start_spiral)
            
            rx, ry, rz = 0.0, 0.0, 0.0
            
            # 发送路径点
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
        
        # 发送最终点
        # end_point = [self.start_point_spiral[0], self.start_point_spiral[1], self.z_end_spiral, 0.0, 0.0, 0.0]
        # self._send_waypoint_fixed(end_point)
        self.get_logger().info("螺旋圆形轨迹点发送完成")
        self.send_start_order()

    def move_square_spiral(self):
        """
        生成平行于XY平面的螺旋下降正方形轨迹
        从(5, 5, 45)开始，逆时针方向绕圈，终点在(5, 5, 36)
        """
        # 发送起始点
        start_x, start_y = 5.0, 5.0
        self._send_waypoint_fixed([start_x, start_y, self.z_start_spiral, 0.0, 0.0, 0.0])
        time.sleep(2)
        
        # 计算每条边的点数
        points_per_circle = self.num_points_spiral // self.num_circles_spiral     
        points_per_side = points_per_circle // 4
        
        point_index = 0
        
        # 生成螺旋正方形轨迹
        for circle in range(self.num_circles_spiral):
            # 当前圈的Z范围
            z_start_circle = self.z_start_spiral + circle * (self.z_end_spiral - self.z_start_spiral) / self.num_circles_spiral
            z_end_circle = self.z_start_spiral + (circle + 1) * (self.z_end_spiral - self.z_start_spiral) / self.num_circles_spiral
            
            # 定义正方形的四个边（从起点(5,5)开始，逆时针）
            # 正方形顶点：右上角(5,5) → 左上角(-5,5) → 左下角(-5,-5) → 右下角(5,-5) → 返回右上角(5,5)
            
            # 第一条边：从(5,5)到(-5,5) - 上边
            for i in range(points_per_side):
                t = i / (points_per_side - 1) if points_per_side > 1 else 0
                x = 5.0 - t * 10.0  # 从5到-5
                y = 5.0
                # Z在当前圈内线性上升
                z = z_start_circle + (i / points_per_circle) * (z_end_circle - z_start_circle)
                
                self._send_waypoint_tracking(x, y, z, 0.0, 0.0, 0.0)
                point_index += 1
            
            # 第二条边：从(-5,5)到(-5,-5) - 左边
            for i in range(points_per_side):
                t = i / (points_per_side - 1) if points_per_side > 1 else 0
                x = -5.0
                y = 5.0 - t * 10.0  # 从5到-5
                # 计算当前点在整圈中的进度
                progress_in_circle = (points_per_side + i) / points_per_circle
                z = z_start_circle + progress_in_circle * (z_end_circle - z_start_circle)
                
                self._send_waypoint_tracking(x, y, z, 0.0, 0.0, 0.0)
                point_index += 1
            
            # 第三条边：从(-5,-5)到(5,-5) - 下边
            for i in range(points_per_side):
                t = i / (points_per_side - 1) if points_per_side > 1 else 0
                x = -5.0 + t * 10.0  # 从-5到5
                y = -5.0
                # 计算当前点在整圈中的进度
                progress_in_circle = (2 * points_per_side + i) / points_per_circle
                z = z_start_circle + progress_in_circle * (z_end_circle - z_start_circle)
                
                self._send_waypoint_tracking(x, y, z, 0.0, 0.0, 0.0)
                point_index += 1
            
            # 第四条边：从(5,-5)到(5,5) - 右边
            for i in range(points_per_side):
                t = i / (points_per_side - 1) if points_per_side > 1 else 0
                x = 5.0
                y = -5.0 + t * 10.0  # 从-5到5
                # 计算当前点在整圈中的进度
                progress_in_circle = (3 * points_per_side + i) / points_per_circle
                z = z_start_circle + progress_in_circle * (z_end_circle - z_start_circle)
                
                self._send_waypoint_tracking(x, y, z, 0.0, 0.0, 0.0)
                point_index += 1
        
        self.get_logger().info(f"螺旋正方形轨迹点发送完成，共{point_index+1}个点")
        self.send_start_order()

    def move_triangle_spiral(self):
        """
        生成平行于XY平面的螺旋下降等边三角形轨迹
        逆时针方向绕三圈，边长为10，中心在原点
        """
        side_length = 5.0  # 三角形边长
        
        # 计算等边三角形的顶点（中心在原点）
        # 等边三角形的外接圆半径 = 边长 / √3
        radius_circumscribed = side_length / math.sqrt(3)
        
        # 计算三个顶点（逆时针方向）
        # 顶点1: 从90度开始，这样三角形的一个顶点在正上方
        angle_start = math.pi / 2  # 90度
        A_x = radius_circumscribed * math.cos(angle_start)
        A_y = radius_circumscribed * math.sin(angle_start)
        
        # 顶点2: 逆时针120度 (90+120=210度)
        angle_B = angle_start + 2 * math.pi / 3  # 逆时针120度
        B_x = radius_circumscribed * math.cos(angle_B)
        B_y = radius_circumscribed * math.sin(angle_B)
        
        # 顶点3: 再逆时针120度 (210+120=330度)
        angle_C = angle_B + 2 * math.pi / 3  # 再逆时针120度
        C_x = radius_circumscribed * math.cos(angle_C)
        C_y = radius_circumscribed * math.sin(angle_C)
        
        # # 验证中心是否在原点
        # center_x = (A_x + B_x + C_x) / 3
        # center_y = (A_y + B_y + C_y) / 3
        # print(f"三角形顶点: A({A_x:.2f}, {A_y:.2f}), B({B_x:.2f}, {B_y:.2f}), C({C_x:.2f}, {C_y:.2f})")
        # print(f"三角形中心: ({center_x:.2f}, {center_y:.2f})")
        
        # 总边数 = 3条边 × 3圈
        total_sides = 3 * self.num_circles_spiral
        # 每条边的点数
        points_per_side = int(math.ceil(self.num_points_spiral / total_sides))
        
        # 发送起始点（从顶点A开始）
        start_with_z = [A_x, A_y, self.z_start_spiral, 0.0, 0.0, 0.0]
        self._send_waypoint_fixed(start_with_z)
        time.sleep(2)
        
        # 记录当前全局点索引
        point_index = 0
        
        # 生成三圈三角形轨迹
        for _ in range(self.num_circles_spiral):
            for side in range(3):  # 3条边
                # 计算这条边实际需要的点数
                current_side_points = points_per_side
                if point_index + points_per_side > self.num_points_spiral:
                    current_side_points = self.num_points_spiral - point_index
                
                for i in range(current_side_points):
                    # 计算全局归一化参数
                    t_global = point_index / (self.num_points_spiral - 1) if self.num_points_spiral > 1 else 0
                    
                    # 当前边内的局部参数
                    t_local = i / current_side_points if current_side_points > 0 else 0
                    
                    # 根据边的编号计算XY坐标
                    if side == 0:  # 从A到B
                        x = A_x + t_local * (B_x - A_x)
                        y = A_y + t_local * (B_y - A_y)
                    elif side == 1:  # 从B到C
                        x = B_x + t_local * (C_x - B_x)
                        y = B_y + t_local * (C_y - B_y)
                    else:  # side == 2 从C回到A
                        x = C_x + t_local * (A_x - C_x)
                        y = C_y + t_local * (A_y - C_y)
                    
                    # Z坐标：使用t_global确保全局线性上升
                    z = self.z_start_spiral + t_global * (self.z_end_spiral - self.z_start_spiral)
                    
                    rx, ry, rz = 0.0, 0.0, 0.0
                    
                    self._send_waypoint_tracking(x, y, z, rx, ry, rz)
                    point_index += 1
                    
                    # 如果已达到总点数，提前退出
                    if point_index >= self.num_points_spiral:
                        break
                
                if point_index >= self.num_points_spiral:
                    break
            
            if point_index >= self.num_points_spiral:
                break
        
        self.get_logger().info(f"螺旋三角形轨迹点发送完成，共{point_index}个点")
        self.send_start_order()

    def move_circle_shaking(self):
        """
        生成姿态空间的摇头轨迹
        位置固定：x=0, y=0, z=40
        rx, ry: 绕三圈（类似于圆形轨迹）
        rz: 从0°线性变化到-30°
        """
        # 设置起始点和结束点
        start_pos = [0.0, 0.0, 45.0]
        
        # 固定位置
        x, y, z = start_pos
        
        # 发送起始点（位置固定，姿态为起始值）
        self._send_waypoint_fixed([x, y, z, 0.0, 10.0, 0.0])
        time.sleep(2)
        
        # 轨迹参数
        num_circles = 3  # 绕三圈
        rz_start = 0.0    # rz起始值
        rz_end = -9.0    # rz结束值
        amplitude = 10.0  # rx, ry的最大幅度（角度）
        
        # 生成轨迹点
        for i in range(self.num_points_shake):
            # 归一化参数 t: 0.0 ~ 1.0
            t = i / max(self.num_points_shake - 1, 1)
            
            # 计算当前相位（角度，单位：度）
            # 绕三圈：0° ~ 1080° (3 * 360°)
            phase = t * 360.0 * num_circles
            
            # 计算 rx, ry (使用三角函数)
            # 注意：rx是绕y轴逆时针，ry是绕x轴顺时针
            # 为了形成圆形轨迹，我们使用正弦和余弦函数
            rx = amplitude * math.sin(math.radians(phase))   # 绕y轴
            ry = amplitude * math.cos(math.radians(phase))   # 绕x轴
            
            # 计算 rz: 从0°线性变化到-30°
            rz = rz_start + t * (rz_end - rz_start)
            
            # 发送路径点
            self._send_waypoint_tracking(x, y, z, rx, ry, rz)
        
        # 发送最终点确保精确到达
        # self._send_waypoint_fixed([x, y, z, rx, ry, rz])
        self.get_logger().info("摇头轨迹点发送完成")
        self.send_start_order()

    def move_square_shaking(self):
        """
        生成位置固定、姿态按正方形螺旋变化的轨迹
        位置固定：x=0, y=0, z=40
        rx, ry: 按正方形轨迹变化，绕三圈
        rz: 从0°线性变化到-30°
        """
        # 固定位置
        x, y, z = 0.0, 0.0, 45.0
        
        # 发送起始点（姿态为起始值）
        self._send_waypoint_fixed([x, y, z, 5.0, 5.0, 0.0])
        time.sleep(2)
        
        # 轨迹参数
        num_circles = 3          # 绕三圈
        amplitude = 5.0         # rx, ry的最大幅度（度）
        rz_start = 0.0           # rz起始值
        rz_end = -30.0           # rz结束值
        
        # 正方形四个顶点（在rx-ry平面上）
        vertices = [
            ( amplitude,  amplitude),  # 右上角
            (-amplitude,  amplitude),  # 左上角
            (-amplitude, -amplitude),  # 左下角
            ( amplitude, -amplitude),  # 右下角
        ]
        
        # 计算每条边的点数
        points_per_circle = self.num_points_shake // num_circles  # 每圈点数
        points_per_side = points_per_circle // 4            # 每条边点数
        
        point_index = 0
        
        # 生成轨迹点
        for _ in range(num_circles):
            for side in range(4):  # 正方形有4条边
                # 当前边的起点和终点
                start_vertex = vertices[side]
                end_vertex = vertices[(side + 1) % 4]
                
                # 生成当前边上的点
                for i in range(points_per_side):
                    # 计算全局进度
                    t_global = point_index / max(self.num_points_shake - 1, 1)
                    
                    # 计算边内进度
                    t_local = i / points_per_side
                    
                    # 线性插值计算rx, ry
                    rx = start_vertex[0] + t_local * (end_vertex[0] - start_vertex[0])
                    ry = start_vertex[1] + t_local * (end_vertex[1] - start_vertex[1])
                    
                    # 计算rz: 从0°线性变化到-30°
                    rz = rz_start + t_global * (rz_end - rz_start)
                    
                    # 发送路径点
                    self._send_waypoint_tracking(x, y, z, rx, ry, rz)
                    
                    point_index += 1
        
        # 发送最终点
        # self._send_waypoint_fixed([x, y, z, rx, ry, rz])
        self.get_logger().info("正方形姿态螺旋轨迹点发送完成")
        self.send_start_order()

    def move_triangle_shaking(self):
        """
        生成位置固定、姿态按等边三角形螺旋变化的轨迹
        位置固定：x=0, y=0, z=40
        rx, ry: 按三角形轨迹变化，绕三圈
        rz: 从0°线性变化到-30°
        """
        # 固定位置
        x, y, z = 0.0, 0.0, 45.0
        
        # 发送起始点（姿态为起始值）
        self._send_waypoint_fixed([x, y, z, 10.0, 0.0, 0.0])
        time.sleep(2)
        
        # 轨迹参数
        num_circles = 3          # 绕三圈
        amplitude = 10.0         # 三角形外接圆半径（度）
        rz_start = 0.0           # rz起始值
        rz_end = -30.0           # rz结束值
        
        # 计算等边三角形三个顶点（在rx-ry平面上，逆时针）
        # 顶点1: (amplitude, 0)
        # 顶点2: (amplitude*cos(120°), amplitude*sin(120°))
        # 顶点3: (amplitude*cos(240°), amplitude*sin(240°))
        vertices = [
            (amplitude, 0.0),  # 顶点1
            (amplitude * math.cos(math.radians(120)), 
            amplitude * math.sin(math.radians(120))),  # 顶点2
            (amplitude * math.cos(math.radians(240)), 
            amplitude * math.sin(math.radians(240))),  # 顶点3
        ]
        
        # 计算每条边的点数
        points_per_circle = self.num_points_shake // num_circles  # 每圈点数
        points_per_side = round(points_per_circle // 3)            # 每条边点数
        
        point_index = 0
        
        # 生成轨迹点
        for _ in range(num_circles):
            for side in range(3):  # 三角形有3条边
                # 当前边的起点和终点
                start_vertex = vertices[side]
                end_vertex = vertices[(side + 1) % 3]
                
                # 生成当前边上的点
                for i in range(points_per_side):
                    # 计算全局进度
                    t_global = point_index / max(self.num_points_shake - 1, 1)
                    
                    # 计算边内进度
                    t_local = i / points_per_side
                    
                    # 线性插值计算rx, ry
                    rx = start_vertex[0] + t_local * (end_vertex[0] - start_vertex[0])
                    ry = start_vertex[1] + t_local * (end_vertex[1] - start_vertex[1])
                    
                    # 计算rz: 从0°线性变化到-30°
                    rz = rz_start + t_global * (rz_end - rz_start)
                    
                    # 发送路径点
                    self._send_waypoint_tracking(x, y, z, rx, ry, rz)
                    
                    point_index += 1
        
        # 发送最终点
        # self._send_waypoint_fixed([x, y, z, rx, ry, rz])
        self.get_logger().info(f"三角形姿态螺旋轨迹点发送完成，共{point_index}个点")
        self.send_start_order()
        
    def _read_x6pose_from_yaml(self) -> np.ndarray:
        """
        从 YAML 文件读取 x6pose 参数(6个float),返回 numpy 数组。
        若文件不存在或参数缺失,返回零向量。
        """
        default = np.zeros(6)
        try:
            with open(self.param_path, 'r', encoding='utf-8') as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    node_key = 'x6_server_node'
                    node_block = loaded.get(node_key, {})
                    ros_params = node_block.get('ros__parameters', {})
                    x6pose = ros_params.get('angle_zero', default.tolist())
                    return np.array(x6pose, dtype=float)
        except Exception as e:
            self.get_logger().warn(f'读取 {self.param_path} 失败,返回默认值：{e}')
        return default

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = ControllerNode()
    
    try:
        while rclpy.ok():
            try:
                mode = input("请输入命令: ")

                if mode == "movep":
                    x6_input = input("请输入目标位姿: ")
                    flag = node.check_input(x6_input)
                    if flag is None:
                        print("输入格式错误，请重新输入")
                        continue
                    pose_rp = node.input_read(x6_input).astype(float)
                    target_joints = pose_rp
                    node._send_waypoint_fixed(target_joints)

                elif mode == "move10":
                    node.move_repeat()
                    
                elif mode == "circle":
                    node.move_circle()
                    
                elif mode == "square":
                    node.move_square()
                    
                elif mode == "triangle":
                    node.move_triangle()

                elif mode == "cir_spiral":
                    node.move_circle_spiral()
                    
                elif mode == "squ_spiral":
                    node.move_square_spiral()
                    
                elif mode == "tri_spiral":
                    node.move_triangle_spiral()

                elif mode == "cir_shake":
                    node.move_circle_shaking()
                    
                elif mode == "squ_shake":
                    node.move_square_shaking()
                    
                elif mode == "tri_shake":
                    node.move_triangle_shaking()
                    
                elif mode == "quit" or mode == "exit":
                    print("退出程序")
                    break
                    
                else:
                    print(f"未知命令: {mode}")
                        
            except KeyboardInterrupt:
                print("\n接收到中断信号")
                break
            except Exception as e:
                print(f"命令执行出错: {e}")
                # 继续循环，不退出
                continue
                
    finally:
        # 只有退出主循环后才清理资源
        print("正在清理资源...")
        node.destroy_node()
        rclpy.shutdown()
        print("程序已退出")


if __name__ == '__main__':
    main()