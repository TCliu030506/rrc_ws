import rclpy
import numpy as np
import math

from . import ur5_kinematics as ur5
from . import URControl
from .URControl import URNode
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from ur5_msg.msg import RobotState

import threading

from enum import Enum

import time


PI = math.pi
joint_base = np.array([0,0,0,0,0,0], dtype=np.float64)
threshold = 0.005  # m 根据实际情况调整

# Enum
class Posetype(Enum):
    """
    AXANG: 等效轴角     theta*[x,y,z]
    EULER: XYZ欧拉角    [x,y,z]
    QUATN: 四元数       [w,x,y,z]
    """
    AXANG = 1
    EULER = 2
    QUATN = 3


def select_mat(type) -> np.ndarray:
    """
    return transform: np.array(4,4)
    """
    ur_model = ur5.URposition
    if type == 1:
        # 角轴表示
        print("角轴表示输入格式:")
        print("轴X(m) 轴Y(m) 轴Z(m) 点X(m) 点Y(m) 点Z(m) 角度(°)")
        axis_x, axis_y, axis_z, point_x, point_y, point_z, angle = map(float, input().split())
        angle = np.radians(angle)
        rotate = R.from_rotvec(np.array([axis_x, axis_y, axis_z]) * angle).as_matrix()
        translation = np.array([point_x, point_y, point_z])
    elif type == 2:
        # 欧拉角表示
        print("欧拉角输入格式：")
        print("点X(m) 点Y(m) 点Z(m) 角度1(°) 角度2(°) 角度3(°)")
        point_x, point_y, point_z, euler1, euler2, euler3 = map(float, input().split())
        euler = np.radians([euler1, euler2, euler3])
        rotate = R.from_euler('xyz', euler).as_matrix()
        translation = np.array([point_x, point_y, point_z])
    elif type == 3:
        print("关节角度输入格式：")
        print("角度1(°) 角度2(°) 角度3(°) 角度4(°) 角度5(°) 角度6(°)")
        # 关节表示de关节2(°) 关节3(°) 关节4(°) 关节5(°) 关节6(°)")
        joint_angles = np.radians(list(map(float, input().split())))
        # 按照您的代码，似乎dhfwardkmatics函数返回的是一个4x4的转换矩阵
        # 因此您需要提取旋转部分并将其转换为旋转矩阵
        tm_matrix = ur_model.fwdkinematics_dh(joint_angles)
        rotate = tm_matrix[:3, :3]
        translation = tm_matrix[:3, 3]
    elif type == 4:
        # 四元数表示
        print("四元数输入格式：")
        print("位置X(m) 位置Y(m) 位置Z(m) 四元数w 四元数x 四元数y 四元数z")
        position_x, position_y, position_z, quat_w, quat_x, quat_y, quat_z = map(float, input().split())
        quat = np.array([quat_w, quat_x, quat_y, quat_z])
        rotate = R.from_quat(quat).as_matrix()
        translation = np.array([position_x, position_y, position_z])
    else:
        print("无效的输入类型")
        return None
    
    transform = np.eye(4)
    transform[:3, :3] = rotate
    transform[:3, 3] = translation
    
    return transform

def is_close_enough(joint_base, joint_target, threshold):
    for j_base, j_target in zip(joint_base, joint_target):
        if abs(j_base - j_target) > threshold:
            return False
    return True

def custom_msg_callback(msg):
    # joint_base.append(msg.joint_pos)
    joint_base = msg.joint_pos.copy()

def user_interface():
    ##URposition和URNode类的实例化
    ur_position = ur5.URposition()
    urnode = URControl.URNode()

    # //定义初始量
    # 定义目标位置、初始位置和中间位置的坐标数组（x,y,z)

    """
    不是好的选择
    target_p = [0] * 3  # 创建一个长度为3，初始值都为0的列表
    initial_p = [0] * 3
    middle_p = [0] * 3 
    """

    """
    注意设定成浮点数,否则默认为整数处理运算了
    """
    p_target    = np.array([0,0,0],dtype=np.float64)
    p_initial   = np.array([0,0,0],dtype=np.float64)
    p_middle    = np.array([0,0,0],dtype=np.float64)

    # 定义目标关节角度、初始关节角度、中间关节角度和关节目标存储的数组(关节)
    joint_target    = np.array([0,0,0,0,0,0],dtype=np.float64)  # 创建一个长度为6，初始值都为0的列表
    joint_initial   = np.array([0,0,0,0,0,0],dtype=np.float64)
    joint_middle    = np.array([0,0,0,0,0,0],dtype=np.float64)

    # 用于条件的变量
    type = 0  
    continue_flag = 0

    # 创建一个3D变换矩阵，这是一个4x4单位矩阵
    mat_initial = np.identity(4)
    mat_middle = np.identity(4)
    mat_target = np.identity(4)
    # 如果你需要旋转部分，可以使用scipy库中的Rotation
    rotation = R.from_euler('xyz', [0, 0, 0], degrees=True)
    rot_matrix = rotation.as_matrix()
    # 将旋转矩阵集成到变换矩阵中（如果需要旋转的话）
    mat_initial[:3, :3] = rot_matrix
    mat_middle[:3, :3] = rot_matrix
    mat_target[:3, :3] = rot_matrix

    ##用户输入获取直线运动的起始点坐标
    print("输入直线运动起点位姿")
    print("请选择输入运动起点位姿的方式:角轴表示(1),欧拉角表示(2),关节角度表示(3),四元数表示(4)")
    type = int(input())
    mat_initial = select_mat(type)

    print("输入直线运动终点位姿")
    print("请选择输入运动终点位姿的方式:角轴表示(1),欧拉角表示(2),关节角度表示(3),四元数表示(4)")
    type = int(input())
    mat_target = select_mat(type)
    mat_middle = mat_initial.copy()

    ##将机械臂运动至初始位置的逆运动学计算
    ur_position.set_joint_ref(joint_base)
    ur_position.invkinematics_dh(mat_initial, joint_initial)  ##三个输入变量需要解决前面矩阵定义问题
    
    ##此处用打印出至初始位置的关节角度，并判断是否执行
    print("直线运动初始位置关节角度依次为")
    for i in range(6):  # Python中的for循环和C++略有不同
        print(joint_initial[i] * 180 / PI)  # 假设PI已经被定义
    # 判断是否继续执行
    print("请确定是否需要继续执行运动: (继续运动请输入1,终止运动请输入0) ")
    continue_flag = int(input())  # 获取用户输入
    if continue_flag == 0:
        exit(0)  # 在Python中使用exit(0)来退出程序

    ##运动至初始位置
    #urnode.movej(joint_initial)
        
    # 将中间位置初始值赋值为运动起始点
    p_middle = mat_initial[:3,-1]
    # middle_p[0] = mat_initial[0, 3]
    # middle_p[1] = mat_initial[1, 3]
    # middle_p[2] = mat_initial[2, 3]

    # 初始位置点赋值
    p_initial = mat_initial[:3,-1]
    # initial_p[0] = mat_initial[0, 3]
    # initial_p[1] = mat_initial[1, 3]
    # initial_p[2] = mat_initial[2, 3]

    # 目标位置点赋值
    p_target = mat_target[:3,-1]
    # target_p[0] = mat_target[0, 3]
    # target_p[1] = mat_target[1, 3]
    # target_p[2] = mat_target[2, 3]
    
    ##每次运动的起点关节角度为上一次运动的目标关节角度，初始值为直线起点关节角度
    joint_middle = joint_initial.copy()

    # 将运动分为100段运动
    segments = 100 #是否考虑过如果分段太细,会有什么现象?
    # 太啰嗦了
    # stepX = (target_p[0] - initial_p[0]) / segments
    # stepY = (target_p[1] - initial_p[1]) / segments
    # stepZ = (target_p[2] - initial_p[2]) / segments

    for i in range(1, segments+1):
        # 求出第i段直线终点坐标
        p_next = p_initial*(segments-i)/segments + p_target*i/segments
        # 求出第i段运动终点的关节角度
        mat_middle[:3, 3] = p_next

        # 第i段运动逆运动学求解
        ur_position.set_joint_ref(joint_middle)
        ur_position.invkinematics_dh(mat_middle, joint_target)
        
        #输出该段终点关节角度关节角度
        print(f"直线运动第{i}个目标点对应的关节角度依次为：")
        for j in range(6):
            print(joint_target[j] * 180 / np.pi)  # 使用NumPy的pi常量
        
        #运动至该段目标位置
        #urnode.movej(joint_target)

        #此处应该判断此时机械臂是否运行至目标位置，如果没有则程序不继续往下运行
        while not is_close_enough(joint_base, joint_target, threshold):
            time.sleep(0.005)
            pass  # pass是一个占位符，应该被实际更新joint_base值的代码替换

        # 将终点关节角度作为下一段运动的初始关节角度
        joint_middle = joint_target.copy()

    # std::cout << "直线运动完成" << std::endl;
    print("直线运动完成")

def main(args=None):
    
    rclpy.init(args=args)
    node = rclpy.create_node('custom_msg_subscriber')
    subscriber = node.create_subscription(RobotState, 'robotstate', custom_msg_callback, 10)


    main_thread = threading.Thread(target=user_interface)
    main_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


