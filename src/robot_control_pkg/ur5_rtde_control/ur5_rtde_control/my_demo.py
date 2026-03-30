import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
import threading   
import time  
import copy                                 
from geometry_msgs.msg import PoseStamped 
import math 
from ur5_rtde_control.ur5_rtde_control import URCONTROL


def main(args=None):
    rclpy.init(args=args)

    # 示例化 UR5 控制类
    ur = URCONTROL("192.168.1.102")
    # ur.movel([0.05, -0.25, 0.4, 2.9, 1.7, 0.3])
    _initial_pose = ur.get_tcp_pose()
    print("初始位姿: \n", _initial_pose)

    rclpy.shutdown() 

if __name__ == '__main__':

    main()