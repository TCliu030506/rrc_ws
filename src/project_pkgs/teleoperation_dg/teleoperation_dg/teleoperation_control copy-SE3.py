import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import copy
import rclpy
import time  
from rclpy.node import Node
from omni_msgs.msg import OmniState
from ur5_rtde_control.ur5_rtde_control import URCONTROL
from teleoperation_dg.tool_transfer import TransCoordinate

class Subscriber(Node):

    def __init__(self):
        # 初始化节点
        super().__init__('Aimooe_coordinate_transfer')
        # 实例化 UR5 控制类
        self.ur = URCONTROL("192.168.1.102")
        # 实例化坐标转换类（视角不转移，末端有位置变化）
        O2A = [0, 0, 0, 0, 0, 0]
        tool = [0.0, 0.0329, 0.398, 0.0, 0.0, 0.0]
        self.trans = TransCoordinate(O2A_pos=O2A, trans_pos=tool)  
        # 创建主端机器人订阅者
        self.subscription = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.handle_msg,
            10
        )
        # 存储主端状态（在回调中更新，在发布线程中读取）变量
        self._lock = threading.Lock()
        self._master_position = None
        self._master_orientation = None
        self._master_velocity = None
        self._master_pose = None

        # 存储从端初始位姿与目标位姿变量
        self._initial_pose = None  # [x, y, z, rx, ry, rz]
        self._target_pose = None
        
        # 主端“零点”位姿（第一次进入遥操作时的参考位姿，用于做增量）
        self._ref_master_pose = None  # [x, y, z, rx, ry, rz]

        # 位置、姿态映射比例（可根据实际手感自行调整）
        self._pos_scale = [1.2, 1.2, 1.2]      # X、Y、Z 比例
        # 例如：主端旋转 1 rad，对应从端旋转 0.2 rad
        self._rot_scale = [0.6, 0.6, 0.6]      # RX、RY、RZ 比例

        # 启动 100Hz 发布线程
        self._pub_thread = threading.Thread(
            target=self._publish_target_pose_loop,
            daemon=True
        )
        self._pub_thread.start()
    
    
    def handle_msg(self, msg):
        # 存储接收到的数据，供发布线程使用
        with self._lock:
            # 这里用 deepcopy，避免另一个线程修改同一对象导致竞态
            self._master_position = copy.deepcopy(msg.pose.position)
            self._master_orientation = copy.deepcopy(msg.pose.orientation)
            self._master_velocity = copy.deepcopy(msg.velocity)
            # 将单位由mm转换为m，并转换为 UR 风格的旋转向量
            self._master_position.x /= 1000.0
            self._master_position.y /= 1000.0
            self._master_position.z /= 1000.0
            # 将四元数转换为旋转向量
            quat = [self._master_orientation.x, self._master_orientation.y, self._master_orientation.z, self._master_orientation.w]
            rot_vec = R.from_quat(quat).as_rotvec()
            self._master_pose = [self._master_position.x, self._master_position.y, self._master_position.z, rot_vec[0], rot_vec[1], rot_vec[2]]
    
    
    def _publish_target_pose_loop(self):
        period = 0.01  # 100 Hz
        self.get_logger().info('Publish thread started.')  # 调试用
        while rclpy.ok():
            with self._lock:
                master_pose = copy.deepcopy(self._master_pose)
            if master_pose is not None:
                target_pose = self._map_master_to_slave_pose(master_pose)
                if target_pose is not None:
                    # 发布目标位姿（这里假设你有发布器，示例中用print代替）
                    self.get_logger().debug(f'Target Pose: {target_pose}')
                    # 实际发布代码：self.publisher.publish(target_pose)
                    self.ur.sevol_l(target_pose)
            time.sleep(period)
    
    
    def _map_master_to_slave_pose(self, master_pose):
        # 初始化参考位姿（第一次调用时）
        if self._ref_master_pose is None:
            self._ref_master_pose = copy.deepcopy(master_pose)
        
        # 初始化从端初始位姿（第一次调用时，从法兰位姿转换为工具位姿）
        if self._initial_pose is None:
            flange_pose = self.ur.get_tcp_pose()  # 假设返回法兰位姿 [x,y,z,rx,ry,rz]
            self._initial_pose = flange_pose
            # self._initial_pose = self.trans.trans_O_to_tool(flange_pose)  # 转换为工具位姿
        
        # 计算 SE(3) 相对变换
        T_ref = self._pose_to_se3(self._ref_master_pose)
        T_cur = self._pose_to_se3(master_pose)
        T_rel = np.linalg.inv(T_ref) @ T_cur  # 相对变换
        
        # 缩放相对变换
        T_rel_scaled = self._scale_se3(T_rel, self._pos_scale, self._rot_scale)
        
        # 应用到从端初始位姿
        T_initial = self._pose_to_se3(self._initial_pose)
        T_target = T_initial @ T_rel_scaled  # 假设增量在末端系表达（右乘）
        
        # 转换回 [x,y,z,rx,ry,rz]
        target_pose = self._se3_to_pose(T_target)
        
        # 转换回法兰位姿（如果需要）
        # target_pose = self.trans.trans_tool_to_O(target_pose)
        
        return target_pose
    
    
    def _pose_to_se3(self, pose):
        """将 [x,y,z,rx,ry,rz] 转换为 4x4 SE(3) 矩阵"""
        x, y, z, rx, ry, rz = pose
        R_mat = R.from_rotvec([rx, ry, rz]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = [x, y, z]
        return T
    
    
    def _se3_to_pose(self, T):
        """将 4x4 SE(3) 矩阵转换为 [x,y,z,rx,ry,rz]"""
        x, y, z = T[:3, 3]
        R_mat = T[:3, :3]
        rot_vec = R.from_matrix(R_mat).as_rotvec()
        rx, ry, rz = rot_vec
        return [x, y, z, rx, ry, rz]
    
    
    def _scale_se3(self, T, pos_scale, rot_scale):
        """对 SE(3) 变换进行缩放：平移和旋转分别缩放"""
        # 提取平移和旋转
        t = T[:3, 3]
        R_mat = T[:3, :3]
        
        # 缩放平移
        t_scaled = t * np.array(pos_scale)
        
        # 缩放旋转：使用 logmap 到旋转向量，缩放，然后 expmap
        rot_vec = R.from_matrix(R_mat).as_rotvec()
        rot_vec_scaled = rot_vec * np.array(rot_scale)
        R_scaled = R.from_rotvec(rot_vec_scaled).as_matrix()
        
        # 组合回变换矩阵
        T_scaled = np.eye(4)
        T_scaled[:3, :3] = R_scaled
        T_scaled[:3, 3] = t_scaled
        return T_scaled
    
    
    def _xyzquat_to_xyzrxryrz(self, position, orientation):
        # 如果需要转换四元数到旋转向量
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot_vec = R.from_quat(quat).as_rotvec()
        return [position.x, position.y, position.z, rot_vec[0], rot_vec[1], rot_vec[2]]



def main(args=None):
    rclpy.init(args=args)

    master_sub = Subscriber()
    
    rclpy.spin(master_sub)

    rclpy.shutdown() 

if __name__ == '__main__':
    main()