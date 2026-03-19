import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
import threading   
import time  
import copy                                 
import math 
from scipy.spatial.transform import Rotation as R
from ur5_rtde_control.ur5_rtde_control import URCONTROL

class Subscriber(Node):

    def __init__(self):
        # 初始化节点
        super().__init__('Aimooe_coordinate_transfer')
        # 实例化 UR5 控制类
        self.ur = URCONTROL("192.168.1.102")
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
            self._master_velocity.x /= 1000.0
            self._master_velocity.y /= 1000.0
            self._master_velocity.z /= 1000.0
            self._master_pose = self._xyzquat_to_xyzrxryrz(self._master_position, self._master_orientation)
            # # 打印测试
            # self.get_logger().info(
            #     f'Received Master Pose: x={self._master_pose[0]:.3f}, y={self._master_pose[1]:.3f}, z={self._master_pose[2]:.3f}, '
            #     f'rx={self._master_pose[3]:.3f}, ry={self._master_pose[4]:.3f}, rz={self._master_pose[5]:.3f}'
            # )
            # self.get_logger().info(f'Received Master Velocity: x={self._master_velocity.x}, y={self._master_velocity.y}, z={self._master_velocity.z}')


    def _publish_target_pose_loop(self):
        period = 0.01  # 100 Hz
        self.get_logger().info('Publish thread started.')  # 调试用
        while rclpy.ok():
            with self._lock:
                master_position = copy.deepcopy(self._master_position)
                master_orientation = copy.deepcopy(self._master_orientation)

            if master_position is not None and master_orientation is not None:
                target_pose = self._map_master_to_slave_pose(master_position, master_orientation)
                self._target_pose = target_pose
                self.ur.sevol_l(target_pose)
            time.sleep(period)

    def _map_master_to_slave_pose(self, master_position, master_orientation):
        """
        基于“增量式映射”的主从位姿映射函数（位置线性，姿态四元数复合）。

        输入:
            master_position: geometry_msgs.msg.Point
            master_orientation: geometry_msgs.msg.Quaternion

        输出:
            target_pose: [x, y, z, rx, ry, rz]
                         从端（UR5）目标位姿
        """
        # 确保从端初始位姿存在
        if self._initial_pose is None:
            self._initial_pose = self.ur.get_tcp_pose()
        # 首帧：记录主端参考位姿，作为“零点”
        if self._ref_master_pose is None:
            self._ref_master_pose = [master_position.x, master_position.y, master_position.z,
                                     master_orientation.x, master_orientation.y, master_orientation.z, master_orientation.w]
            return self._initial_pose

        # 位置增量（线性）
        dx = (master_position.x - self._ref_master_pose[0]) * self._pos_scale[0]
        dy = (master_position.y - self._ref_master_pose[1]) * self._pos_scale[1]
        dz = (master_position.z - self._ref_master_pose[2]) * self._pos_scale[2]

        # 姿态增量（四元数）
        master_ref_quat = [self._ref_master_pose[3], self._ref_master_pose[4], self._ref_master_pose[5], self._ref_master_pose[6]]
        master_cur_quat = [master_orientation.x, master_orientation.y, master_orientation.z, master_orientation.w]
        # 四元数归一化
        master_ref_rot = R.from_quat(master_ref_quat).inv()
        master_cur_rot = R.from_quat(master_cur_quat)
        delta_rot = master_cur_rot * master_ref_rot

        # 从端初始姿态（旋转向量 -> 四元数）
        slave_init_rot = R.from_rotvec([self._initial_pose[3], self._initial_pose[4], self._initial_pose[5]])
        # 复合后目标姿态（四元数）
        target_rot = delta_rot * slave_init_rot
        # 可选：缩放旋转增量（仅对轴角有效，四元数缩放需谨慎）
        axis_angle = target_rot.as_rotvec()
        axis_angle[0] *= self._rot_scale[0]
        axis_angle[1] *= self._rot_scale[1]
        axis_angle[2] *= self._rot_scale[2]

        # 叠加到从端初始位姿，得到目标位姿
        target_pose = [
            self._initial_pose[0] + dx,
            self._initial_pose[1] + dy,
            self._initial_pose[2] + dz,
            axis_angle[0],
            axis_angle[1],
            axis_angle[2],
        ]
        return target_pose
    
    def _xyzquat_to_xyzrxryrz(self, position, orientation):
        """
        使用 scipy.spatial.transform.Rotation 进行四元数到旋转向量的标准转换。
        """
        x = position.x
        y = position.y
        z = position.z
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotvec = R.from_quat(quat).as_rotvec()
        return [x, y, z, rotvec[0], rotvec[1], rotvec[2]]

    def _xyzrxryrz_to_xyzquat(self, pose):
        """
        使用 scipy.spatial.transform.Rotation 进行旋转向量到四元数的标准转换。
        """
        x, y, z, rx, ry, rz = pose
        quat = R.from_rotvec([rx, ry, rz]).as_quat()
        return [x, y, z, quat[0], quat[1], quat[2], quat[3]]

def main(args=None):
    rclpy.init(args=args)

    master_sub = Subscriber()
    
    rclpy.spin(master_sub)

    rclpy.shutdown() 

if __name__ == '__main__':

    main()