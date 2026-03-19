import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
import threading   
import time  
import copy                                 
from geometry_msgs.msg import PoseStamped 
import math 
from ur5_rtde_control.ur5_rtde_control import URCONTROL

class Subscriber(Node):

    def __init__(self):
        # 初始化节点
        super().__init__('Aimooe_coordinate_transfer')
        # 示例化 UR5 控制类
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
            # 拷贝一份当前主端位姿
            with self._lock:
                master_pose = copy.deepcopy(self._master_pose)

            if master_pose is not None:
                # 基于增量映射计算从端目标位姿 [x, y, z, rx, ry, rz]
                target_pose = self._map_master_to_slave_pose(master_pose)
                self._target_pose = target_pose

                # 打印测试
                self.get_logger().info(
                    f'  Target Pose: x={target_pose[0]:.3g}, y={target_pose[1]:.3g}, z={target_pose[2]:.3g}, '
                    f'rx={target_pose[3]:.3g}, ry={target_pose[4]:.3g}, rz={target_pose[5]:.3g}'
                )
                # 直接调用 UR 的 movel 控制从端机器人运动：
                # self.ur.movel(target_pose)
                self.ur.sevol_l(target_pose)

            time.sleep(period)

    def _map_master_to_slave_pose(self, master_pose):
        """
        基于“增量式映射”的主从位姿映射函数。

        输入:
            master_pose: [x, y, z, rx, ry, rz]
                         主端当前位姿（已转换为 UR 风格的旋转向量）

        输出:
            target_pose: [x, y, z, rx, ry, rz]
                         从端（UR5）目标位姿
        """
        # 确保从端初始位姿存在
        if self._initial_pose is None:
            self._initial_pose = self.ur.get_tcp_pose()

        # 首帧：记录主端参考位姿，作为“零点”
        if self._ref_master_pose is None:
            self._ref_master_pose = master_pose
            # 首帧不移动，从端保持在初始位姿
            return self._initial_pose

        # 计算主端相对参考位姿的增量
        dx = master_pose[0] - self._ref_master_pose[0]
        dy = master_pose[1] - self._ref_master_pose[1]
        dz = master_pose[2] - self._ref_master_pose[2]
        drx = master_pose[3] - self._ref_master_pose[3]
        dry = master_pose[4] - self._ref_master_pose[4]
        drz = master_pose[5] - self._ref_master_pose[5]

        # 应用比例系数（可在 __init__ 中调整）
        sx, sy, sz = self._pos_scale
        srx, sry, srz = self._rot_scale

        dx *= sx
        dy *= sy
        dz *= sz
        drx *= srx
        dry *= sry
        drz *= srz

        # 叠加到从端初始位姿，得到目标位姿
        target_pose = [
            self._initial_pose[0] + dx,
            self._initial_pose[1] + dy,
            self._initial_pose[2] + dz,
            self._initial_pose[3] + drx,
            self._initial_pose[4] + dry,
            self._initial_pose[5] + drz,
        ]

        return target_pose
    
    def _xyzquat_to_xyzrxryrz(self, position, orientation):
        """
        将 (x, y, z + 四元数) 转换为 (x, y, z, rx, ry, rz)
        position: geometry_msgs.msg.Point
        orientation: geometry_msgs.msg.Quaternion
        返回: [x, y, z, rx, ry, rz]，其中 rx, ry, rz 为旋转向量 (轴 * 角度, 单位: rad)
        """
        x = position.x
        y = position.y
        z = position.z

        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # 归一化四元数，避免数值误差
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            # 无效四元数，退化为零旋转
            return [x, y, z, 0.0, 0.0, 0.0]
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        # 四元数 -> 轴角
        angle = 2.0 * math.acos(qw)          # 旋转角度
        s = math.sqrt(max(0.0, 1.0 - qw*qw)) # = sin(angle/2)
        eps = 1e-8

        if s < eps or angle < eps:
            # 角度非常小，近似认为没有旋转
            rx = ry = rz = 0.0
        else:
            ax = qx / s
            ay = qy / s
            az = qz / s
            # 旋转向量：轴 * 角度
            rx = ax * angle
            ry = ay * angle
            rz = az * angle

        return [x, y, z, rx, ry, rz]



def main(args=None):
    rclpy.init(args=args)

    master_sub = Subscriber()
    
    rclpy.spin(master_sub)

    rclpy.shutdown() 

if __name__ == '__main__':

    main()