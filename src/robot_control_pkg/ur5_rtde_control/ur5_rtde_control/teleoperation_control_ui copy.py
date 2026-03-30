# 控制信号含义说明（来自话题 tus_control 的 UIControl.control_flag）：
#   1：机器人初始化
#   2：机器人作业准备（ur5 运动到初始位姿）
#   3：开启系统遥操作（根据主端位姿增量持续控制 UR5 运动）
#   4：关闭系统遥操作（暂停主从端位姿增量控制）
#   5：退出系统（程序退出，关闭所有线程、发布者、订阅者、节点）
#   10：系统遥操作中（持续根据主端位姿增量控制 UR5 运动）---本节点内部使用--

import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
import threading
import time
import copy
from geometry_msgs.msg import PoseStamped
import math
from ur5_rtde_control.ur5_rtde_control import URCONTROL
from ui_control_msg.msg import UiControl


class Subscriber(Node):

    def __init__(self):
        # 初始化节点
        super().__init__('Aimooe_coordinate_transfer')
        # 示例化 UR5 控制类（构造时即完成机器人初始化）
        self.ur = URCONTROL("192.168.1.102")
        # 创建主端机器人订阅者
        self.subscription = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.handle_msg,
            10
        )
        # 创建订阅者，用于接收控制遥操作开始/关闭的控制信号
        self.subscription_ui = self.create_subscription(
            UiControl,
            'tus_control',
            self.handle_msg_ui,
            10
        )
        # 存储遥操作控制信号变量（1~5 外加内部使用的 10）
        self._control_signal = None
        # 发布线程运行标志（用于退出）
        self._running = True

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
        self._pos_scale = [0.5, 0.5, 0.5]      # X、Y、Z 比例
        self._rot_scale = [0.1, 0.1, 0.1]      # RX、RY、RZ 比例

        # 作业准备位姿（UR5 初始位姿），支持参数配置
        self.declare_parameter(
            'job_ready_pose',
            [0.071, -0.50, 0.45, 0.00, math.pi, 0.00]  # 示例位姿，根据实际情况修改
        )
        self._job_ready_pose = self.get_parameter('job_ready_pose').value

        # 启动 100Hz 发布线程（根据控制信号决定是否执行遥操作）
        self._pub_thread = threading.Thread(
            target=self._publish_target_pose_loop,
            daemon=True
        )
        self._pub_thread.start()

    def handle_msg(self, msg: OmniState):
        # 存储接收到的数据，供发布线程使用
        with self._lock:
            self._master_position = copy.deepcopy(msg.pose.position)
            self._master_orientation = copy.deepcopy(msg.pose.orientation)
            self._master_velocity = copy.deepcopy(msg.velocity)
            # 将单位由 mm 转换为 m，并转换为 UR 风格的旋转向量
            self._master_position.x /= 1000.0
            self._master_position.y /= 1000.0
            self._master_position.z /= 1000.0
            self._master_velocity.x /= 1000.0
            self._master_velocity.y /= 1000.0
            self._master_velocity.z /= 1000.0
            self._master_pose = self._xyzquat_to_xyzrxryrz(
                self._master_position, self._master_orientation
            )
            # self.get_logger().info(
            #     f'Received Master Pose: '
            #     f'x={self._master_pose[0]:.3f}, y={self._master_pose[1]:.3f}, '
            #     f'z={self._master_pose[2]:.3f}, rx={self._master_pose[3]:.3f}, '
            #     f'ry={self._master_pose[4]:.3f}, rz={self._master_pose[5]:.3f}'
            # )

    def handle_msg_ui(self, msg: UiControl):
        """根据 UI 发送的控制信号（control_flag）执行不同功能。"""
        # 读取 control_flag 作为控制信号
        signal = int(msg.control_flag)
        with self._lock:
            self._control_signal = signal
        self.get_logger().info(f'Received Control Signal: {self._control_signal}')

        # 分发到具体功能
        self._handle_control_signal(signal)

    def _handle_control_signal(self, signal: int):
        """实现 1~5 控制命令的具体功能。"""
        if signal == 1:
            # 1：机器人初始化（URCONTROL 在节点启动时已完成初始化，这里仅重置内部状态）
            self.get_logger().info('Command 1: 机器人初始化（URCONTROL 已在节点启动时完成初始化）')
            with self._lock:
                self._initial_pose = None
                self._ref_master_pose = None
                # 如有需要可在此补充对 UR5 的复位动作

        elif signal == 2:
            # 2：机器人作业准备（ur5 运动到初始位姿）
            self.get_logger().info('Command 2: 机器人作业准备（运动到初始位姿）')
            try:
                # 使用参数 job_ready_pose 作为准备位姿
                ready_pose = list(self._job_ready_pose)
                self.ur.sevol_l(ready_pose, speed=0.001, acceleration=0.01)
                self.get_logger().info(f'UR5 sevol_l to job_ready_pose: {ready_pose}')
            except AttributeError:
                self.get_logger().warn(
                    'URCONTROL.sevol_l() 未实现，无法执行作业准备位姿运动。'
                )

        elif signal == 3:
            # 3：开启系统遥操作（开始主从增量映射）
            self.get_logger().info('Command 3: 开启系统遥操作')
            with self._lock:
                # 重新设定参考位姿，在 _map_master_to_slave_pose 中会使用
                self._ref_master_pose = None
                # 不清空 _initial_pose，这样可以保留之前的初始从端位姿

        elif signal == 4:
            # 4：关闭系统遥操作（暂停主从端位姿增量控制）
            self.get_logger().info('Command 4: 关闭系统遥操作')
            with self._lock:
                # 将控制信号保持为 4，发布线程会停止发送目标位姿
                # 清空参考位姿，下一次重新开启时重新标定零点
                self._ref_master_pose = None

        elif signal == 5:
            # 5：退出系统（程序退出，关闭所有线程、发布者、订阅者、节点）
            self.get_logger().info('Command 5: 退出系统')
            # 停止发布线程循环
            self._running = False
            # 调用 rclpy.shutdown，使 spin 退出
            rclpy.shutdown()

        else:
            # 其他控制信号（包括内部使用的 10）不在此处单独处理
            self.get_logger().debug(f'Unhandled control signal in dispatcher: {signal}')

    def _publish_target_pose_loop(self):
        """100Hz 发布线程：根据控制信号决定是否执行遥操作。"""
        period = 0.01  # 100 Hz
        self.get_logger().info('Publish thread started.')
        while rclpy.ok() and self._running:
            # 拷贝当前控制信号和主端位姿
            with self._lock:
                control = self._control_signal
                master_pose = copy.deepcopy(self._master_pose)

            # 仅在 control_signal 为 3 或 10 时执行遥操作
            if control not in (3, 10):
                time.sleep(period)
                continue

            if master_pose is not None:
                # 基于增量映射计算从端目标位姿 [x, y, z, rx, ry, rz]
                target_pose = self._map_master_to_slave_pose(master_pose)
                self._target_pose = target_pose
                try:
                    # 直接调用 UR 的 sevol_l 控制从端机器人运动
                    self.ur.sevol_l(target_pose)
                except AttributeError:
                    self.get_logger().warn(
                        'URCONTROL.sevol_l() 未实现，无法下发目标位姿到 UR5。'
                    )

            time.sleep(period)

        self.get_logger().info('Publish thread stopped.')

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
        # 如果收到“开启遥操作”（3），则在第一次进入时设置初始位姿并切到 10 状态
        if self._control_signal == 3:
            self._initial_pose = self.ur.get_tcp_pose()
            self.get_logger().info('Control started: Initial pose set.')
            self._ref_master_pose = None  # 重置主端参考位姿
            # 更新为 10 表示“遥操作进行中”
            self._control_signal = 10

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

        # 应用比例系数
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