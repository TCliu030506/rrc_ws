import rclpy
import os
from pathlib import Path
import numpy as np
import threading
from rclpy.node import Node
from rclpy.parameter import Parameter
from lkactuator_python.lk_client import LK_Motor
from lkactuator_msg.msg import LkPosition
from x6_msg.msg import X6Position
import x6_kinematics.kinematics_X6 as X6
from x6_kinematics.kinematics_X6 import ExRot
import yaml
from msg_codec_py.msg_coder import MsgCoder
from msg_codec_cpp.srv import Script

"""
----------------------------------------
x6_service.py
----------------------------------------
version 1.1.1: x6_service.py 2026-1-23

- 提供了新的motor_seq以适配不同电机安装方式
- 删除了冗余代码
----------------------------------------
version 1.1: x6_service.py 2025-12-17
author: Yuchen Hong
email:  12425014@zju.edu.cn

新的特性：
- 多圈角度记录移动到lk_service.py
- 运动学函数与逆运动学函数与kinematics_X6.py最新版本一致
----------------------------------------
"""


PRINT_ENABLED = 0 #1为启用所有print 0为关闭

# 自定义 print 函数
def custom_print(*args, **kwargs):
    if PRINT_ENABLED == 1:
        print(*args, **kwargs)

class x6_server(Node):
    def __init__(self):
        super().__init__('x6_server_node')
        self.is_cmdmsg_on = False
        self.angles: list[int] = [0] * 6                        # 最新角度值, 单位: 0.001度
        self.joint: np.ndarray = np.array([0.0] * 6, dtype=float) # 校准后的关节角度（弧度）
        self.is_updated   = [False] * 6                         # 是否收到该路
        self.warn_once = False
        # self.motor_seq: list[int] = [5, 4, 3, 2, 1, 6]
        self.motor_seq: list[int] = [1, 2, 3, 4, 5, 6]
        self.motors = [LK_Motor(i) for i in self.motor_seq]

        package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.param_path = Path(os.path.join(package_path,'param', 'x6param.yaml'))
        self.declare_parameter('angle_zero', [0] * 6)
        self.angle_zero = np.array(self.get_parameter('angle_zero').value, dtype=int)

       
        self.X6_Kine = X6.X6_solver()
        self.X6Joint = X6.X6Joint()
        self.data_lock = threading.Lock()

        self.x6_sub = self.create_subscription(LkPosition,'lkposition',self.callback_lkpos,10)
        self.x6_pub = self.create_publisher(X6Position, "x6position", 10)    
        self.srv_pub = self.create_service(Script, "x6_service", self.x6srv_callback)
        self.tolk= self.create_client(Script, "lk_service")
        self.get_logger().info(f"Publisher created on topic: {'x6position'}")
        self.cmd_script = { b'ST': {'cmd': '2s'},
                            b'U6': {'cmd': '2s'},
                            b'L6': {'cmd': '2s'},
                            b'SP': {'cmd': '2s'},
                            b'M6': {'cmd': '2s', 'spd': 'q', 'px': 'd', 'py': 'd', 'pz': 'd', 'rx': 'd', 'ry': 'd', 'rz': 'd'},
        }

        # 每 1/100s 检查是否收齐一批
        self.create_timer(1./50, self.timer_callback)
        self.create_timer(1./0.2, self.check)
        self.get_logger().info('lkposition subscriber started')

    def x6srv_callback(self, req_srv: Script.Request, res: Script.Response):
        with self.data_lock:
            try:
                encoded_msg = bytes(req_srv.command.data)
                cmd_ = MsgCoder.msg_decode(encoded_msg, self.cmd_script)
                is_success = False
                match cmd_['cmd']:
                    case b'ST':
                        is_success = self.set_zero()
                    case b'U6':
                        is_success = self.unlock()
                    case b'L6':
                        is_success = self.lock()
                    case b'SP':
                        is_success = self.stop()
                    case b'M6':
                        is_success = self.move_abs(int(cmd_['spd']),float(cmd_['px']),float(cmd_['py']),float(cmd_['pz']),float(cmd_['rx']),float(cmd_['ry']),float(cmd_['rz']))
                    case _:
                        is_success = self.get_logger().warn(f"Unknown mode: {cmd_['cmd']}")
                if is_success:
                    res.reply= True
                else:
                    res.reply= False

            except Exception as e:
                self.get_logger().error(f"x6服务执行出错: {e}")
                res.reply = False
            return res
    
    def request_srv(self, req_srv: Script.Request):
        future = self.tolk.call_async(req_srv)
        #无法等待接收回复 会占用线程   

    def unlock(self):
        for motor in self.motors:
            self.request_srv(motor.unlock())
        return True

    def lock(self):
        for motor in self.motors:
            self.request_srv(motor.lock())
        return True
    
    def stop(self):
        for motor in self.motors:
            self.request_srv(motor.stop())
        return True

    def move_abs(self, maxspeed: int, px: float, py: float, pz: float, rx: float, ry: float, rz: float):
        """
        :param rx: 说明: 绕y轴旋转角度(单位:度)
        :type rx: float
        :param ry: 说明: 绕x轴旋转角度(单位:度)
        :type ry: float
        :param rz: 说明: 绕z轴旋转角度(单位:度)
        :type rz: float
        """
        pose_ = np.array([px,py,pz,rx,ry,rz])
        pose_rp = X6.ExPose(pose_[0:3], X6.ExRot.from_rp(pose_[3:6], degrees=True))

        # 判断是否有解
        q6 = self.X6_Kine.inverse_kinematics(pose_rp)
        if q6 is None:
            print(self.X6_Kine.error)
            return False
        else:
            joint_goal = q6.joint(True).flatten() * 1000
            joint_motor = -joint_goal + self.angle_zero    
            # self.get_logger().info(f"Joint motor: {joint_motor}")
            for i in range(6):
                self.request_srv(self.motors[i].move_angle_absolute(maxspeed,int(joint_motor[i])))   
            return True
        
        # 待验证是否跑通，先保留旧版
        # pose_rp = np.array([px,py,pz,rx,ry,rz])
        # pose_rp[3:6] = np.deg2rad(pose_rp[3:6])
        # # 判断是否有解
        
        # if not self.X6_Kine.invKine_pose_rp(pose_rp[0:3],pose_rp[3:6]):
        #     return False
        # else:
        #     q6 = self.X6_Kine.solution
        #     q6_goal = np.rad2deg(q6)*1000
        #     #重新排列为电机id顺序
        #     # joint_goal = q6_goal[[3, 0, 4, 1, 5, 2]]
        #     joint_goal = q6_goal
        #     # self.get_logger().info(f"Joint goal: {joint_goal}")
        #     # 加入角度差
        #     joint_motor = -joint_goal + self.angle_zero    
        #     # self.get_logger().info(f"Joint motor: {joint_motor}")
        
        # for i in range(6):
        #     self.request_srv(self.motors[i].move_angle_absolute(maxspeed,int(joint_motor[i])))   
        # return True
            
    def callback_lkpos(self, msg: LkPosition):
        """订阅回调：把 val 写入 angles 指定位置"""
        with self.data_lock:
            try:
                motor_id = int(msg.header.frame_id)
                if motor_id not in self.motor_seq:
                    self.get_logger().warn(f'frame_id not in seq: {motor_id}')
                    return
                idx = self.motor_seq.index(int(motor_id))
                # 写入角度值
                self.angles[idx] = msg.val
                #self.get_logger().info(f'{self.angles}')
                self.is_updated[idx] = True
                #self.get_logger().info(f'{self.is_updated}')
            except Exception as e:
                self.get_logger().error(f"Error decoding message: {e}")

    def check(self):
        """每5秒重置一次have，防止电机信号丢失而不知""" 
        if not all(self.is_updated):
            missing = [str(i+1) for i, ok in enumerate(self.is_updated) if not ok]
            #self.get_logger().info(f"等待电机ID: {','.join([str(self.motor_seq[i]) for i in missing])}  当前角度: {self.angles}")
            return
        self.is_updated = [False] * 6

    def timer_callback(self):
        angles_ = np.array(self.angles, dtype=float)
        # self.joint: np.ndarray = (-angles_ + self.angle_zero)/1000 #由于电机增量为角度减小，关节角度=-绝对角度+零点矫正值
        # self.joint: np.ndarray = (angles_ - self.angle_zero)/1000 #绳驱方案/电机绝对位置增大连杆角度增加（顺时针）
        self.joint: np.ndarray = (angles_ - self.angle_zero)/2000 #绳驱方案 传动比2：1
        self.pos_pub()

    def pos_pub(self):
        """
        发布校准后的关节角度和位姿消息
        """
        #解析角度数据，返回位姿并发布
        q6 = self.X6Joint.from_joint(self.joint, degrees=True)
        if (pose_out := self.X6_Kine.forward_kinematics(q6, precise=False)) is None:
            pose_rp_ = np.zeros(6)  
            if not self.warn_once:
                # print(self.X6_Kine.error)
                self.get_logger().warning("Forward kinematics has no solution!")
                self.stop()
                self.warn_once = True
            return
        else:   
            pose_rp_ = pose_out.pose_rp(True).flatten()
            self.warn_once = False

        msg_topub = X6Position()
        msg_topub.pose_rp = pose_rp_.tolist()
        msg_topub.joints = self.joint.tolist()
        self.x6_pub.publish(msg_topub)

    def set_zero(self)-> bool:
        """
        设定新的零位
        """
        pose_ = self._read_x6pose_from_yaml()
        # pose_[3:6] = ExRot.from_rotvec(pose_[3:6]).as_rp()
        pose_rp = X6.ExPose(pose_[0:3], X6.ExRot.from_rp(pose_[3:6], degrees=True))

        # 判断是否有解
        q6 = self.X6_Kine.inverse_kinematics(pose_rp)
        if q6 is None:
            print(self.X6_Kine.error)
            return False
        else:
            joint_6 = q6.joint(True).flatten() * 1000
            # angle_zero = np.array(self.angles) + joint_6 # 由于电机增量为角度减小，关节角度=-绝对角度+零点矫正值（直驱和磁耦合方案）
            # angle_zero = np.array(self.angles) - joint_6 #（绳驱方案）
            angle_zero = np.array(self.angles) - 2*joint_6 #（绳驱方案）传动比2：1
            angle_zero_int: list[int] = np.round(angle_zero).astype(int).tolist()

            # 更新参数到 ROS 参数服务器
            self.set_parameters([
                Parameter(
                    name='angle_zero',
                    type_=Parameter.Type.INTEGER_ARRAY,
                    value=angle_zero_int
                )
            ])

            # 打印日志确认
            self.get_logger().info(f"angle_zero 已更新为: {angle_zero_int}")
            # 写入 YAML，持久化
            self._save_angle_zero_to_yaml(angle_zero)
            # 立刻让当前进程生效（不走参数服务器回调，直接更新本地缓存）
            self.angle_zero = angle_zero
            return True

    def _read_x6pose_from_yaml(self) -> np.ndarray:
        """
        从 YAML 文件读取 x6pose 参数（6个float），返回 numpy 数组。
        若文件不存在或参数缺失，返回零向量。
        """
        default = np.zeros(6)
        try:
            with open(self.param_path, 'r', encoding='utf-8') as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    node_key = self.get_name()  # 'x6_server_node'
                    node_block = loaded.get(node_key, {})
                    ros_params = node_block.get('ros__parameters', {})
                    x6pose = ros_params.get('x6pose', default.tolist())
                    return np.array(x6pose, dtype=float)
        except Exception as e:
            self.get_logger().warn(f'读取 {self.param_path} 失败，返回默认值：{e}')
        return default

    def _save_angle_zero_to_yaml(self, angle_zero: np.ndarray):
        """
        将 angle_zero 写入 YAML 文件（原地更新/创建），结构示例：
        x6_server_node:
          ros__parameters:
            angle_zero: [ ..6个int.. ]
        采用原子写入防止损坏。
        """
        data = {}
        if self.param_path.exists():
            try:
                with open(self.param_path, 'r', encoding='utf-8') as f:
                    loaded = yaml.safe_load(f)
                    if isinstance(loaded, dict):
                        data = loaded
            except Exception as e:
                self.get_logger().warn(f'读取 {self.param_path} 失败，将覆盖写入：{e}')

        node_key = self.get_name()  # 'x6_server_node'
        node_block = data.get(node_key, {})
        ros_params = node_block.get('ros__parameters', {})

        ros_params['angle_zero'] = [int(x) for x in angle_zero.tolist()]
        node_block['ros__parameters'] = ros_params
        data[node_key] = node_block

        tmp = self.param_path.with_suffix(self.param_path.suffix + '.tmp')
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
            os.replace(tmp, self.param_path)  # 原子替换
            self.get_logger().info(f'已保存 angle_zero 到 {self.param_path}')
        except Exception as e:
            self.get_logger().error(f'写入 {self.param_path} 失败：{e}')
            if tmp.exists():
                try: tmp.unlink()
                except Exception: pass
            raise

def main():
    rclpy.init()
    node = x6_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()