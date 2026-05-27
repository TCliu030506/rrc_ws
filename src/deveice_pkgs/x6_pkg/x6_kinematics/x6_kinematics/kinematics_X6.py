# -*- coding: utf-8 -*-
#!/usr/bin/env python3

"""
----------------------------------------
Kinematics solver for parallel mechanism X6
----------------------------------------
version 1.0.4: kinematics_X6.py 2025-11-25
重写了正逆运动学中对位置和姿态的调用方式,
以ExPose类作为统一的位姿表示,提升了代码的可读性和维护性:
以X6Joint类作为统一的关节角度表示, 便于关节角度的管理和操作:
- from_urs_sru: 从URS和SRU支链角度创建X6Joint对象
- from_joint: 从完整关节角度创建X6Joint对象
- urs: 获取URS支链角度
- sru: 获取SRU支链角度

新增了X6_solver类的函数:
- inverse_kinematics: 逆运动学求解, 输入ExPose, 输出X6Joint
- forward_kinematics: 正运动学求解, 输入X6Joint, 输出ExPose
原有的正逆运动学函数(如forkine_pose_axang, invkine_pose_axang等)仍然保留,但已被重构为调用新的类方法, 以保证向后兼容性
推荐使用新的类方法进行正逆运动学求解

大部分函数新增了degrees参数:
默认为False, 可以选择输入输出角度的单位为度或弧度

----------------------------------------
version 1.0.3: kinematics_X6.py 2025-11-10
增加了正逆运动学函数对坐标化姿态的支持,
并完善了代码注释说明

----------------------------------------
version 1.0.2: kinematics_X6.py 2025-11-03
对函数forkine_pose_axang和invkine_pose_axang进行了改进,
引入了try-except异常处理机制, 提升了代码的健壮性

----------------------------------------
version 1.0.1: kinematics_X6.py 2025-10-30
author: Yunjiang Wang
email:  yunjiang.wang@zju.edu.cn

新的特性：
- 增加了YAML配置文件: X6_config.yaml, 便于参数调整
- 优化了正运动学算法, 通过保存上一次计算结果作为参考值, 提高了正运动学收敛速度
----------------------------------------
"""

import numpy as np
import yaml
from typing import Dict, Any, Union
from scipy.spatial.transform import Rotation
import os
import traceback

# 获取模块所在目录的绝对路径
module_dir = os.path.dirname(os.path.realpath(__file__))
# 构建配置文件的完整路径
config_path = os.path.join(module_dir, "X6_config.yaml")

# 打开并读取 YAML 文件
with open(config_path, "r", encoding="utf-8") as file:
    config: Dict[str, Any] = yaml.safe_load(file)

def Unit(v: Union[np.ndarray, float]) -> Union[np.ndarray, float]:
    if isinstance(v, (int, float)):
        return 0.0 if abs(v) < 1e-10 else (1.0 if v > 0 else -1.0)
    if v.ndim != 1:
        raise ValueError("只支持一维数组")
    norm = np.linalg.norm(v)
    if norm < 1e-10:
        if len(v) == 1:
            return np.array([0.0])
        raise ValueError("Cannot normalize the zero vector.")
    return v / norm

class ExRot(Rotation):
    """
    扩展的旋转类，继承自 scipy.spatial.transform.Rotation
    支持坐标化姿态（rp）与倾扭角（rtat）之间的转换
    倾扭角（rtat）定义为：
    rtat = [fai, theta, delta]
    其中 fai 是方向角，theta 是偏转角，delta 是扭转角

    坐标化姿态（rp）定义为：
    rx = theta * cos(fai)
    ry = theta * sin(fai)
    rz = delta
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @classmethod
    def create_self(cls, rot: Rotation) -> "ExRot":
        """
        从 Rotation 对象创建 ExRot 对象
        """
        quat = rot.as_quat()
        return cls(quat, normalize=False, copy=False)

    @staticmethod
    def rotx(angle_ :float, degrees: bool = False) -> np.ndarray:
        return Rotation.from_rotvec(np.array([angle_, 0., 0.]), degrees=degrees).as_matrix()
    
    @staticmethod
    def roty(angle_ :float, degrees: bool = False) -> np.ndarray:
        return Rotation.from_rotvec(np.array([0., angle_, 0.]), degrees=degrees).as_matrix()
    
    @staticmethod
    def rotz(angle_ :float, degrees: bool = False) -> np.ndarray:
        return Rotation.from_rotvec(np.array([0., 0., angle_]), degrees=degrees).as_matrix()
    
    @classmethod
    def from_rtat(cls, rtat_ :np.array, degrees: bool = False) -> "ExRot":
        rotm = cls.rotz(rtat_[0], degrees=degrees) @ cls.roty(rtat_[1], degrees=degrees) @ cls.rotz(rtat_[2]-rtat_[0], degrees=degrees)
        return cls.create_self(Rotation.from_matrix(rotm))
    
    @classmethod
    def from_rp(cls, rp_ :np.array, degrees: bool = False) -> "ExRot":
        if degrees:
            rp_ = np.radians(rp_)
        rtat_ = np.array([np.arctan2(rp_[1].item(),rp_[0].item()), np.linalg.norm(rp_[0:2]), rp_[2].item()])
        rotm = cls.rotz(rtat_[0], degrees=degrees) @ cls.roty(rtat_[1], degrees=degrees) @ cls.rotz(rtat_[2]-rtat_[0], degrees=degrees)
        return cls.create_self(Rotation.from_matrix(rotm))
    
    @classmethod
    def from_euler(cls, seq: str, angles: np.ndarray, degrees: bool = False) -> "ExRot":
        return cls.create_self(Rotation.from_euler(seq, angles, degrees=degrees))
    
    @classmethod
    def from_rotvec(cls, rotvec: np.ndarray, degrees: bool = False) -> "ExRot":
        return cls.create_self(Rotation.from_rotvec(rotvec, degrees=degrees))
    
    @classmethod
    def from_quat(cls, quat: np.ndarray) -> "ExRot":
        return cls.create_self(Rotation.from_quat(quat))
    
    @classmethod
    def from_matrix(cls, rotm: np.ndarray) -> "ExRot":
        return cls.create_self(Rotation.from_matrix(rotm))
    
    def as_rtat(self, degrees: bool = False) -> np.ndarray:
        rmat_ = self.as_matrix()
        rtat_ = np.array([0.,0.,0.])
        rtat_[0] = np.arctan2(rmat_[1,2],rmat_[0,2])
        rtat_[1] = np.arccos(rmat_[2,2])
        if rtat_[1] < 1e-6:
            rtat_[2] = np.arctan2(rmat_[1,0],rmat_[0,0])
        else:
            delta_fai = np.arctan2(rmat_[2,1],-rmat_[2,0])
            rtat_[2] = delta_fai + rtat_[0]
            if rtat_[2]>np.pi:
                rtat_[2] -= 2*np.pi
            elif rtat_[2] < -np.pi:
                rtat_[2] += 2*np.pi
        if degrees:
            rtat_ = np.degrees(rtat_)
        return rtat_
    
    def as_rp(self, degrees: bool = False) -> np.ndarray:
        rtat_ = self.as_rtat(False)
        rp = np.array([0.,0.,0.])
        rp[0] = rtat_[1] * np.cos(rtat_[0])
        rp[1] = rtat_[1] * np.sin(rtat_[0])
        rp[2] = rtat_[2]
        if degrees:
            rp = np.degrees(rp)
        return rp

class ExPose:
    """
    位姿类，包含位置和旋转信息
    位置使用 (x, y, z) 表示，旋转使用 ExRot 类表示
    位置和旋转可以通过多种方式进行初始化
    例如：
    pose = ExPose(np.array([x, y, z]), rotation)  # rotation 可以是 Rotation、ExRot、或旋转矩阵
    """
    def __init__(self, position: np.ndarray = np.zeros(3), rotation = None):
        """
        初始化 ExPose 类，包含位置和旋转
        :param x: 位置的 x 坐标
        :param y: 位置的 y 坐标
        :param z: 位置的 z 坐标
        :param rotation: 旋转信息，可以是Rotation、ExRot、和旋转矩阵，默认 None 表示单位旋转
        """
        self.set_position(position[0], position[1], position[2])
        self.set_rotation(rotation)
        
    def position(self)-> np.ndarray:
        """
        返回位姿的位置部分
        :return: 位置向量 (x, y, z)
        """
        return np.array([self.x, self.y, self.z])
    
    def orientation(self):
        """
        返回位姿的旋转部分
        :return: 旋转对象 ExRot
        """
        return self.rotation

    def transfer_matrix(self):
        """
        返回位姿的传递矩阵表示
        :return: 传递矩阵（4x4）
        """
        return np.block([[self.rotation.as_matrix(), self.position().reshape(3,1)],
                         [0, 0, 0, 1]])
        
    def pose_euler(self, seq: str = 'xyz', deg: bool = False) -> np.ndarray:
        """
        返回位姿的欧拉角表示 (x, y, z, roll, pitch, yaw), 默认顺序为 'xyz', 单位为弧度
        :return: 位姿向量 (6,)
        """
        return np.array([self.x, self.y, self.z, *self.rotation.as_euler(seq, degrees=deg)])
    
    def pose_rotvec(self, deg: bool = False) -> np.ndarray:
        """
        返回位姿的轴角表示 (x, y, z, rx, ry, rz), 默认单位为弧度
        :return: 轴角（6,）
        """
        return np.array([self.x, self.y, self.z, *self.rotation.as_rotvec(degrees=deg)])
    
    def pose_rp(self, deg: bool = False) -> np.ndarray:
        """
        返回位姿的坐标化姿态表示 (x, y, z, rx, ry, rz), 默认单位为弧度
        :return: 坐标化姿态（6,）
        """
        return np.array([self.x, self.y, self.z, *self.rotation.as_rp(degrees=deg)])
    
    def set_position(self, x, y, z):
        """
        设置新的位置
        :param x: 新的 x 坐标
        :param y: 新的 y 坐标
        :param z: 新的 z 坐标
        """
        self.x = x
        self.y = y
        self.z = z
    
    def set_rotation(self, rotation = None):
        """
        设置新的旋转信息
        :param rotation: 新的旋转信息，可以是Rotation、ExRot、和旋转矩阵，默认 None 表示单位旋转
        """
        if rotation is None:
            self.rotation = ExRot.identity()  # 默认单位旋转
        elif isinstance(rotation, Rotation):
            self.rotation = ExRot.from_matrix(rotation.as_matrix())  # 如果传入的是 Rotation 对象
        elif isinstance(rotation, ExRot):
            self.rotation = rotation  # 如果传入的是 ExRot 对象
        elif isinstance(rotation, np.ndarray):
            if rotation.shape == (3, 3):
                self.rotation = ExRot.from_matrix(rotation)  # 如果是旋转矩阵
            else:
                raise ValueError("Unsupported rotation input type.")
        else:
            raise ValueError("Rotation should be either a Rotation object, or a numpy array of shape (3, 3) for matrix.")
    
    @staticmethod
    def from_euler(position: np.ndarray, euler_angles: np.ndarray, deg: bool = False) -> "ExPose":
        """
        从欧拉角创建 ExPose 对象
        :param position: 位置向量 (3,)
        :param euler_angles: 欧拉角（单位：度或弧度，取决于 deg 参数）
        :param deg: 是否以度为单位，默认 False 表示弧度
        :return: 一个 ExPose 实例
        """
        rotation = ExRot.from_euler('xyz', euler_angles, degrees=deg)
        return ExPose(position, rotation)
    
    @staticmethod
    def from_rotvec(position: np.ndarray, rotvec: np.ndarray, deg: bool = False) -> "ExPose":
        """
        从旋转向量创建 ExPose 对象
        :param position: 位置向量 (3,)
        :param rotvec: 旋转向量（3,）
        :param deg: 是否以度为单位，默认 False 表示弧度
        :return: 一个 ExPose 实例
        """
        rotation = ExRot.from_rotvec(rotvec, degrees=deg)
        return ExPose(position, rotation)
    
    @staticmethod
    def from_rp(position: np.ndarray, rp: np.ndarray, degrees: bool = False) -> "ExPose":
        """
        从坐标化姿态创建 ExPose 对象
        :param position: 位置向量 (3,)
        :param rp: 坐标化姿态（3,）
        :param degrees: 是否以度为单位，默认 False 表示弧度
        :return: 一个 ExPose 实例
        """
        rotation = ExRot.from_rp(rp, degrees=degrees)
        return ExPose(position, rotation)

    @staticmethod
    def from_quaternion(position: np.ndarray, quaternion: np.ndarray) -> "ExPose":
        """
        从四元数创建 ExPose 对象
        :param position: 位置向量 (3,)
        :param quaternion: 四元数（4,）
        :return: 一个 ExPose 实例
        """
        rotation = ExRot.from_quat(quaternion)
        return ExPose(position, rotation)
    
    @staticmethod
    def from_matrix(position: np.ndarray, rotation_matrix: np.ndarray) -> "ExPose":
        """
        从旋转矩阵创建 ExPose 对象
        :param x: 位置的 x 坐标
        :param y: 位置的 y 坐标
        :param z: 位置的 z 坐标
        :param rotation_matrix: 旋转矩阵（3x3）
        :return: 一个 ExPose 实例
        """
        rotation = ExRot.from_matrix(rotation_matrix)
        return ExPose(position, rotation)
    
    def __repr__(self):
        return f"ExPose(Position: ({self.x}, {self.y}, {self.z}), Rotation: {self.rotation.as_matrix()})"

class X6Joint:
    """
    X6 关节类，包含六个关节角度, 单位为弧度.
    包含URS支链角度theta和SRU支链角度phi，
    依次为 phi_1, theta_1, phi_2, theta_2, phi_3, theta_3
    """
    def __init__(self, angles: np.ndarray = np.zeros(6)):
        """
        初始化 X6Joint 类，包含六个关节角度
        :param angles: 关节角度向量 (6,)
        """
        if angles.shape != (6,):
            raise ValueError("Angles must be a numpy array of shape (6,).")
        self.angles = angles
    
    def urs(self, degrees=False) -> np.ndarray:
        """
        获取URS支链的关节角度
        :return: 关节角度向量 (3,)
        """
        if degrees:
            return np.degrees(self.angles[1::2])
        return self.angles[1::2]
    
    def sru(self, degrees=False) -> np.ndarray:
        """
        获取SRU支链的关节角度
        :return: 关节角度向量 (3,)
        """
        if degrees:
            return np.degrees(self.angles[0::2])
        return self.angles[0::2]
    
    def joint(self, degrees=False) -> np.ndarray:
        """
        获取所有关节角度
        :return: 关节角度向量 (6,)
        """
        if degrees:
            return np.degrees(self.angles)
        return self.angles
    
    @staticmethod
    def from_urs_sru(theta: np.ndarray, phi: np.ndarray, degrees=False) -> "X6Joint":
        """
        从URS和SRU支链的关节角度创建 X6Joint 对象
        :param theta: URS支链关节角度向量 (3,)
        :param phi: SRU支链关节角度向量 (3,)
        :param degrees: 是否以度为单位，默认 False 表示弧度
        :return: 一个 X6Joint 实例
        """
        if theta.shape != (3,) or phi.shape != (3,):
            raise ValueError("URS and SRU angles must be numpy arrays of shape (3,).")
        if degrees:
            theta = np.radians(theta)
            phi = np.radians(phi)
        angles = np.zeros(6)
        angles[1::2] = theta
        angles[0::2] = phi
        return X6Joint(angles)
    
    @staticmethod
    def from_joint(angles: np.ndarray, degrees=False) -> "X6Joint":
        """
        从关节角度创建 X6Joint 对象
        :param angles: 关节角度向量 (6,)
        :param degrees: 是否以度为单位，默认 False 表示弧度
        :return: 一个 X6Joint 实例
        """
        if degrees:
            angles = np.radians(angles)
        return X6Joint(angles)
    
    def __repr__(self):
        return f"X6Joint(Angles(deg): {np.degrees(self.angles)})"

def cos_law(a_ :float, b_ :float, c_ :float) -> float:
    if ((a_ + b_) < c_) or (abs(a_ - b_) > c_):
        raise ValueError('No solution for cosine law!')
    return np.arccos( (a_**2 + b_**2 - c_**2)/(2.0*a_*b_) )

def TrihedralAngle(alpha_ :float, beta_ :float, gamma_ :float) -> float:
    """
    二面角余弦公式
    """
    if  (alpha_ + beta_ - gamma_) > 0. and \
        (alpha_ - beta_ + gamma_) > 0. and \
        (-alpha_ + beta_ + gamma_) > 0.:
        theta_ = np.arccos((np.cos(gamma_) - np.cos(alpha_) * np.cos(beta_)) /
                    (np.sin(alpha_) * np.sin(beta_)) )
        return theta_
    else:
        raise ValueError('Function trihedralAngle has no solution!')

def DihedralAngle(u_axis :np.ndarray, u_1 :np.ndarray, u_2 :np.ndarray) -> float:
    """
    二面角向量公式
    """
    try:
        ua = Unit(u_axis)
        u1 = Unit(u_1)
        u2 = Unit(u_2)
        return Unit( np.dot( np.cross(ua,u1), u2) ) * np.arccos(np.dot( Unit(np.cross(ua,u1)), Unit(np.cross(ua,u2)) ))
    except ValueError:
        raise ValueError("Function dihedralAngle has no solution!")

def Coord_vec3(vec1_ :np.ndarray, vec2_ :np.ndarray, alpha_ :float, beta_ :float) -> np.ndarray:
    """
    根据与两个向量的夹角计算第三个向量
    """
    # 归一化输入向量
    try:
        u1 = Unit(vec1_)
        u2 = Unit(vec2_)
    except ValueError:
        raise ValueError("Input vectors must be non-zero.")
    
    # 计算 gamma 角度
    gamma_ = np.arccos(np.dot(u1, u2))
    
    # 判断是否满足三角形不等式
    if  (alpha_ + beta_ - gamma_) > 0. and \
        (alpha_ - beta_ + gamma_) > 0. and \
        (-alpha_ + beta_ + gamma_) > 0.:
        # 构造方程并求解 vec3
        A = np.array([u1, u2, np.cross(u1, u2)])  # 形成一个 3x3 的矩阵
        b = np.array([  np.cos(alpha_),
                        np.cos(beta_), 
                        np.sqrt((np.sin(alpha_) * np.sin(beta_)) ** 2 - (np.dot(u1, u2) - np.cos(alpha_) * np.cos(beta_)) ** 2)])
        
        
        # 求解方程 A * vec3 = b
        vec3 = np.linalg.inv(A) @ b
        
        return vec3
    else:
        raise ValueError("Function coord_vec3 has no solution!")

# kinematics solver for parallel mechanism X6
class X6_solver:
    def __init__(self) -> None:
        self.omega          = np.array(config['omega']) * np.pi / 3
        self.rho            = np.deg2rad( np.array(config['rho']) )
        self.radius         = np.array(config['radius'])
        self.ax3            = config['axis3']

        self.range_low      = np.deg2rad( np.array(config['range_low']) )
        self.range_upp      = np.deg2rad( np.array(config['range_upp']) )
        joint3_low          = np.arccos( self.radius[0]/self.radius[1] )
        joint3_upp          = self.rho[0] + np.arccos( self.radius[0]*np.sin(self.rho[0])/self.radius[1] )
        self.range_low[2]   = joint3_low
        self.range_upp[2]   = joint3_upp
        
        # 设置正运动学计算精度
        self.threshold      = np.deg2rad(config.get('threshold', 0.01))
        self.threshold_upp  = np.deg2rad(config.get('threshold_upp', 0.1))
        self.debug          = config.get('debug', False)
        self.solution       = np.array([0.,0.,0.,0.,0.,0.])
        self.error          = ""

        # 邻域搜索范围，单位弧度，用于快速初始姿态估计
        self.neighborhood   = np.deg2rad(config.get('neighborhood', 5.0))
        self.joint_old      = np.zeros(6)       # 上一次正运动学计算的关节角度: phi_1, theta_1, phi_2, theta_2, phi_3, theta_3
        self.rp_old         = np.array([0.,0.,0.])

        # 计算初始的坐标化姿态，更新 self.joint_old 和 self.rp_old
        theta_              = np.deg2rad( np.array([40., 40., 40.]) )
        phi_                = np.deg2rad( np.array([30., 30., 30.]) )
        self.forward_kinematics(X6Joint.from_urs_sru(theta_, phi_), precise=False)
            
    def __del__(self) -> None:
        print('\nX6_solver Closed.')

    def rp_init(self, theta_ :np.ndarray, phi_ :np.ndarray) -> np.ndarray:
        """
        初始姿态估计, 输入URS和SRU支链角度theta和phi, 给出坐标化姿态的预测值
        """
        joint = X6Joint.from_urs_sru(theta_, phi_).joint()
        distance = np.linalg.norm(joint - self.joint_old)
        if distance < self.neighborhood:
            return self.rp_old
        else:
            p_ = self.for_Position(theta_)
            p_unit_ = Unit(p_)
            rp_ = np.zeros(3)
            if np.arccos(p_unit_[2].item()) > np.deg2rad(0.001):
                rp_[0:2] = np.arccos(p_unit_[2].item()) * Unit(p_unit_[0:2])
            # rp_[2] = np.deg2rad(-15)
            return rp_

    def inverse_kinematics(self, pose: ExPose) -> X6Joint:
        """
        逆运动学, 输入位姿ExPose, 输出关节角度X6Joint
        若无解, 则返回None
        调用返回值时,需检查返回值是否为None
        """
        self.error = ""
        try:
            theta: np.ndarray = self.inv_Position(pose.position())
            phi: np.ndarray = self.inv_Orientation(pose.position(), pose.orientation().as_matrix())
            return X6Joint.from_urs_sru(theta, phi)
        except ValueError as e:
            self.error = str(e)
            traceback.print_exc()
            return None
        
    def forward_kinematics(self, joint6: X6Joint, precise: bool = True) -> ExPose:
        """
        正运动学, 输入关节角度X6Joint, 输出位姿ExPose
        precise: 是否要求精确解, 默认True
        若有精确解, 则返回位姿ExPose
        若无无解, 近似解, 或precise为True且无精确解, 则返回None
        默认精度阈值为0.1度(self.threshold_upp)
        调用返回值时,需检查返回值是否为None
        """
        self.error = ""
        try:
            [p_, rp_] = self.for_Orientation(joint6.urs(), joint6.sru())
            q6_0 = joint6.joint()
            if (q6 := self.inverse_kinematics(ExPose.from_rp(p_, rp_))) is None:
                self.error = "No forward kinematics solution!"
                return None
            bias = np.linalg.norm(q6_0 - q6.joint())
            if self.debug:
                print("Debug Info: FK Bias (deg): {:.6f}".format( np.degrees(bias) ))
            if bias > self.threshold_upp:
                self.error = "No accurate forward kinematics solution, Bias: {:.6f} rad".format( bias )
                if precise:
                    return None
            return ExPose.from_rp(p_, rp_)
        except ValueError as e:
            self.error = str(e)
            return None

    def invKine_pose_axang(self, p_ :np.ndarray, rvec_ :np.ndarray) -> bool:
        """
        逆运动学, 输入位置和角轴姿态, 输出是否有解, 并将值“角度theta和phi”存入self.solution变量
        """
        return self.invKine_pose_rp(p_, ExRot.from_rotvec(rvec_).as_rp())
        
    def invKine_pose_rp(self, p_ :np.ndarray, rp_ :np.ndarray) -> bool:
        """
        逆运动学, 输入位置和坐标化姿态, 输出是否有解, 并将值“角度theta和phi”存入self.solution变量
        """
        try:
            self.solution = self.inverse_kinematics(ExPose.from_rp(p_, rp_)).joint()
            return True
        except ValueError as e:
            self.error = str(e)
            return False
        
    def forKine_pose_axang(self, theta_ :np.ndarray, phi_ :np.ndarray) -> bool:
        """
        正运动学, 输入URS和SRU支链角度theta和phi, 输出是否有近似解, 并将值“位置和角轴姿态”存入self.solution变量
        """
        try:
            [p_, rp_] = self.for_Orientation(theta_, phi_)
            self.solution = np.hstack((p_, ExRot.from_rp(rp_).as_rotvec()))

            q6_0 = X6Joint.from_urs_sru(theta_, phi_).joint()
            q6 = self.inverse_kinematics(ExPose.from_rp(p_, rp_)).joint()
            if np.linalg.norm(q6_0 - q6) > self.threshold_upp:
                self.error = "No accurate forward kinematics solution!"
                return False
            else:
                return True
        except ValueError as e:
            self.error = str(e)
            return False
        
    def forKine_pose_rp(self, theta_ :np.ndarray, phi_ :np.ndarray) -> bool:
        """
        正运动学, 输入URS和SRU支链角度theta和phi, 输出是否有近似解, 并将值“位置和坐标化姿态”存入self.solution变量
        """
        try:
            [p_, rp_] = self.for_Orientation(theta_, phi_)
            self.solution = np.hstack((p_, rp_))

            q6_0 = X6Joint.from_urs_sru(theta_, phi_).joint()
            q6 = self.inverse_kinematics(ExPose.from_rp(p_, rp_)).joint()
            if np.linalg.norm(q6_0 - q6) > self.threshold_upp:
                self.error = "No accurate forward kinematics solution!"
                return False
            else:
                return True
        except ValueError as e:
            self.error = str(e)
            return False
    
    def _solve_delta(self, p_norm :float) -> float:
        if  p_norm > (self.radius[1] + self.radius[0]) or \
            p_norm < abs(self.radius[1] - self.radius[0]):
            raise ValueError("delta has no solution!")
        return cos_law(p_norm, self.radius[0], self.radius[1])

    def func_jacobian_rp(self, p_ :np.ndarray, rp_ :np.ndarray) -> np.ndarray:
        """
        计算姿态雅可比矩阵, 输入位置和坐标化姿态, 输出雅可比矩阵
        """
        # 计算初始的 phi_0
        phi_0 = self.inv_Orientation_rp(p_, rp_)
        
        # 步长（0.01度转换为弧度）
        step_ = np.deg2rad(0.01)
        
        # 修改 rp（即计算 delta_r）
        rp_step = rp_ + np.eye(3) * step_  # 生成单位矩阵并与步长相乘
        
        # 初始化 phi_ 为一个 3x3 的零矩阵
        phi_ = np.zeros((3, 3))
            
        # 计算每个偏导数
        for i in range(3):
            phi_[i, :] = self.inv_Orientation_rp(p_, rp_step[i, :])
        
        # 计算 delta_phi
        delta_phi = phi_ - phi_0
        
        # 计算雅可比矩阵
        # delta_rp = mat * delta_phi
        # delta_phi[1,2,3] = mat_inv * delta_rp[1,2,3]
        mat_inv_ = delta_phi.T / step_
        
        return np.linalg.inv(mat_inv_)
    
    def inv_Position(self, p_ :np.ndarray) -> np.ndarray:
        """
        位置逆运动学, 输入位置, 输出URS支链角度theta
        """
        delta_ = self._solve_delta(np.linalg.norm(p_))
        
        # 初始化数组
        pB_a = np.zeros((3, 3))
        psi_a = np.zeros(3)
        theta_ = np.zeros(3)

        for i in range(3):
            pB_a[:, i] = ExRot.rotz(self.omega[0,i]) @ np.array([1., 0., 0.])
            psi_a[i] = np.arccos( np.dot(pB_a[:, i], Unit(p_)) )
            
            cross_prod_unit = Unit( np.cross(pB_a[:, i], p_) )
            theta_[i] =  np.arccos(np.dot(cross_prod_unit, np.array([0., 0., 1.]))) \
                - TrihedralAngle(psi_a[i], self.rho[0], delta_)
        
        return theta_
    
    def inv_Orientation(self, p_ :np.ndarray, rmat_ :np.ndarray) -> np.ndarray:
        """
        姿态逆运动学, 输入位置和姿态旋转矩阵, 输出SRU支链角度phi
        """
        delta_ = self._solve_delta(np.linalg.norm(p_))
        
        # 初始化数组
        pB_b = np.zeros((3, 3))
        psi_b = np.zeros(3)
        theta_b = np.zeros(3)

        # 主循环 1
        for i in range(3):
            # 计算旋转矩阵并应用
            vec_B = ExRot.rotz(self.omega[1,i]) @ np.array([1., 0., 0.])
            pB_b[:, i] = rmat_ @ vec_B
            
            # 计算 psi_b(i)
            psi_b[i] = np.arccos(-np.dot(pB_b[:, i], Unit(p_)))
            
            # 计算 theta_b(i)
            cross_prod_unit = Unit( np.cross(pB_b[:, i], p_) )
            theta_b[i] = -np.arccos(np.dot(cross_prod_unit, rmat_ @ np.array([0., 0., -1.]))) + \
                TrihedralAngle(psi_b[i], self.rho[0], delta_)

        # 初始化其他数组
        pT_b = np.zeros((3, 3))
        p_m = np.zeros((3, 3))
        p_n = np.zeros((3, 3))
        phi_ = np.zeros(3)

        # 主循环 2
        for i in range(3):
            # 计算 vec_t2m 并更新 p_m
            vec_t2m = rmat_ @ ExRot.from_euler('zxz', \
                [self.rho[0], theta_b[i], self.omega[1,i]], degrees=False).as_matrix() \
                @ np.array([self.radius[0], 0., 0])
            
            # 计算 p_m
            p_m[:, i] = p_ + vec_t2m

            # 计算 p_n
            p_n[:, i] = p_m[:, i] + \
                self.ax3 * Coord_vec3(vec_t2m, p_m[:, i], self.rho[1], self.rho[2])
            
            # 计算 pT_b
            pT_b[:, i] = ExRot.rotz(self.omega[1,i]) @ np.array([1., 0., 0.])
            
            # 计算 sigma_bi
            sigma_bi = np.arccos(np.dot(pT_b[:, i], Unit(p_n[:, i])))
            
            # 计算 phi(i)
            cross_prod2_unit = Unit( np.cross(pT_b[:, i], p_n[:, i]) )
            phi_[i] = np.arccos(np.dot(cross_prod2_unit, np.array([0., 0., 1.]))) - \
                TrihedralAngle(sigma_bi, self.rho[4], self.rho[3])
        return phi_
    
    def inv_Orientation_rp(self, p_ :np.ndarray, rp_ :np.ndarray) -> np.ndarray:
        """
        姿态逆运动学, 输入位置和坐标化姿态, 输出SRU支链角度phi
        """
        rmat_ = ExRot.from_rp(rp_).as_matrix()
        return self.inv_Orientation(p_, rmat_)
    
    def for_Position(self, theta_ :np.ndarray) -> np.ndarray:
        """
        位置正运动学, 输入URS支链角度theta, 输出位置
        """
        # point M
        p_m = np.zeros((3, 3))  # 初始化空矩阵
        for i in range(3):
            p_m[:, i] = ExRot.from_euler('zxz', [self.rho[0], theta_[i], self.omega[0,i]], degrees=False).as_matrix() \
                @ np.array([self.radius[0], 0., 0])
        
        # 计算 r_o
        p_m_diff_1 = p_m[:, 0] - p_m[:, 1]
        p_m_diff_2 = p_m[:, 0] - p_m[:, 2]
        p_m_diff_3 = p_m[:, 1] - p_m[:, 2]
        
        cos_theta = np.dot( Unit(p_m_diff_1), Unit(p_m_diff_2) )
        r_o = np.linalg.norm(p_m_diff_3) / (2 * np.sin(np.arccos(cos_theta)))
        
        # 计算 p
        part1_ = self.radius[0] ** 2 - r_o ** 2
        part2_ = self.radius[1] ** 2 - r_o ** 2
        p_ = np.linalg.inv(p_m.T) @ np.ones(3) * (part1_ + np.sqrt(part1_ * part2_))
        
        return p_
    
    def for_Orientation(self, theta_ :np.ndarray, phi_ :np.ndarray) -> list:
        """
        姿态正运动学, 输入URS支链和SRU支链角度theta和phi, 输出位置和坐标化姿态
        """
        try_times = 10
        max_step = np.deg2rad(15.)  # 最大步长（15度转换为弧度）

        # 求解位置
        p_ = self.for_Position(theta_)

        # 获取起始点
        rp_ = self.rp_init(theta_,phi_)

        # 获取初始的 phi3
        phi3 = self.inv_Orientation_rp(p_, rp_)
        delta_phi = phi_ - phi3

        try_index = 1
        while np.linalg.norm(delta_phi) > self.threshold:
            try_index += 1
            if try_index > try_times:
                break

            phi_move = delta_phi
            if np.linalg.norm(phi_move) > max_step:
                phi_move = Unit(phi_move) * max_step

            # 计算雅可比矩阵
            mat = self.func_jacobian_rp(p_, rp_)

            # 求解 delta_rp
            delta_rp = mat @ phi_move

            # 更新 rp
            rp_ = rp_ + delta_rp

            # 计算新的 phi3
            phi3 = self.inv_Orientation_rp(p_, rp_)

            # 更新 delta_phi
            delta_phi = phi_ - phi3

        # 检查迭代次数
        if try_index > try_times:
            print(f"Failed after {try_times} times iterations")
        else:
            # 记忆计算结果
            self.rp_old = rp_
            self.joint_old = X6Joint.from_urs_sru(theta_, phi_).joint()
            if self.debug:
                print(f"Return result after {try_index} times iterations")

        # 返回结果
        return [p_, rp_]

    def joint_solution(self, pose: ExPose) -> np.ndarray:
        """
        输出当前位姿所有关节转角, joint为6x6矩阵, 第1~3行表示URS支链, 第4~6行表示SRU支链
        """
        omega_u = self.omega[0]
        omega_s = self.omega[1]
        rho_1, rho_2, rho_3, rho_4, rho_5 = self.rho
        r1, r2 = self.radius
        length = self.ax3
        p = pose.position()

        joint = np.zeros((6,6), float)

        pnorm = np.linalg.norm(p)
        joint[:, 2] = np.arccos((r1**2 + r2**2 - pnorm**2) / (2*r1*r2))
        delta = np.arccos((pnorm**2 + r1**2 - r2**2) / (2*pnorm*r1))
        rotm = pose.orientation.as_matrix()

        # ---- URS (行 0..2)
        for i in range(3):
            omega_ui = omega_u[i]
            v_Bu = ExRot.rotz(omega_ui) @ np.array([1.,0.,0.])
            psi  = np.arccos(np.dot(v_Bu, Unit(p)))

            theta = np.arccos(np.dot(Unit(np.cross(v_Bu, p)), np.array([0.,0.,1.]))) - \
                    TrihedralAngle(psi, rho_1, delta)
            joint[i, 0] = theta

            p_M   = ExRot.from_euler('zxz', [rho_1, theta, omega_ui], degrees=False).as_matrix() @ np.array([r1, 0., 0.])
            v_p2M = p_M - p
            v_M2N = Coord_vec3(v_p2M, p_M, rho_3, rho_2)
            p_N   = p_M + length * v_M2N

            p_B = v_Bu * r1
            joint[i, 1] = DihedralAngle(p_M, p_N, p_B)

            v_Tu  = rotm @ (ExRot.rotz(omega_ui) @ np.array([1.,0.,0.]))
            v_p2N = p_N - p
            v_OHu = Coord_vec3(v_Tu, v_p2N, rho_5, rho_4)

            joint[i, 3] = DihedralAngle(v_p2N, v_p2M, v_OHu)
            joint[i, 4] = DihedralAngle(v_OHu, v_Tu, v_p2N)

            sigma = np.arccos(np.dot(v_Tu, Unit(v_p2N)))
            phi   = -np.arccos(np.dot(Unit(np.cross(v_Tu, v_p2N)), np.array([0.,0.,1.]))) + \
                    TrihedralAngle(sigma, rho_5, rho_4)
            joint[i, 5] = phi

        # ---- SRU (行 3..5)
        for i in range(3):
            omega_si = omega_s[i]
            row  = i + 3
            v_Bu = rotm @ (ExRot.rotz(omega_si) @ np.array([1.,0.,0.]))
            psi  = np.arccos(-np.dot(v_Bu, Unit(p)))

            theta = -np.arccos(np.dot(Unit(np.cross(v_Bu, p)), np.array([0.,0.,-1.]))) + \
                    TrihedralAngle(psi, rho_1, delta)
            joint[row, 0] = theta

            v_p2M = rotm @ ExRot.from_euler('zxz', [rho_1, theta, omega_si], degrees=False).as_matrix() @ np.array([r1, 0., 0.])
            p_M   = p + v_p2M
            v_M2N = Coord_vec3(v_p2M, p_M, rho_2, rho_3)
            p_N   = p_M + length * v_M2N

            v_p2N = p_N - p
            joint[row, 1] = DihedralAngle(v_p2M, v_Bu, v_p2N)

            v_Tu  = ExRot.rotz(omega_si) @ np.array([1.,0.,0.])
            v_OHu = Coord_vec3(p_N, v_Tu, rho_4, rho_5)

            joint[row, 3] = DihedralAngle(p_N, v_OHu, p_M)
            joint[row, 4] = DihedralAngle(v_OHu, p_N, v_Tu)

            sigma = np.arccos(np.dot(v_Tu, Unit(p_N)))
            phi   =  np.arccos(np.dot(Unit(np.cross(v_Tu, p_N)), np.array([0.,0.,1.]))) - \
                    TrihedralAngle(sigma, rho_5, rho_4)
            joint[row, 5] = phi

        return joint    

