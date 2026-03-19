import numpy as np

class TransCoordinate:
    """
    坐标系转换工具类
    O系：机械臂的基坐标系/世界坐标系
    A系：相机坐标系
    PF系：PF机构的工具坐标系
    """
    def __init__(self,
                 O2A_pos=None,              # O系到A系的转换参数[x,y,z,rx,ry,rz]
                 trans_pos=None):           # PF几何参数（6维[x,y,z,rx,ry,rz]）
        """
        参数可自定义：
        O2A_pos: O系到A系的转换参数[x,y,z,rx,ry,rz]，默认[0,0,0,0,-pi/2,0]
        trans_pos: PF几何参数，默认[-0.04196, 0, 0.16989, 0, 135度, 0]
        """
        self.O2A_pos = np.array(O2A_pos) if O2A_pos is not None else np.array([0, 0, 0, 0, -np.pi/2, 0])
        self.tool_pos = np.array(trans_pos) if trans_pos is not None else np.array([-0.04196, 0, 0.16989, 0, 135.0/180*np.pi, 0])

        # O系到A系的齐次变换矩阵
        self.T_OA = self.xyzRPY_to_homogeneous_transform(*self.O2A_pos)
        self.T_AO = np.linalg.inv(self.T_OA)

        # 法兰到工具坐标系的齐次变换矩阵
        self.T_FT = self.xyzRPY_to_homogeneous_transform(*self.tool_pos)
        self.T_TF = np.linalg.inv(self.T_FT)

    @staticmethod
    def homogeneous_transform(rotation, translation):
        """
        构建齐次变换矩阵
        参数：
            rotation: 3x3旋转矩阵
            translation: 3维平移向量
        返回：
            4x4齐次变换矩阵
        """
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

    @staticmethod
    def euler_angles_to_rotation_matrix(rx, ry, rz):
        """
        欧拉角转旋转矩阵（ZYX顺序）
        参数：
            rx, ry, rz: 绕X、Y、Z轴旋转角度（弧度）
        返回：
            3x3旋转矩阵
        """
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        return Rz @ Ry @ Rx

    def xyzRPY_to_homogeneous_transform(self, x, y, z, rx, ry, rz):
        """
        将[x, y, z, rx, ry, rz]转为齐次变换矩阵
        参数：
            x, y, z: 平移
            rx, ry, rz: 欧拉角（弧度）
        返回：
            4x4齐次变换矩阵
        """
        rotation = self.euler_angles_to_rotation_matrix(rx, ry, rz)
        translation = np.array([x, y, z])
        return self.homogeneous_transform(rotation, translation)

    @staticmethod
    def homogeneous_transform_to_xyzRPY(T):
        """
        齐次变换矩阵转[x, y, z, rx, ry, rz]
        参数：
            T: 4x4齐次变换矩阵
        返回：
            6维数组[x, y, z, rx, ry, rz]，弧度
        """
        translation = T[:3, 3]
        rotation = T[:3, :3]
        sy = np.sqrt(rotation[0,0]**2 + rotation[1,0]**2)
        singular = sy < 1e-6
        if not singular:
            rx = np.arctan2(rotation[2,1], rotation[2,2])
            ry = np.arctan2(-rotation[2,0], sy)
            rz = np.arctan2(rotation[1,0], rotation[0,0])
        else:
            rx = np.arctan2(-rotation[1,2], rotation[1,1])
            ry = np.arctan2(-rotation[2,0], sy)
            rz = 0
        return np.array([translation[0], translation[1], translation[2], rx, ry, rz])

    def tool_trans(self, pos):
        """
        A系下机械臂法兰位姿转为A系下工具末端位姿
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            A系下工具末端位姿（6维数组）
        """
        posMatrixEigen = self.xyzRPY_to_homogeneous_transform(*pos)
        targetMatrixEigen = posMatrixEigen @ self.T_FT
        return self.homogeneous_transform_to_xyzRPY(targetMatrixEigen)

    def tool_trans_inv(self, pos):
        """
        A系下工具末端位姿转为A系下机械臂末端位姿
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            机械臂末端位姿（6维数组）
        """
        posMatrixEigen = self.xyzRPY_to_homogeneous_transform(*pos)
        targetMatrixEigen = posMatrixEigen @ self.T_TF
        return self.homogeneous_transform_to_xyzRPY(targetMatrixEigen)

    def trans_OA(self, pos):
        """
        O系下机械臂位姿转为A系下机械臂位姿
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            A系下机械臂末端位姿（6维数组）
        """
        P_O = self.xyzRPY_to_homogeneous_transform(*pos)
        P_A = self.T_AO @ P_O
        return self.homogeneous_transform_to_xyzRPY(P_A)

    def trans_AO(self, pos):
        """
        A系下机械臂位姿转为O系下机械臂位姿
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            O系下机械臂末端位姿（6维数组）
        """
        P_A = self.xyzRPY_to_homogeneous_transform(*pos)
        P_O = self.T_OA @ P_A
        return self.homogeneous_transform_to_xyzRPY(P_O)

    def trans_AO_16(self, pos):
        """
        O系下机械臂位姿转为A系下机械臂位姿（齐次矩阵16维）
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            A系下机械臂末端齐次矩阵（16维数组）
        """
        P_O = self.xyzRPY_to_homogeneous_transform(*pos)
        P_A = self.T_AO @ P_O
        return P_A.flatten()

    def trans_AO_16(self, pos):
        """
        A系下机械臂位姿转为O系下机械臂位姿（齐次矩阵16维）
        参数：
            pos: 6维数组[x, y, z, rx, ry, rz]
        返回：
            O系下机械臂末端齐次矩阵（16维数组）
        """
        P_A = self.xyzRPY_to_homogeneous_transform(*pos)
        P_O = self.T_OA @ P_A
        return P_O.flatten()

    def trans_tool_to_O_16(self, pos):
        """
        A系下的工具末端位姿转为O系下机械臂末端齐次矩阵（16维）
        参数：
            A系下的工具末端位姿： 6维数组[x, y, z, rx, ry, rz]
        返回：
            O系下机械臂末端齐次矩阵（16维数组）
        """
        P_A = self.tool_trans_inv(pos)
        P_O = self.trans_AO_16(P_A)
        return P_O.flatten()
    
    def trans_O_to_tool_16(self, pos):
        """
        O系下机械臂位姿转为A系下的工具末端齐次矩阵（16维）
        参数：
            O系下机械臂位姿： 6维数组[x, y, z, rx, ry, rz]
        返回：
            A系下的工具末端齐次矩阵（16维数组）
        """
        P_A = self.trans_OA(pos)
        P_A_tool = self.tool_trans(P_A)
        P_tool = self.xyzRPY_to_homogeneous_transform(*P_A_tool)
        return P_tool.flatten()
    
    def trans_O_to_tool(self, pos):
        """
        O系下机械臂位姿转为A系下的工具末端位姿（6维）
        参数：
            O系下机械臂位姿： 6维数组[x, y, z, rx, ry, rz]
        返回：
            A系下的工具末端位姿（6维数组）
        """
        P_A = self.trans_OA(pos)
        P_A_tool = self.tool_trans(P_A)
        return P_A_tool

    def trans_tool_to_O(self, pos):
        """
        A系下的工具末端位姿转为O系下机械臂末端位姿（6维）
        参数：
            A系下的工具末端位姿： 6维数组[x, y, z, rx, ry, rz]
        返回：
            O系下机械臂末端位姿（6维数组）
        """
        P_A = self.tool_trans_inv(pos)
        P_O6 = self.trans_AO(P_A)
        return P_O6


if __name__ == "__main__":
    # 指定参数实例化
    O2A = [0, 0, 0, 0, 0, 0]
    tool = [0.0, 0.0329, 0.398, 0.0, 0.0, 0.0]
    trans = TransCoordinate(O2A_pos=O2A, trans_pos=tool)
    print("O2A_pos:", trans.O2A_pos)
    print("tool_pos:", trans.tool_pos)
    
    # 初始测试位置 
    pos_O = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3])  # O系下的机械臂法兰位姿：[x, y, z, rx, ry, rz]，弧度
    print("初始O系下机械臂位姿:", pos_O)

    # 测试O系到A系的转换
    pos_A = trans.trans_OA(pos_O)
    print("从O系转移到A系后的位姿:", pos_A)

    # 测试A系到O系的转换
    pos_O_back = trans.trans_AO(pos_A)
    print("从A系转回到O系后的位姿:", pos_O_back)

    # 测试工具坐标系变换
    tool_pos = trans.tool_trans(pos_A)
    print("转移得到的A系下工具的位姿:", tool_pos)

    tool_pos_inv = trans.tool_trans_inv(tool_pos)
    print("转回得到的A系下机械臂的位姿:", tool_pos_inv)

    # 测试O系到工具坐标系的转换
    tool_pos_from_O = trans.trans_O_to_tool(pos_O)
    print("O系下机械臂位姿转为A系下工具末端位姿:", tool_pos_from_O)

    # 测试工具坐标系到O系的转换
    pos_O_from_tool = trans.trans_tool_to_O(tool_pos)
    print("A系下工具末端位姿转回O系下机械臂末端位姿:", pos_O_from_tool)