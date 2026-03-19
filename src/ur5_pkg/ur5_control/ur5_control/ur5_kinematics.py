import math
import numpy as np
cos = math.cos
sin = math.sin
PI = math.pi

D1 = 0.08935245198864346
D4 = 0.1108629663680774
D5 = 0.09481752424615827
D6 = 0.08250304262136304
A2 = 0.4255142946511841
A3 = 0.3922769788

class URposition:
    """
    求解UR运动学的类
    """
    def __init__(self) -> None:
        self.joint_limit = np.array([[-PI*2, -PI, -PI*5/6, -PI*4/3, -PI*2, -PI*2],
                                     [PI*2,   0,  PI*5/6,    PI/3,  PI*2,  PI*2]])
        self.joint_ref = np.array([0,0,0,0,0,0])
        pass

    def set_joint_ref(self, joint_set: np.ndarray):
        """
        设置参考关节坐标
        """
        assert isinstance(joint_set, np.ndarray), "Input must be a numpy array"
        assert joint_set.ndim == 1 and joint_set.shape[0] == 6, "Input must be a 1D array with 6 elements"
        self.joint_ref = joint_set

    def fwdkinematics_dh(self, joint_val: np.ndarray) -> np.ndarray:
        """
        计算正运动学(DH建模法)
        """
        c1 = cos(joint_val[0])
        s1 = sin(joint_val[0])
        c2 = cos(joint_val[1])
        s2 = sin(joint_val[1])
        c5 = cos(joint_val[4])
        s5 = sin(joint_val[4])
        c6 = cos(joint_val[5])
        s6 = sin(joint_val[5])
        c23 = cos(joint_val[1] + joint_val[2])
        s23 = sin(joint_val[1] + joint_val[2])
        c234 = cos(joint_val[1] + joint_val[2] + joint_val[3])
        s234 = sin(joint_val[1] + joint_val[2] + joint_val[3])

        tm = np.eye(4)
        tm[0][0] = c1 * c234 * c5 * c6 + s1 * s5 * c6 - c1 * s234 * s6
        tm[0][1] = -c1 * c234 * c5 * s6 - s1 * s5 * s6 - c1 * s234 * c6
        tm[0][2] = -c1 * c234 * s5 + s1 * c5
        tm[0][3] = -c1 * c234 * s5 * D6 + s1 * c5 * D6 + c1 * s234 * D5 - c1 * (A3 * c23 + A2 * c2) + D4 * s1
        tm[1][0] = s1 * c234 * c5 * c6 - c1 * s5 * c6 - s1 * s234 * s6
        tm[1][1] = -s1 * c234 * c5 * s6 + c1 * s5 * s6 - s1 * s234 * c6
        tm[1][2] = -s1 * c234 * s5 - c1 * c5
        tm[1][3] = -s1 * c234 * s5 * D6 - c1 * c5 * D6 + s1 * s234 * D5 - s1 * (A3 * c23 + A2 * c2) - D4 * c1
        tm[2][0] = s234 * c5 * c6 + c234 * s6
        tm[2][1] = -s234 * c5 * s6 + c234 * c6
        tm[2][2] = -s234 * s5
        tm[2][3] = -s234 * s5 * D6 - c234 * D5 - (A3 * s23 + A2 * s2) + D1

        return tm

    def invkinematics_dh(self, mat: np.ndarray, joint: np.ndarray) -> bool:
        """
        计算逆运动学(DH建模法)
        """
        flag = np.array([[1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1],
                         [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1]])

        solution = np.zeros((8, 6), dtype=np.float64)
        joint_val_check = np.zeros(8)
        has = False

        for i in range(8):
            if self.inv_solve(mat, solution[i], flag[i], self.joint_ref[5]):
                has = True
                shiftangle_array(solution[i],self.joint_ref)
                joint_val_check[i] = check_joint_array(self.joint_limit, solution[i])
                # Print solution
                if False:
                    print("fail one solution:")
                    print(solution[i] * 180 / math.pi)

            else:
                joint_val_check[i] = False

        if not has:
            print("Solutions all failed")
            return False

        # Check
        for i in range(8):
            if joint_val_check[i]:
                if not robot_pose_check(solution[i]):
                    joint_val_check[i] = False
        if False:
            print("fail one solution plus:")
            print(solution[i] * 180 / math.pi)

        # Select solution
        choice = 8
        distance = 12 * math.pi
        for i in range(8):
            if joint_val_check[i]:
                dis_sum = np.sum(np.abs(solution[i] - self.joint_ref))
                if distance > dis_sum:
                    distance = dis_sum
                    choice = i

        if 0 <= choice <= 7:
            joint[:] = solution[choice]
            # print("Adopted Inverse kinematics solution:", joint)
            return True
        else:
            # print("failed")
            return False

    def inv_solve(self, mat, joint, flag_in, joint6):
        """
        计算指定的一组逆运动学解(DH建模法)
        """
        # theta1
        t1_ = (mat[0, 3] - D6 * mat[0, 2]) ** 2 + (-mat[1, 3] + D6 * mat[1, 2]) ** 2 - D4 ** 2
        if t1_ < 0:
            print("机械臂无法达到该范围,(x,y)需要远离基座标系原点！")
            return False
        joint[0] = np.arctan2(D4, flag_in[0] * np.sqrt(t1_)) - np.arctan2(-mat[1, 3] + D6 * mat[1, 2],
                                                                          mat[0, 3] - D6 * mat[0, 2])
        limitangle(joint[0])
        tc1 = cos(joint[0])
        ts1 = sin(joint[0])

        # theta5
        t5_ = ts1 * mat[0, 2] - tc1 * mat[1, 2]
        if t5_ ** 2 > 1:
            print("Rotation matrix is not qualified")
            return False
        joint[4] = np.arctan2(flag_in[1] * np.sqrt(1.0 - t5_ ** 2), t5_)
        joint[4] = limitangle(joint[4])

        # theta6
        flag6 = np.sign(np.sin(joint[4]))
        if flag6 == 0:
            joint[5] = joint6
            joint[5] = limitangle(joint[5])
            theta234 = np.arctan2(mat[2, 0], tc1 * mat[0, 0] + ts1 * mat[1, 0]) - joint[5]
        else:
            joint[5] = np.arctan2(flag6 * (-ts1 * mat[0, 1] + tc1 * mat[1, 1]),
                                  flag6 * (ts1 * mat[0, 0] - tc1 * mat[1, 0]))
            joint[5] = limitangle(joint[5])
            theta234 = np.arctan2(-mat[2, 2] * flag6, -(tc1 * mat[0, 2] + ts1 * mat[1, 2]) * flag6)

        ts5 = sin(joint[4])
        tc234 = cos(theta234)
        ts234 = sin(theta234)

        # theta2
        M = -tc1 * mat[0, 3] - ts1 * mat[1, 3] - tc234 * ts5 * D6 + ts234 * D5
        N = D1 - mat[2, 3] - ts234 * ts5 * D6 - tc234 * D5
        mn = M * M + N * N + A2 * A2 - A3 * A3

        t2_ = 4.0 * A2 * A2 * (M * M + N * N) - mn ** 2
        if t2_ < 0:
            print("机械臂无法达到该范围，(x,y,z)不能超过半径750mm的球型空间!")
            return False
        joint[1] = np.arctan2(mn, flag_in[2] * np.sqrt(t2_)) - np.arctan2(M * A2, N * A2)
        joint[1] = limitangle(joint[1])

        # theta3
        tc2 = cos(joint[1])
        ts2 = sin(joint[1])

        joint[2] = np.arctan2(N - A2 * ts2, M - A2 * tc2) - joint[1]
        joint[2] = limitangle(joint[2])

        # theta4
        joint[3] = theta234 - joint[1] - joint[2]
        joint[3] = limitangle(joint[3])

        return True


def limitangle(theta):
    while theta > PI:
        theta -= 2.0 * PI
    while theta <= (-PI + 0.0001):
        theta += 2.0 * PI
    return theta


def limitangle_array(theta):
    for i in range(len(theta)):
        theta[i] = limitangle(theta[i])


def shiftangle(theta, base):
    while theta > (base + PI):
        theta -= 2.0 * PI
    while theta < (base - PI + 0.0001):
        theta += 2.0 * PI
    return theta


def shiftangle_array(theta, base):
    for i in range(len(base)):
        theta[i] = shiftangle(theta[i], base[i])


def check_joint(limit, index, joint):
    if joint < limit[0, index]:
        return False
    if joint > limit[1, index]:
        return False
    return True


def check_joint_array(limit, joint):
    for i in range(len(joint)):
        val = joint[i]
        while val <= limit[0, i]:
            val += PI * 2.0
        while val >= limit[1, i]:
            val -= PI * 2.0
        if val <= limit[0, i]:
            return False
        else:
            joint[i] = val
    return True


def robot_pose_check(joint):

    link2 = np.array([A2 * np.cos(np.pi + joint[1]), A2 * np.sin(np.pi + joint[1])])
    link3 = np.array([A3 * np.cos(np.pi + joint[1] + joint[2]), A3 * np.sin(np.pi + joint[1] + joint[2])])
    link5 = np.array([D5 * np.cos(np.pi + joint[1] + joint[2] + np.pi / 2 + joint[3]),
                      D5 * np.sin(np.pi + joint[1] + joint[2] + np.pi / 2 + joint[3])])

    link23 = link2 + link3
    link23_unit = link23 / np.linalg.norm(link23)

    link23_normal = link2 - np.dot(link2, link23_unit) * link23_unit
    if link23_normal.dot(np.array([0, 1])) < 0:
        return False

    return True


def pose_to_matrix(pose):
    X, Y, Z, RX, RY, RZ = pose

    # Rotation matrices for RX (Roll), RY (Pitch), RZ (Yaw)
    R_X = np.array([[1, 0, 0],
                    [0, math.cos(RX), -math.sin(RX)],
                    [0, math.sin(RX), math.cos(RX)]])

    R_Y = np.array([[math.cos(RY), 0, math.sin(RY)],
                    [0, 1, 0],
                    [-math.sin(RY), 0, math.cos(RY)]])

    R_Z = np.array([[math.cos(RZ), -math.sin(RZ), 0],
                    [math.sin(RZ), math.cos(RZ), 0],
                    [0, 0, 1]])

    # Combined rotation matrix
    R = np.dot(R_Z, np.dot(R_Y, R_X))

    # Translation vector
    t = np.array([X, Y, Z])

    # Construct the pose matrix (4x4 homogeneous transformation matrix)
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = R
    pose_matrix[:3, 3] = t

    return pose_matrix

def matrix_to_pose(mat):
    # Extract rotation matrix and translation vector
    R = mat[:3, :3]
    t = mat[:3, 3]

    # Calculate Roll (RX), Pitch (RY), Yaw (RZ) using Euler angles (RPY)
    RX = math.atan2(R[2, 1], R[2, 2])
    RY = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
    RZ = math.atan2(R[1, 0], R[0, 0])

    # Calculate X, Y, Z positions
    X = t[0]
    Y = t[1]
    Z = t[2]

    # Return [X, Y, Z, RX, RY, RZ]
    return [X, Y, Z, RX, RY, RZ]

# def RPY_rad_to_deg(RPY):
#     X, Y, Z, RX, RY, RZ = RPY
#     RX = RX*180/PI
#     RY = RY*180/PI
#     RZ = RZ*180/PI
#     return [X, Y, Z, RX, RY, RZ]

def joint_deg_to_rad(joint_deg):
    joint_rad = [0]*6
    for i in range(6):
        joint_rad[i] = joint_deg[i]*PI/180
    return joint_rad

"""
UR5 kinematics Demo(FOR TEST)
"""

def main(args=None):
    a = URposition()
    # l1 = np.array([0, -PI/2, -PI/2, -PI/2, PI/2, 0])
    l1 = np.array([21.15, -94.59, -110.76, -156.41, 286.34, -91.47])
    l1 = joint_deg_to_rad(l1)
    print("l1关节表示：\n")
    print(l1)
    l2 = a.fwdkinematics_dh(l1)
    print("l1正运动学求出矩阵：\n")
    print(l2)
    print("矩阵转为RPY表示：\n")
    print(matrix_to_pose(l2))
    joint = l1
    a.invkinematics_dh(l2, joint)
    print("矩阵逆运动求解出关节：\n")
    print(joint)
    print("---------------------------------\n")

    # RPY = np.array([0.48740,-0.10922,0.43196,2.223,-2.224,-0.004])
    RPY = np.array([0.48144,0.04405,0.24782,0.267,-4.672,-0.115])
    print("RPY表示：\n")
    print(RPY)
    print("RPY转为旋转矩阵表示：\n")
    print(pose_to_matrix(RPY))
    a.invkinematics_dh(pose_to_matrix(RPY), joint)
    print("RPY矩阵逆运动求解出关节：\n")
    print(joint)

if __name__ == '__main__':
    main()