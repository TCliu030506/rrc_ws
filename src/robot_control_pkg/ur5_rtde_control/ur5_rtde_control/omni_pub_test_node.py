import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState
from geometry_msgs.msg import PoseStamped
import math


def _quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / n, y / n, z / n, w / n]


def _quat_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_inv(q):
    # unit quaternion inverse
    return _quat_conjugate(q)


def _quat_mul(q1, q2):
    # Hamilton product; matches composition like: R(q1) * R(q2)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quat_from_axis_angle(axis, theta):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    ax /= n
    ay /= n
    az /= n
    half = 0.5 * float(theta)
    s = math.sin(half)
    c = math.cos(half)
    return [ax * s, ay * s, az * s, c]


def _quat_to_rotvec(q):
    # ensure unit
    q = _quat_normalize(q)
    x, y, z, w = q

    # choose canonical sign to keep angle <= pi (helps reduce output jumps)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    v_norm = math.sqrt(x * x + y * y + z * z)
    if v_norm < 1e-12:
        return [0.0, 0.0, 0.0]

    theta = 2.0 * math.atan2(v_norm, w)
    ax = x / v_norm
    ay = y / v_norm
    az = z / v_norm
    return [ax * theta, ay * theta, az * theta]


def _rotvec_to_quat(rvec):
    rx, ry, rz = rvec
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    ax = rx / theta
    ay = ry / theta
    az = rz / theta
    return _quat_from_axis_angle([ax, ay, az], theta)


def _twist_angle_about_axis_from_quat(delta_q, axis):
    """从增量旋转四元数 delta_q 中提取绕给定 axis 的 twist（有符号角度，rad）。

    axis 表达在 delta_q 所在的坐标系内。
    """
    delta_q = _quat_normalize(delta_q)
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return 0.0
    ax /= n
    ay /= n
    az /= n

    vx, vy, vz, w = delta_q
    # 向量部在 axis 上投影
    proj = vx * ax + vy * ay + vz * az
    vpx, vpy, vpz = ax * proj, ay * proj, az * proj
    qt = _quat_normalize([vpx, vpy, vpz, w])

    # qt 对应纯 twist：v = axis*sin(theta/2), w=cos(theta/2)
    sin_half = qt[0] * ax + qt[1] * ay + qt[2] * az
    cos_half = qt[3]
    return 2.0 * math.atan2(sin_half, cos_half)


def _twist_decompose_xyz_from_quat(delta_q):
    """将增量旋转 delta_q 近似分解为绕 x/y/z 三轴的 twist + 残差。

    分解形式（按顺序）：
        delta_q = qx * qy * qz * q_residual

    其中 qx/qy/qz 分别为绕参考坐标系 x/y/z 轴的 twist 四元数；
    q_residual 为剩余旋转（不做缩放时可保留以尽量保持原始姿态）。

    注意：这不是严格的欧拉角分解（因此不会出现典型的万向节锁），
    但它是一种连续、可用于“分轴调手感”的近似分解。
    """
    q = _quat_normalize(delta_q)

    ax_x = [1.0, 0.0, 0.0]
    ax_y = [0.0, 1.0, 0.0]
    ax_z = [0.0, 0.0, 1.0]

    # 依次提取并剥离 x/y/z twist
    ang_x = _twist_angle_about_axis_from_quat(q, ax_x)
    qx = _quat_from_axis_angle(ax_x, ang_x)
    q = _quat_mul(_quat_inv(qx), q)

    ang_y = _twist_angle_about_axis_from_quat(q, ax_y)
    qy = _quat_from_axis_angle(ax_y, ang_y)
    q = _quat_mul(_quat_inv(qy), q)

    ang_z = _twist_angle_about_axis_from_quat(q, ax_z)
    qz = _quat_from_axis_angle(ax_z, ang_z)
    q = _quat_mul(_quat_inv(qz), q)

    q_residual = _quat_normalize(q)
    return ang_x, ang_y, ang_z, q_residual


class Subscriber(Node):

    def __init__(self):
        super().__init__('phantom_pose_rxryrz_publisher')
        self.subscription = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.handle_msg,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/phantom/pose_rxryrz',
            10
        )

    def handle_msg(self, msg: OmniState):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.pose.position.x / 1000.0
        pose.pose.position.y = msg.pose.position.y / 1000.0
        pose.pose.position.z = msg.pose.position.z / 1000.0

        q = _quat_normalize([
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ])
        rx, ry, rz = _quat_to_rotvec(q)
        rx_deg = math.degrees(float(rx))
        ry_deg = math.degrees(float(ry))
        rz_deg = math.degrees(float(rz))

        # 将 rx/ry/rz（角度制）写入 orientation.x/y/z，w 置 0
        pose.pose.orientation.x = rx_deg
        pose.pose.orientation.y = ry_deg
        pose.pose.orientation.z = rz_deg
        pose.pose.orientation.w = 0.0

        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    master_sub = Subscriber()
    rclpy.spin(master_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()