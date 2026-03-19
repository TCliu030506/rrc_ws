import rclpy
import rclpy.clock
import math
from rclpy.node import Node
from std_msgs.msg import String

##  movej(self, joint_positions,joint_parameters) 函数通过输入给定机械臂的关节位置和运动参数
##  joint_positions 含有六个弧度制的关节位置
##  a: joint acceleration of leading axis [rad/s^2]
##  v: joint speed of leading axis [rad/s]
##  t: time [S]
##  r: blend radius

class URNode(Node):
    def __init__(self):
        super().__init__('ur5_client_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

    def movej(self, joint_positions, a=None, v=None, t=None,r=None):
        """
        Controls the UR5 to move in joint space.

        Parameters:
        joint_positions (list): A list containing six joint positions to specify the target joint positions [rad].
        a (float, optional): Joint acceleration of leading axis [rad/s^2] (default is 0.2).
        v (float, optional): Joint speed of leading axis [rad/s] (default is 0.2).
        t (float, optional): Time [s] (default is 0).
        r (float, optional): Blend radius [m] (default is 0).

        Returns:
        None

        Examples:
        movej([0, 0, 0, 0, 0, 0]) # Moves to joint positions [0, 0, 0, 0, 0, 0] #using default acceleration and speed.
        movej([0, -1.57, -1.57, -1.57, 0, 1.57], a=0.5, v=0.3) # Moves to joint positions [0, -1.57, -1.57, -1.57, 0, 1.57] using specified acceleration and speed.
        movej([0, -1.57, -1.57, -1.57, 0, 1.57], 0.5, 0.3, 2, 0.1) # Moves to joint positions [0, -1.57, -1.57, -1.57, 0, 1.57] using specified all parameters.
        """

        default_a = 0.2
        default_v = 0.2
        default_t = 0
        default_r = 0
        a = a if a is not None else default_a
        v = v if v is not None else default_v
        t = t if t is not None else default_t
        r = r if r is not None else default_r
        
        msg = String()
        msg.data = f"movej({joint_positions},{a},{v},{t},{r})"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def movel(self, eulur_pos, a=None, v=None, t=None,r=None):
        """
        Controls the UR5 to move  (linear in tool-space).

        Parameters:
        joint_positions (list): 欧拉角表示(角度为[rad]).
        a (float, optional): Joint acceleration of leading axis [rad/s^2] (default is 0.2).
        v (float, optional): Joint speed of leading axis [rad/s] (default is 0.2).
        t (float, optional): Time [s] (default is 0).
        r (float, optional): Blend radius [m] (default is 0).

        Returns:
        None

        Examples:
        movel([0.2, 0.3, 0.5, 0, 0, 3.14]) # 运动到工具坐标系x=0.2m y=0.3m z=0.5m rx=0 ry=0 rz=3.14 #using default acceleration and speed.
        movel([0.2, 0.3, 0.5, 0, 0, 3.14], a=0.5, v=0.3) # 运动到工具坐标系x=0.2m y=0.3m z=0.5m rx=0 ry=0 rz=3.14 using specified acceleration and speed.
        movel([0.2, 0.3, 0.5, 0, 0, 3.14], 0.5, 0.3, 2, 0.1) # 运动到工具坐标系x=0.2m y=0.3m z=0.5m rx=0 ry=0 rz=3.14 using specified all parameters.
        """

        default_a = 0.1
        default_v = 0.1
        default_t = 0
        default_r = 0
        a = a if a is not None else default_a
        v = v if v is not None else default_v
        t = t if t is not None else default_t
        r = r if r is not None else default_r
        
        msg = String()
        msg.data = f"movel(p{eulur_pos},{a},{v},{t},{r})"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def servoj(self, joint_positions, a = None, v = None, t = None,lookahead_time = None, gain = None):

        default_a = 0.1
        default_v = 0.05
        default_t = 0.008
        default_lookahead_time = 0.1
        default_gain = 300
        a = a if a is not None else default_a
        v = v if v is not None else default_v
        t = t if t is not None else default_t
        lookahead_time = lookahead_time if lookahead_time is not None else default_lookahead_time
        gain = gain if gain is not None else default_gain
        
        msg = String()
        msg.data = f"servoj({joint_positions},{a},{v},{t},{lookahead_time},{gain})"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    # 没有实现，暂时不可用
    def get_inverse_kin(self,eulur_pos, qnear = None,maxPositionError = None, maxOrientationError = None):

        default_maxPositionError = 1e-9
        default_maxOrientationError = 1e-9
        default_qnear = [0,-90,-90,-90,90,0]
        qnear = qnear if qnear is not None else default_qnear
        maxPositionError = maxPositionError if maxPositionError is not None else default_maxPositionError
        maxOrientationError = maxOrientationError if maxOrientationError is not None else default_maxOrientationError
        
        msg = String()
        msg.data = f"get_inverse_kin(p{eulur_pos},{qnear},{maxPositionError},{maxOrientationError})"
        joint_position = self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        return joint_position

    def stop(self):
        """
        Controls the robot arm to stop.
        """
        msg = String()
        msg.data = "stopj(20)"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


##  rad_to_deg() 弧度转换为角度  
def rad_to_deg(radian):
    degree = radian * (180 / math.pi)
    return degree

##  deg_to_rad() 角度转换为弧度
def deg_to_rad(degree):
    radian = degree * (math.pi / 180)
    return radian


