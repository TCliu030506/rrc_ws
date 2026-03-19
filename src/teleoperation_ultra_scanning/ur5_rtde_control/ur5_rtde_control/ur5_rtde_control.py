# 代码定义了一个名为 URCONTROL 的 Python 类，用于与 UR 机器人进行通信和控制。
# 该类封装了 rtde_control 和 rtde_receive 库的功能，提供了包括：
# “连接机器人、断开连接、发送 servoL 指令以及获取 TCP 姿态“等方法。

import rtde_control
import rtde_receive

class URCONTROL:
    def __init__(self,robot_ip):
        # Connect to the robot
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not self.rtde_c.isConnected():
            print("Failed to connect to the robot control interface.")
            return
        if not self.rtde_r.isConnected():
            print("Failed to connect to the robot receive interface.")
            return
        print("Connected to the robot.")
        # # Set TCP offset if needed（CYX）
        # self.rtde_c.setTcp([0.0, 0.0339, 0.208, 0.0, 0.0, 0.0])
        # # Set TCP offset if needed（LZH-平动）
        # self.rtde_c.setTcp([0.0, 0.0, 0.22, 0.0, 0.0, 0.0])
        # # Set TCP offset if needed（LZH-旋转）
        # self.rtde_c.setTcp([0.0, 0.0, 0.60, 0.0, 0.0, 0.0])
        # Set TCP offset if needed（默认测试）
        self.rtde_c.setTcp([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Define servoL parameters
        self.speed = 0.15  # m/s
        self.acceleration = 0.1  # m/s^2
        self.dt = 1.0/100  # dt for 500Hz, or 1.0/125 for 125Hz
        self.lookahead_time = 0.1  # s
        self.gain = 300  # proportional gain
        self.asynchronous = True  # whether to run servoL asynchronously

        
    def sevol_l(
        self,
        target_pose,
        speed: float | None = None,
        acceleration: float | None = None,
        dt: float | None = None,
        lookahead_time: float | None = None,
        gain: float | None = None,
    ):
        """servoL，控制参数可选。不传则使用对象默认值。"""
        speed = self.speed if speed is None else speed
        acceleration = self.acceleration if acceleration is None else acceleration
        dt = self.dt if dt is None else dt
        lookahead_time = self.lookahead_time if lookahead_time is None else lookahead_time
        gain = self.gain if gain is None else gain
        self.rtde_c.servoL(target_pose, speed, acceleration, dt, lookahead_time, gain)

    def movel(
        self,
        target_pose,
        speed: float | None = None,
        acceleration: float | None = None,
        asynchronous: bool | None = None,
    ):
        """moveL，控制参数可选。不传则使用对象默认值。"""
        speed = self.speed if speed is None else speed
        acceleration = self.acceleration if acceleration is None else acceleration
        asynchronous = self.asynchronous if asynchronous is None else asynchronous
        self.rtde_c.moveL(target_pose, speed, acceleration, asynchronous)

    def get_tcp_pose(self):
        return self.rtde_r.getActualTCPPose()
    
    def disconnect(self):
        if self.rtde_c:
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()
        print("已断开UR连接")


# example
# if __name__ == "__main__":
#     ur = URCONTROL("192.168.1.15")
#     target_pose = [0.437, -0.1, 0.846, -0.11019068574221307, 1.59479642933605, 0.07061926626169934]
    
#     ur.sevol_l(target_pose)

def main():
    ur = URCONTROL("192.168.1.102")
    initial_pose = [-0.17607630829367915, 0.17307133742923295, 0.583081120240345, -0.9636317184046123, -1.3850409964238102, 0.386835359716154]
    target_pose = initial_pose
    target_pose[3] = 0.0 # 设置目标姿态的第四个元素为0.0
    target_pose[4] = 0.0 # 设置目标姿态的第四个元素为0.0
    target_pose[5] = 0.0 # 设置目标姿态的第五个元素为0.0
    pose = ur.get_tcp_pose()
    print("初始的TCP姿态: \n", pose)
    ur.sevol_l(target_pose)
    pose = ur.get_tcp_pose()
    print("运动后的TCP姿态: \n", pose)

if __name__ == '__main__':

    main()

