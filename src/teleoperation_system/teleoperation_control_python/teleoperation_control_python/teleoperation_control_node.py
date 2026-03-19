import rclpy
import time
import math
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from xmate_cr7_msg.srv import Cr7Script
from sigema7_msg.msg import Sigema7Position

class TeleOperation(Node):
    callback_flag = 0
    telecontrol_thread_flag = 1
    xmate_pos = [0.0] * 6
    xmate_vel = 0.0
    lock = threading.Lock()  # 添加锁对象

    def __init__(self):
        super().__init__('teleoperation_node')

        # 创建服务客户端
        self.client = self.create_client(Cr7Script, 'xmate_cr7_server')
        # 等待服务器启动
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return
            self.get_logger().info('Service not available, waiting again...')

        # # 发送请求
        # self.send_request("open_rtControl_loop")
        # # self.send_request("follow_pos_start")
        # # 暂停3s，确保机械臂已经进入控制模式
        # time.sleep(3)  

        # 开启新的线程
        thread = threading.Thread(target=self.telecontrol_thread)
        thread.start()

        # 订阅话题
        self.subscription = self.create_subscription(
            Sigema7Position,
            'sigema7_getpos',
            self.topic_callback,
            80)

    def send_request(self, command, pose=[], speed=0, zone=0):
        # 确保服务可用
        if not self.client.service_is_ready():
            self.get_logger().warn('Service not available, waiting...')
            self.client.wait_for_service()
        # 创建请求
        request = Cr7Script.Request()
        request.command = command
        request.pose = pose
        request.speed = speed
        request.zone = zone
        # 测试command、pose、speed、zone的值
        self.get_logger().info(f'Sending request: command={command}, pose={pose}, speed={speed}, zone={zone}')

        # 异步发送请求
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # 测试是否发送成功
        if future.result() is not None:
            self.get_logger().info('Request sent successfully')

        if future.result() is not None:
            # 处理响应
            response = future.result()
            self.get_logger().info(f'Service response: {response.result}')
            if response.pose:
                self.get_logger().info('Pose response:')
                for val in response.pose:
                    self.get_logger().info(f'  {val}')
            if response.joint:
                self.get_logger().info('Joint response:')
                for val in response.joint:
                    self.get_logger().info(f'  {val}')
            if response.vel:
                self.get_logger().info('Velocity response:')
                for val in response.vel:
                    self.get_logger().info(f'  {val}')
            if response.acc > 0:
                self.get_logger().info(f'Acceleration response: {response.acc}')
            if response.jerk > 0:
                self.get_logger().info(f'Jerk response: {response.jerk}')
            if response.torque:
                self.get_logger().info('Torque response:')
                for val in response.torque:
                    self.get_logger().info(f'  {val}')
            return response  # 返回服务器响应
        else:
            self.get_logger().error('Failed to call service xmate_cr7_server')
            return None  # 调用失败返回 None

    def telecontrol_thread(self):
        while True:
            if self.callback_flag == 1:
                with self.lock:  # 获取锁
                    # 将机械臂的目标运动位置逆运动学求解得到关节目标位置
                    response = self.send_request("inv_kinematics", self.xmate_pos)
                    self.get_logger().info('完成逆运动学求解')
                    if response and hasattr(response, 'joint'):
                        xmate_joint = response.joint
                        # 打印机械臂的目标位置
                        self.get_logger().info(f'xmate_joint: {xmate_joint}')
                        # 更新机械臂的目标位置
                        # self.send_request("rtControl_pos_update", xmate_joint)
                    else:
                        self.get_logger().error('Failed to get joint positions from inverse kinematics.')
                    # 更新回调标志
                    self.callback_flag = 0
                    # self.telecontrol_thread_flag = 1
                    self.get_logger().info(f'{self.callback_flag}')
            # 以1000HZ频率运行
            time.sleep(0.001)

    def topic_callback(self, msg):
        # if self.telecontrol_thread_flag == 1:
            self.get_logger().info('Received message')
            # 提取话题里的消息
            sigma7_pos = msg.sigema7_pos
            sigema7_vel_linear = msg.sigema7_vel_linear
            sigema7_force = msg.sigema7_force
            self.get_logger().info('成功提取话题里的消息')
            # 打印提取的位置
            self.get_logger().info(f'sigma7_pos: {sigma7_pos}')
            self.get_logger().info(f'sigema7_vel_linear: {sigema7_vel_linear}')
            self.get_logger().info(f'sigema7_force: {sigema7_force}')

            # 将sigema7的数据映射到机械臂端
            with self.lock:  # 获取锁
                self.pos_mapping(sigma7_pos, self.xmate_pos)
                self.xmate_vel = self.vel_mapping(sigema7_vel_linear)
                self.get_logger().info('成功完成映射')
                # 打印机械臂的目标位置
                self.get_logger().info(f'xmate_pos: {self.xmate_pos}')
                self.get_logger().info(f'xmate_vel: {self.xmate_vel}')
                # 更新回调标志
                self.callback_flag = 1
                self.telecontrol_thread_flag = 0
            # 测试
            self.get_logger().info('成功完成话题订阅者回调')


    def pos_mapping(self, sigma7_pos, arm_pos):
        # 设置机械臂的默认位置和sigema7的默认位置
        arm_default_pos = [0.150,0.360,0.660,math.pi,0.0,-math.pi/2]
        sigma7_pos_default = [-0.006798,-0.000586,-0.003852,0.00278983,-0.00896451,0.0069999]
        mapping_ratio = 2.0
        mapping_rad_ratio = 0.9
        # 将sigema7的数据映射到机械臂端
        arm_pos[0] = arm_default_pos[0]+(sigma7_pos[0]-sigma7_pos_default[0])*mapping_ratio
        arm_pos[1] = arm_default_pos[1]+(sigma7_pos[1]-sigma7_pos_default[1])*mapping_ratio
        arm_pos[2] = arm_default_pos[2]+(sigma7_pos[2]-sigma7_pos_default[2])*mapping_ratio
        arm_pos[3] = arm_default_pos[3]+(sigma7_pos[3]-sigma7_pos_default[3])*mapping_rad_ratio
        arm_pos[4] = arm_default_pos[4]+(sigma7_pos[4]-sigma7_pos_default[4])*mapping_rad_ratio
        arm_pos[5] = arm_default_pos[5]+(sigma7_pos[5]-sigma7_pos_default[5])*mapping_rad_ratio

    def vel_mapping(self, sigma7_vel):
        # 定义映射比例
        mapping_ratio = 6.0
        # 将sigema7的速度映射到机械臂端
        vel_mapping = [0] * len(sigma7_vel)
        vel_mapping[0] = sigma7_vel[0]*mapping_ratio*1000
        vel_mapping[1] = sigma7_vel[1]*mapping_ratio*1000
        vel_mapping[2] = sigma7_vel[2]*mapping_ratio*1000
        # 求出速度的大小
        arm_vel  = math.sqrt(vel_mapping[0]**2+vel_mapping[1]**2+vel_mapping[2]**2)
        return arm_vel


def main(args=None):
    rclpy.init(args=args)
    teleoperation = TeleOperation()
    executor = MultiThreadedExecutor(4)
    executor.add_node(teleoperation)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        teleoperation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
