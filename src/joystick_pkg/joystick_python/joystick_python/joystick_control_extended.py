import rclpy
import math
from rclpy.node import Node
from joystick_msg.msg import JoystickParameter
# from jyactuator_msg.srv import JyAction
# from mtactuator_msg.srv import MtAction
from inspire_python.inspire_client import Client as Client_ins
from ur5_control.URControl import URNode
from mtactuator_python.mtactuator_client import Client as Client_mt
from math import sqrt, atan2

JY_PARAMETER = 360
MT_GEAR_RATIO = 30*1.0/26
PI = 3.14159265359

#此处修改需要直线运动的各个目标定点(欧拉角表示)  要根据实际需要位置修改！
position_ur=[[0.19333,-0.16368,0.41335,1.384,1.226,1.130],
             [0.35395,-0.17518,0.34463,1.384,1.226,1.130]]   #基座坐标系 位姿表示 （x，y，z，rx,ry,yz) 单位：mm、rad

count_y = 0 #用于记录y按键的次数
count_x = 0 #用于记录y按键的次数

class JoystickControlClient(Node):
    def __init__(self):
        super().__init__('joystick_control_client')
        self.joystick_subscriber = self.create_subscription(
            JoystickParameter,
            'joystick_input',
            self.joystick_input_callback,
            80)
        # self.motor_service_client_jy = self.create_client(JyAction, 'jyactuator_srv')
        # self.motor_service_client_mt = self.create_client(MtAction, 'mtactuator_srv')
        self.ur_node = URNode() 
        
        self.position_jy = 800
        self.position_mt = 1000
        self.is_locked = False

    def joystick_input_callback(self, msg):
        xx = msg.xx
        yy = msg.yy
        lx = msg.lx
        ly = msg.ly
        a = msg.a
        b = msg.b
        x = msg.x
        y = msg.y  #用于执行直线命令
        lb = msg.lb   #代表调整机械臂运动
        rb = msg.rb   #代表调整末端连续体运动
        lt = msg.lt   
        rt = msg.rt

        mode = 0   # 0代表机械臂模式，1代表连续体模式
        global count_y
        global count_x
        global position_ur

        if rb == 1:
            mode = 1

        if lb == 1:
            mode = 0

        if mode == 0:
            
            ##按b键，逐次执行直线运动至指定点，最多执行len(POSITION_UR)次
            if b == 1:
                if count_y < len(position_ur):
                    self.position_ur=position_ur[count_y]
                    self.ur_node.movel(self.position_ur)
                    count_y = count_y+1
                    self.get_logger().info(f'LinearMotion {count_y}')
                if len(position_ur) <= count_y <= len(position_ur)+3:
                    count_y = count_y+1
                    self.get_logger().info('LinearMotion already done')

                if count_y > len(position_ur)+3:
                    self.position_ur[0] = self.position_ur[0] - (0.33492-0.35464)*0.5
                    self.position_ur[1] = self.position_ur[1] - (-0.17037+0.17520)*0.5
                    self.position_ur[2] = self.position_ur[2] - (0.22856-0.34458)*0.5
                    self.ur_node.movel(self.position_ur)
                    self.get_logger().info('PULL OUT')  
            
            ##XYZ微调，每按一下，末端移动1mm
            if xx == -32767:
                self.position_ur[0] = self.position_ur[0]-0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('X move 1mm (LEFT)')

            if xx == 32767:
                self.position_ur[0] = self.position_ur[0]+0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('X move 1mm (RIGHT)')

            if yy == -32767:
                self.position_ur[1] = self.position_ur[1]-0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('Y move 1mm (BACK)')    

            if yy == 32767:
                self.position_ur[1] = self.position_ur[1]+0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('Y move 1mm (FORWARD)')  

            if a == 1:
                self.position_ur[2] = self.position_ur[2]- 0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('Z move 1mm (DOWN)')  
            
            if y == 1:
                self.position_ur[2] = self.position_ur[2]+ 0.001
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('Z move 1mm (DOWN)')  

            if lt > 10000:
                self.position_ur[3] = self.position_ur[3]-0.0015
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RX rotate 1 degree (-)')  

            if rt > 10000:
                self.position_ur[3] = self.position_ur[3]+0.0015
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RX rotate 1 degree (+)')  

            if lx < -32000:
                self.position_ur[4] = self.position_ur[4]-0.0008
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RY rotate 1 degree (-)')  
            
            if lx > 32000:
                self.position_ur[4] = self.position_ur[4]+0.0008
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RY rotate 1 degree (+)')  

            if ly < -32000:
                self.position_ur[5] = self.position_ur[5]-0.0008
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RZ rotate 1 degree (-)')  
            
            if ly > 32000:
                self.position_ur[5] = self.position_ur[5]+0.0008
                self.ur_node.movel(self.position_ur)
                self.get_logger().info('RZ rotate 1 degree (+)')  

            if x == 1:
                if count_x < 1:
                    self.position_ur[0] = self.position_ur[0]+ (0.33492-0.35464)*0.75
                    self.position_ur[1] = self.position_ur[1]+ (-0.17037+0.17520)*0.75
                    self.position_ur[2] = self.position_ur[2]+ (0.22856-0.34458)*0.75
                    self.ur_node.movel(self.position_ur)
                    self.get_logger().info('PULL IN')  
                    count_x = count_x+1
                else:
                    self.position_ur[0] = self.position_ur[0]+ (0.33492-0.35464)*0.05
                    self.position_ur[1] = self.position_ur[1]+ (-0.17037+0.17520)*0.05
                    self.position_ur[2] = self.position_ur[2]+ (0.22856-0.34458)*0.05
                    self.ur_node.movel(self.position_ur)
                    self.get_logger().info('PULL IN (Slightly)')  
                    count_x = count_x+1
            
        else:
            self.get_logger().info('TEST FOR CONTINUEN') ##测试用

            # joystick_ins_position = sqrt(lx**2 + ly**2) / JY_PARAMETER
            # joystick_mt_position = atan2(ly, lx) / PI * 180 / MT_GEAR_RATIO * 100

            # if a == 1:  # Reset
            #     self.control_motor_position_ins(300)
            #     self.control_motor_position_mt_absolute(100, 0)
            # else:
            #     if b == 1:
            #         self.is_locked = True
            #     elif x == 1:
            #         self.is_locked = False

            #     if lx != 0 or ly != 0:
            #         if not self.is_locked:
            #             if lx > 2000 or ly > 2000:
            #                 self.control_motor_position_mt_absolute(100, joystick_mt_position)
            #             self.control_motor_position_ins(800 + joystick_ins_position)
            #             joystick_ins_position = 0
            #             joystick_mt_position = 0
            #     else:
            #         if yy == 32767:
            #             self.position_jy += 10
            #             self.control_motor_position_ins(self.position_jy)
            #             self.get_logger().info('YY_UP PRESSED!')
            #         elif yy == -32767:
            #             self.position_jy -= 10
            #             self.control_motor_position_ins(self.position_jy)
            #             self.get_logger().info('YY_DOWN PRESSED!')
            #         elif xx == 32767:
            #             self.position_mt = 1000
            #             self.control_motor_position_mt_add(100, self.position_mt)
            #             self.get_logger().info('XX_RIGHT PRESSED!')
            #         elif xx == -32767:
            #             self.position_mt = -1000  # reverse rotation
            #             self.control_motor_position_mt_add(100, self.position_mt)
            #             self.get_logger().info('XX_LEFT PRESSED!')

            

    def control_motor_position_ins(self, position):
        motor = Client_ins(3)
        motor.move_vel(position, 100)
        '''
        request = JyAction.Request()
        request.pos = position
        request.tim = 2.0
        self.motor_service_client_jy.call_async(request)
        '''

    def control_motor_position_mt_add(self, max_speed, position):
        motor = Client_mt(1)
        motor.move_angle_add(max_speed, position)
        '''
        request = MtAction.Request()
        request.mode = 1
        request.maxspeed = max_speed
        request.pos = position
        self.motor_service_client_mt.call_async(request)
        '''

    def control_motor_position_mt_absolute(self, max_speed, position):
        motor =Client_mt(1)
        motor.move_angle_absolute(max_speed, position)
        '''
        request = MtAction.Request()
        request.mode = 3
        request.maxspeed = max_speed
        request.pos = position
        self.motor_service_client_mt.call_async(request)
        '''

def main(args=None):
    rclpy.init(args=args)
    joystick_control_client = JoystickControlClient()
    rclpy.spin(joystick_control_client)
    joystick_control_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

