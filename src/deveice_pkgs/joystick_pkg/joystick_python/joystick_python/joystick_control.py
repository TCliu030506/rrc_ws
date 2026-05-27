import rclpy
from rclpy.node import Node
from joystick_msg.msg import JoystickParameter
# from jyactuator_msg.srv import JyAction
# from mtactuator_msg.srv import MtAction
from inspire_pkg.inspire_python.inspire_python.inspire_client import Client_ins
from mtactuator_pkg.mtactuator_python.mtactuator_python.mtactuator_client import Client as Client_mt
from math import sqrt, atan2

JY_PARAMETER = 360
MT_GEAR_RATIO = 30*1.0/26
PI = 3.14159265359

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

        joystick_ins_position = sqrt(lx**2 + ly**2) / JY_PARAMETER
        joystick_mt_position = atan2(ly, lx) / PI * 180 / MT_GEAR_RATIO * 100

        if a == 1:  # Reset
            self.control_motor_position_ins(300)
            self.control_motor_position_mt_absolute(100, 0)
        else:
            if b == 1:
                self.is_locked = True
            elif x == 1:
                self.is_locked = False

            if lx != 0 or ly != 0:
                if not self.is_locked:
                    if lx > 2000 or ly > 2000:
                        self.control_motor_position_mt_absolute(100, joystick_mt_position)
                    self.control_motor_position_ins(800 + joystick_ins_position)
                    joystick_ins_position = 0
                    joystick_mt_position = 0
            else:
                if yy == 32767:
                    self.position_jy += 10
                    self.control_motor_position_ins(self.position_jy)
                    self.get_logger().info('YY_UP PRESSED!')
                elif yy == -32767:
                    self.position_jy -= 10
                    self.control_motor_position_ins(self.position_jy)
                    self.get_logger().info('YY_DOWN PRESSED!')
                elif xx == 32767:
                    self.position_mt = 1000
                    self.control_motor_position_mt_add(100, self.position_mt)
                    self.get_logger().info('XX_RIGHT PRESSED!')
                elif xx == -32767:
                    self.position_mt = -1000  # reverse rotation
                    self.control_motor_position_mt_add(100, self.position_mt)
                    self.get_logger().info('XX_LEFT PRESSED!')

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

