
#from x6_msg.srv import X6Action
import ast
import rclpy
import numpy as np
from rclpy.node import Node
import x6_kinematics.kinematics_X6 as X6
from msg_codec_cpp.srv import Script
from msg_codec_py.msg_coder import MsgCoder

class x6_clients(Node):
    def __init__(self):
        super().__init__('x6_client')
        self.x6_pf = x6_platform()
        self.X6_Kine = X6.X6_solver()
        self.client = self.create_client(Script, 'x6_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cmd_script = { b'ST': {'cmd': '2s'},
                            b'U6': {'cmd': '2s'},
                            b'L6': {'cmd': '2s'},
                            b'TS': {'cmd': '2s'},
                            b'M6': {'cmd': '2s', 'spd': 'q', 'px': 'd', 'py': 'd', 'pz': 'd', 'rx': 'd', 'ry': 'd', 'rz': 'd'},
        }

    def request(self, request: Script.Request):
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def set_zero(self):
        req = self.x6_pf.set_zero()
        self.request(req)

    def unlock(self):
        req = self.x6_pf.unlock()
        self.request(req)    
    
    def lock(self):
        req = self.x6_pf.lock()
        self.request(req)   

    def move_abs(self, maxspeed: int, px: float, py: float, pz: float, rx: float, ry: float, rz: float):
        req: Script.Request = self.x6_pf.move_abs(maxspeed, px, py, pz, rx, ry, rz)
        self.request(req)

    def test(self):
        req = self.x6_pf.test()
        self.request(req)

    def check_input(self,x6_input: np.array):
        """
        检查输入是否符合要求：[int/float,int/float,...] 共6个元素
        返回合法的list，否则返回 None
        """
        try:
            values = ast.literal_eval(x6_input)
        except Exception:
            return None

        if not (isinstance(values, list) and len(values) == 6 and all(isinstance(v, (int, float)) for v in values)):
            return None
        
        return values
    
class x6_platform():
    def __init__(self):
        self.cmd_script = { b'ST': {'cmd': '2s'},
                            b'U6': {'cmd': '2s'},
                            b'L6': {'cmd': '2s'},
                            b'TS': {'cmd': '2s'},
                            b'SP': {'cmd': '2s'},
                            b'M6': {'cmd': '2s', 'spd': 'q', 'px': 'd', 'py': 'd', 'pz': 'd', 'rx': 'd', 'ry': 'd', 'rz': 'd'},
        }
    def set_zero(self)-> Script.Request:
        """
        设定新的零位
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'ST',
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req

    def unlock(self)-> Script.Request:
        """
        掉电，无法接收运动指令，此时能且仅能手拧
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'U6',
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req
    
    def lock(self)-> Script.Request:
        """
        上电，恢复接收运动指令，仍可手拧，只在接受运动指令后锁住
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'L6',
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req
    
    def stop(self)-> Script.Request:
        """
        停止运动
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'SP',
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req

    def move_abs(self,maxspeed_: int, px_: float, py_: float, pz_: float, rx_: float, ry_: float, rz_: float)-> Script.Request:
        """
        绝对运动
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'M6',
            'spd': maxspeed_,
            'px': px_,
            'py': py_,
            'pz': pz_,
            'rx': rx_,
            'ry': ry_,
            'rz': rz_,
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req

    def test(self)-> Script.Request:
        """
        测试
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'TS',
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode, self.cmd_script)
        req.command.data = list(encoded_data)
        return req
 
   