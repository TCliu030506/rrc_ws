
from lkactuator_msg.srv import LkAction
from msg_codec_cpp.srv import Script
from msg_codec_py.msg_coder import MsgCoder
from std_msgs.msg import UInt8MultiArray

class LK_Motor():

    def __init__(self, id: int) -> None:
        super().__init__()
        self.id = int(id)
        MsgCoder.cmd_script = { b'LK': {'cmd': '2s', 'id': 'b'},
                                b'UN': {'cmd': '2s', 'id': 'b'},
                                b'SP': {'cmd': '2s', 'id': 'b'},
                                b'MS': {'cmd': '2s', 'id': 'b', 'spd': 'q'},
                                b'MA': {'cmd': '2s', 'id': 'b', 'spd': 'q', 'pos': 'q'},
                                b'MB': {'cmd': '2s', 'id': 'b', 'spd': 'q', 'pos': 'q'},
                                b'RA': {'cmd': '2s', 'id': 'b'},
                                b'RP': {'cmd': '2s', 'id': 'b', 'num': 'b'}
        }

    def move_angle_add(self, maxspeed_: int, pos_: int) -> Script.Request:
        """
        增量模式 
        """ 
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'MA',
            'id': self.id,
            'spd': maxspeed_,
            'pos': pos_
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req
    
    def move_speed(self, maxspeed_: int) -> Script.Request :
        """
        速度控制模式
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'MS',
            'id': self.id,
            'spd': maxspeed_
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req

    def move_angle_absolute(self, maxspeed_: int, pos_: int) -> Script.Request:
        """
        多圈模式
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'MB',
            'id': self.id,
            'spd': maxspeed_,
            'pos': pos_
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req
        
    def lock(self) -> Script.Request:
        """
        上锁，无法移动
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'LK',
            'id': self.id,
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req

    def unlock(self) -> LkAction.Request:
        """
        解锁，可移动
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'UN',
            'id': self.id,
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req
    
    def stop(self) -> LkAction.Request:
        """
        停止运动
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'SP',
            'id': self.id,
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req
    
    def read_angle(self) -> LkAction.Request:
        """
        读取多圈绝对角度
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'RA',
            'id': self.id,
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req

    def read_control_parameter(self, number: int) -> LkAction.Request:
        """
        读取控制参数
        """
        req = Script.Request()  
        data_to_encode = {
            'cmd': b'RP',
            'id': self.id,
            'num': number
        }
        encoded_data = MsgCoder.msg_encode(data_to_encode)
        req.command.data = list(encoded_data)
        return req
    
