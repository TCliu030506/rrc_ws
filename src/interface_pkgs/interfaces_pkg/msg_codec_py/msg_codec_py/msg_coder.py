from typing import Dict, Any
import struct

"""
MsgCoder: 将字典编码为一条 UTF-8 字符串并解码回字典。

用法概述:
- 定义不同指令的字典字段集合，如 cmd_move, cmd_lock 等
- 定义msg_encode内对应指令的编码规则
- 定义msg_decode内对应指令的解码规则
"""

class MsgCoder:
    # 定义支持的指令类型及其标签字段
    cmd_script = {b'XX': {'cmd': '2s'}}

    @classmethod
    def add_cmd_script(cls, cmd: bytes, format_dict: Dict[str, str]):
        """
        添加一条指令的编码规则, 如：
        add_cmd_script(b'MV', {'cmd': '2s', 'id': 'b', 'pos': 'q', 'vel': 'q'})
        参考类型表:
        'b': int (1 byte)
        'h': int (2 bytes)    
        'i': int (4 bytes)
        'q': int (8 bytes)
        'f': float (4 bytes)
        'd': double (8 bytes) 
        's': char[] (string, variable length)
        '2s': bytes (2 bytes)
        """
        cls.cmd_script[cmd] = format_dict

    @classmethod
    def msg_encode(cls, data: Dict[str, Any], cmd_script_: Dict[bytes, Dict[str, str]] = None) -> bytes:
        """
        data: 必须包含 'cmd'字段，以便对应相应指令，其它字段可选。
        cmd_script_: 默认为None，若传入编码参数，则按此参数编码，示例:
        {   b'XX': {'cmd': '2s'}, 
            b'LK': {'cmd': '2s', 'id': 'b'}}
        返回: UTF-8 编码的单条字符串
        """
        if cmd_script_ is None:
            cmd_script = cls.cmd_script
        else:
            cmd_script = cmd_script_

        if 'cmd' not in data:
            raise ValueError("'cmd' field is required")
        cmd = data['cmd']
        if cmd not in cmd_script:
            raise ValueError(f"Unknown cmd: {cmd}")
        
        cmd_format: Dict[str, str] = cmd_script[cmd]
        format_str = '<' + ''.join(cmd_format.values())

        # 打包数据为二进制
        msg_vals = []
        for key in cmd_format.keys():
            if key not in data:
                raise ValueError(f"Missing required field: {key}")
            msg_vals.append(data[key])
        msg: bytes = struct.pack(format_str, *msg_vals)
        
        return msg
        
    @classmethod
    def msg_decode(cls, msg: bytes, cmd_script_: Dict[bytes, Dict[str, str]] = None) -> Dict[str, Any]:
        """
        解析一条编码后的消息字符串
        cmd_script_: 默认为None，若传入编码参数，则按此参数解码，示例:
        {   b'XX': {'cmd': '2s'}, 
            b'LK': {'cmd': '2s', 'id': 'b'}}
        根据msg内容, 返回包含 'cmd'等字段的字典。
        """
        if cmd_script_ is None:
            cmd_script = cls.cmd_script
        else:
            cmd_script = cmd_script_

        if len(msg) < 2:
            raise ValueError("msg too short")
        
        cmd = msg[:2]
        if cmd not in cmd_script:
            raise ValueError(f"Unknown cmd: {cmd}")
        
        cmd_format: Dict[str, str] = cmd_script[cmd]
        format_str = '<' + ''.join(cmd_format.values())

        msg_vals = struct.unpack(format_str, msg)

        keys_list = list(cmd_format.keys())
        if len(keys_list) != len(msg_vals):
            raise ValueError(f"unmatch field count: expect {len(keys_list)}, but got {len(msg_vals)}")
        # 解析回字典
        data: Dict[str, Any] = {}
        for key, val in zip(keys_list, msg_vals):
            data[key] = val
        
        return data
        # 示例用法:
        #访问字典字段： id_val = data['id']

if __name__ == "__main__":

    # 定义指令类型与标签字段的映射, 根据cmd判断指令类型

    # 直接设置所有指令的编码规则
    MsgCoder.cmd_script = {b'XX': {'cmd': '2s'}, 
                           b'LK': {'cmd': '2s', 'id': 'b'},
                           b'UN': {'cmd': '2s', 'id': 'b'}}
   
    # 添加新的指令编码规则
    MsgCoder.add_cmd_script(b'MV', {'cmd': '2s', 'id': 'b', 'pos': 'q', 'vel': 'q'})
    
    # 示例数据
    data_to_encode = {
        'cmd': b'MV',
        'id': 6,
        'pos': 1000,
        'vel': 500
    }

    try:
        # 测试编码
        encoded_msg = MsgCoder.msg_encode(data_to_encode)
        print(f"Encoded Message: {encoded_msg}")

        # 测试解码
        decoded_msg = MsgCoder.msg_decode(encoded_msg)
        print(f"Decoded Message: {decoded_msg}")

    except ValueError as e:
        print(f"Error: {e}")


