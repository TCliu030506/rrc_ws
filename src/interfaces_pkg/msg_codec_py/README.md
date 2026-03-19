# 功能包介绍
提供消息编码解码功能的ROS2 Python功能包

## 使用方法
1. `from msg_codec_py.msg_coder import MsgCoder`
2. 设置MsgCoder支持的指令，支持的指令类型包括：`2s`, `b`, `q`, `d`等等，表示不同数据类型：
   `MsgCoder.cmd_script = {b'XX': {'cmd': '2s'}, b'LK': {'cmd': '2s', 'id': 'b'}, b'UN': {'cmd': '2s', 'id': 'b'}}`
   或者使用下面的方法逐条添加指令：
   `MsgCoder.add_cmd_script(b'MV', {'cmd': '2s', 'id': 'b', 'pos': 'q', 'vel': 'd'})`
3. 调用MsgCoder的msg_encode方法编码消息：
   `encoded_msg = MsgCoder.msg_encode(data_to_encode)`
4. 调用MsgCoder的msg_decode方法解码消息：
   `decoded_msg = MsgCoder.msg_decode(encoded_msg)`

## 测试方法
### 程序测试：
1. 修改msg_codec_py/msg_coder.py中主函数内示例数据：
   data_to_encode = {'cmd': b'MV', 'id': 6, 'pos': 1000, 'vel': 500}
2. 在功能包目录打开终端，运行：`$ python3 msg_coder.py`

### 节点测试：
1. 启动ROS2节点发布编码消息：  
   `ros2 run msg_codec_py msgpub_test`
2. 启动ROS2节点订阅编码消息：
   `ros2 run msg_codec_py msgsub_test`
3. 监听`encoded_msg`话题：
   `ros2 topic echo /encoded_msg std_msgs/msg/UInt8MultiArray`
4. 也可以将发布者或者订阅者的节点换成C++节点