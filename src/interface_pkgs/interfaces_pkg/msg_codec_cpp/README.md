# 功能包介绍
提供消息编码解码功能的ROS2 C++功能包

## 内置服务
- Script.srv
- 采用std_msgs/UInt8MultiArray消息类型发送指令

## 使用方法
1. `#include "msg_codec_cpp/msg_coder.h"`
2. 设置MsgCoder支持的指令，支持的指令类型包括：
   MsgCoder::ValType::CHAR2, MsgCoder::ValType::INT16, MsgCoder::ValType::DOUBLE等等，表示不同数据类型。
   - 添加多条指令
      `MsgCoder::registerCommands({ 
        {std::array<char, 2>{'C', 'F'}, {{"cmd", CHAR2}, {"id", INT8}}}, 
        {std::array<char, 2>{'X', 'Y'}, {{"cmd", CHAR2}, {"id", INT8}, {"status", INT8}}} `
      });`
   - 逐条添加指令：
      `MsgCoder::registerCommand(std::string("MV"), {{"cmd", CHAR2}, {"id", INT8}, {"pos", INT64}, {"vel", DOUBLE}});`
3. 调用MsgCoder的msg_encode方法编码消息：
   `auto encoded_msg = MsgCoder::msg_encode(data_to_encode)`
4. 调用MsgCoder的msg_decode方法解码消息：
   `auto decoded_msg = MsgCoder::msg_decode(encoded_msg)`

## 测试方法
### 节点测试：
1. 启动ROS2节点发布编码消息：  
   `ros2 run msg_codec_cpp msg_publisher`
2. 启动ROS2节点订阅编码消息：
   `ros2 run msg_codec_cpp msg_subscriber`
3. 监听`encoded_msg`话题：
   `ros2 topic echo /encoded_msg std_msgs/msg/UInt8MultiArray`
4. 也可以将发布者或者订阅者的节点换成Python节点