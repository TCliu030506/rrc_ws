#include "../include/cybergear.h"

namespace cybergear {

#define CAN_FRAME_HEADER1 0x41
#define CAN_FRAME_HEADER2 0x54
#define CAN_FRAME_TAIL1   0x0D
#define CAN_FRAME_TAIL2   0x0A

#define CAN_WAIT rclcpp::sleep_for(std::chrono::milliseconds(5))

#define REPORT_INFO 1             //打印发送帧信息
#define REPORT_MSG  0             //打印接收帧信息
#define SHOW_ERROR  0             //打印错误信息

static constexpr size_t CAN_FRAME_LEN = 17;  // 帧长度

#define MAX_ID 32                 

cybergear::cybergear() : Node("cybergear_actuator")  // 节点名称为 cybergear_actuator
{ 
  // 初始化
  //定义参数

  this->declare_parameter<int>("pub_rate", 1);            // 默认发布频率为 1Hz
  loop_rate = this->get_parameter("pub_rate").as_int();   // 读取参数服务器中的发布频率(.../para/cybergear_server_node.yaml)
  RCLCPP_INFO(get_logger(), "Loop_rate: %d", loop_rate);

  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");  // 默认串口号为 /dev/ttyUSB0
  serial_port = this->get_parameter("port").as_string();         // 读取参数服务器中的串口号
  RCLCPP_INFO(get_logger(), "Serial port: %s", serial_port.c_str());

  // 将每个 64 位整数 ID 转换为 8 位无符号整数（uint8_t）
  this->declare_parameter<std::vector<int64_t>>("id", std::vector<int64_t>{0x01});    // 默认id为 0x01
  std::vector<int64_t> id__ = this->get_parameter("id").as_integer_array();
  id.clear();
  std::stringstream ss;
  ss << "id: ";
  for(size_t i = 0; i < id__.size(); i++){
    id.push_back(static_cast<uint8_t>(id__[i]));
    ss << "0x" << std::setw(2) << std::setfill('0') << std::hex << id__[i];
    if(i != id__.size()-1) ss << ", ";
  }
  ss << ".";
  // 构造一条日志字符串ss打印,在启动时看到当前加载了哪些电机 ID
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  // 定义发布者和服务
  topic_pub = "cybergear_state";   // 发布的话题名称
  serve_pub = "cybergear_srv";     // 服务的名称

  srv_pub = create_service<cybergear_msg::srv::CybergearScript>(serve_pub, 
                std::bind(&cybergear::serCallback, this, std::placeholders::_1, std::placeholders::_2));  // 创建服务,绑定服务回调函数
  state_pub = this->create_publisher<cybergear_msg::msg::CybergearState>(topic_pub, 10);                 // 创建发布者，发布话题为 cybergear_state，队列大小为 10
  //  用定时器另开一个线程，专门发布消息，避免占用
  timer = this->create_wall_timer(
    std::chrono::milliseconds(1000 / loop_rate),
    std::bind(&cybergear::publish_state, this)
  );                                      // 创建定时器，周期为 1000/loop_rate 毫秒，绑定发布状态函数

  is_srv_on = false;                      // 表示当前是否正在处理服务请求。用于防止在服务执行时进行定时发布。

  msg_index = MAX_ID;                     // 当前要发布的电机 ID 索引（MAX_ID 表示无效）。
  aquire_index = 0;                       // 当前轮询读取的电机索引，用于循环读取每个电机的状态

  // 初始化状态存储数组
  angle_now.clear();      // 每个电机的当前位置
  vel_now.clear();      // 每个电机的当前速度值
  torque_now.clear();    // 每个电机的当前受力值
  temp_now.clear();     // 每个电机的当前温度值
  for(unsigned long i=0; i<id.size(); i++){
    angle_now.push_back(0);
    vel_now.push_back(0);
    torque_now.push_back(0);
    temp_now.push_back(0);
  }

  // 初始化消息模板
  motor_data.header.stamp = now();
  motor_data.header.frame_id = "cybergear_actuator";
  motor_data.id.clear();
  motor_data.angle.clear();
  motor_data.velocity.clear();
  motor_data.torque.clear();
  motor_data.temperature.clear();

  serialport_init();
}

cybergear::~cybergear() { }

bool cybergear::serialport_init()
{
  try {
    //设置串口属性，并打开串口 
    ser.setPort(serial_port); 
    ser.setBaudrate(921600); 
    serial::Timeout to = serial::Timeout::simpleTimeout(100); 
    ser.setTimeout(to);
    ser.open(); 
  } catch (serial::IOException &e) { 
    RCLCPP_ERROR(get_logger(), "Unable to open port %s.", serial_port.c_str());
    return false; 
  } 
  //检测串口是否已经打开，并给出提示信息 
  if (ser.isOpen()) { 
    RCLCPP_INFO(get_logger(), "Serial Port: %s is open!", serial_port.c_str());
  } else { 
    return false; 
  }

  //清空串口数据
  ser.flush();
  for(unsigned long i=0;i<id.size();i++){
    cybergear_send_read_req(id.at(i));  // 发送读取请求
    cybergear_poll_and_parse_once();    // 立即尝试读取并解析一帧
    RCLCPP_INFO(get_logger(), 
        "Initial position for motor id%c: %.2f.", 
        char('0' + (id.at(i) % 10)), angle_now[i]);
  }

  RCLCPP_INFO(get_logger(), "ROS_SRV: ros2 service call /%s cybergear_msg/srv/CybergearScript \"{command: ' ', id: , data: [ ]}\"", serve_pub.c_str());
  RCLCPP_INFO(get_logger(), "command: W-唤醒, E-使能, Z-设置当前位置为零位（仅在失能状态生效）, S-速度位置控制, T-力控模式, R-回零位, D-失能");

  return true;
}

bool cybergear::serCallback(
  const std::shared_ptr<cybergear_msg::srv::CybergearScript::Request> req,
  const std::shared_ptr<cybergear_msg::srv::CybergearScript::Response> res)
{
  is_srv_on = true;
  ser.flush();
  res->result = "OK";
  // 支持命令
  // W-唤醒, 
  // E-使能, 
  // Z-设置当前位置为零位（仅在失能状态生效）, 
  // S-速度位置控制, 
  // T-力控模式, 
  // R-回零位, 
  // D-失能
  switch (req->command.c_str()[0]) {

    case 'W':  
      // 串口唤醒
      send_wake();
      break;

    case 'E':  
      // 电机使能
      send_enable(req->id);
      break;

    case 'Z':  
      // 设置当前位置为零位
      send_set_current_zero(req->id);
      break;

    case 'S':  
      // 速度位置控制（要求数据数组中至少包含2个参数：速度和位置）
      if (req->data.size() >= 2) {
        float speed, pos;
        memcpy(&speed, &req->data[0], sizeof(float));
        memcpy(&pos, &req->data[1], sizeof(float));
        send_speed_position_control(req->id, speed, pos);
      } else {
        res->result = "参数不足";
      }
      break;

    case 'T':  
      // 力控模式（要求数据数组中至少包含5个参数：力矩 位置 速度 kp ki）
      if (req->data.size() >= 5) {
        float torque, position, speed, kp, kd;
        memcpy(&torque, &req->data[0], sizeof(float));
        memcpy(&position, &req->data[1], sizeof(float));
        memcpy(&speed, &req->data[2], sizeof(float));
        memcpy(&kp, &req->data[3], sizeof(float));
        memcpy(&kd, &req->data[4], sizeof(float));
        send_torque_control(req->id, torque, position, speed, kp, kd);
      } else {
        res->result = "参数不足";
      }
      break;

    case 'R':  
      // 回零位
      send_return_to_zero(req->id);
      break;
      
    case 'D':  
      // 电机失能
      send_disable(req->id);
      break;

    default:
      res->result = std::string("Wrong command to srv: ").append(serve_pub);
      break;
  }
  is_srv_on = false;
  return true;
}

// 以下为各功能命令函数
/**
 * @brief  计算 CAN 扩展帧的 4 字节并写入 out[0..3]
 * @param  comm_type  通信类型 (协议文档中的 10 进制转 16 进制)
 * @param  host_id    主机 ID
 * @param  can_id     电机 CAN ID
 * @param  out4       长度≥4 的缓冲区，返回顺序为 [高字节→低字节]
 */
inline void build_extended_id(uint8_t comm_type,
                              uint8_t host_id,
                              uint8_t can_id,
                              uint8_t *out4)
{
    // 1. 拼 32 bit 原始 ID，次高字节固定 0x00
    uint32_t raw = (static_cast<uint32_t>(comm_type) << 24) |
                   (static_cast<uint32_t>(0x00)       << 16) |
                   (static_cast<uint32_t>(host_id)    <<  8) |
                    static_cast<uint32_t>(can_id);

    // 2. 左移 3 位并加扩展帧标识位 0b100
    uint32_t ext = (raw << 3) | 0x04;

    // 3. 按大端写入
    out4[0] = static_cast<uint8_t>(ext >> 24);
    out4[1] = static_cast<uint8_t>(ext >> 16);
    out4[2] = static_cast<uint8_t>(ext >>  8);
    out4[3] = static_cast<uint8_t>(ext       );
}

/***********************************
 * 电机反馈数据 (通信类型 0x02)
 *  · CAN_ID 1-4 :  ω ±30 rad/s,  τ ±12 Nm
 *  · 其他 ID   :  ω ±44 rad/s,  τ ±17 Nm
 *  · 角度      :  ±4π  (≈ 12.57 rad)   —— 全 ID 通用
 *  · 温度      :  Temp(℃) × 10
 ***********************************/
void cybergear::cybergear_send_read_req(uint8_t can_id_req)
{
  uint8_t tx[CAN_FRAME_LEN]{};
  tx[0] = CAN_FRAME_HEADER1;                 // 0x41
  tx[1] = CAN_FRAME_HEADER2;                 // 0x54

  uint8_t eid[4];
  build_extended_id(0x02, 0xFD, can_id_req, eid); // 通信类型 0x02：状态反馈
  memcpy(&tx[2], eid, 4);

  tx[6]  = 0x08;                             // DLC=8
  tx[15] = CAN_FRAME_TAIL1;                  // 0x0D
  tx[16] = CAN_FRAME_TAIL2;                  // 0x0A

  ser.flushInput();                          // 清读缓冲
  ser.write(tx, CAN_FRAME_LEN);
  // 不等待，这里立即返回；读取放到 poll 函数里做
}

// 非阻塞轮询读取+解析（可多帧）
void cybergear::cybergear_poll_and_parse_once()
{
  // 尽量整帧读取；设备可能一口气返回多帧
  while (ser.available() >= CAN_FRAME_LEN) {
    std::array<uint8_t, CAN_FRAME_LEN> rx{};
    size_t n = ser.read(rx.data(), CAN_FRAME_LEN);
    if (n < CAN_FRAME_LEN) break;

    // ① 头尾校验
    if (rx[0]!=CAN_FRAME_HEADER1 || rx[1]!=CAN_FRAME_HEADER2 ||
        rx[15]!=CAN_FRAME_TAIL1  || rx[16]!=CAN_FRAME_TAIL2) {
      RCLCPP_WARN(get_logger(), "CAN frame header/tail mismatch");
      continue;
    }

    // ② 解析 29-bit 扩展 ID
    uint32_t ext_id = (uint32_t(rx[2])<<24)|(uint32_t(rx[3])<<16)|
                      (uint32_t(rx[4])<<8) | uint32_t(rx[5]);
    uint32_t raw_id = ext_id >> 3;            // 去掉 0b100
    uint8_t  comm   = raw_id >> 24;           // 应 = 0x02
    if (comm != 0x02) {
      RCLCPP_WARN(get_logger(), "comm 0x%02X != 0x02", comm);
      continue;
    }
    uint16_t mid16    = (raw_id >> 8) & 0xFFFF;
    uint8_t  motor_id =  mid16 & 0xFF;        // id 在低 8 位
    uint8_t  mode     = (mid16 >> 14)&0x03;   // 可记录：模式
    uint8_t  fault    = (mid16 >>  8)&0x3F;   // 可记录：故障码

    // ③ 提取原始量
    auto u16 = [&](size_t i){ return (uint16_t(rx[i])<<8)|rx[i+1]; };
    uint16_t raw_ang = u16(7);
    uint16_t raw_vel = u16(9);
    uint16_t raw_tq  = u16(11);
    uint16_t raw_tmp = u16(13);

    // ④ 量化区间
    double V_MAX = (motor_id>=1 && motor_id<=4) ? 30.0 : 44.0;
    double T_MAX = (motor_id>=1 && motor_id<=4) ? 12.0 : 17.0;

    const double ANGLE_SCALE  = ( 8.0 * M_PI) / 65535.0;   // –4π~4π
    const double VELOC_SCALE  = ( 2.0 * V_MAX ) / 65535.0; // –V_MAX~V_MAX
    const double TORQUE_SCALE = ( 2.0 * T_MAX ) / 65535.0; // –T_MAX~T_MAX
    const double TEMP_SCALE   = 0.1;

    // ⑤ 反量化
    double angle = raw_ang * ANGLE_SCALE  - 4.0 * M_PI;
    double vel   = raw_vel * VELOC_SCALE  - V_MAX;
    double tq    = raw_tq  * TORQUE_SCALE - T_MAX;
    double temp  = raw_tmp * TEMP_SCALE;

    // ⑥ 映射到 id 索引
    size_t i = 0;
    for (; i<id.size(); ++i) if (id[i]==motor_id) break;
    if (i == id.size()) {
      RCLCPP_WARN(get_logger(), "unknown motor id=%u", motor_id);
      continue;
    }

    // ⑦ 更新缓存、通知待发布索引
    angle_now[i]  = angle;
    vel_now[i]    = vel;
    torque_now[i] = tq;
    temp_now[i]   = temp;
    msg_index     = static_cast<uint8_t>(i);

    // （可选日志）
    RCLCPP_INFO(get_logger(), "Motor[%u] mode=%u fault=0x%02X | θ=%.3f, ω=%.3f, τ=%.3f, T=%.1f",
      motor_id, mode, fault, angle, vel, tq, temp);
  }
}

void cybergear::publish_state()
{
  if (!is_srv_on) {
    // 1) 先尝试把串口里现有的帧都解析掉
    cybergear_poll_and_parse_once();

    // 2) 若有新解析出的电机，立刻发布
    if (msg_index != MAX_ID) {
      upload_msg(msg_index);
    }

    // 3) 轮询下一个待请求的电机，并发送读取请求
    if (++aquire_index >= id.size()) aquire_index = 0;
    cybergear_send_read_req(id[aquire_index]);
  }
}

void cybergear::upload_msg(uint8_t index)
{
  motor_data.header.stamp = now();
  // 生成一个唯一的帧标识字符串
  std::string frame_id = "cybergear_actuator_id";
  frame_id.push_back(char('0' + (id.at(index) % 10)));
  motor_data.header.frame_id = frame_id;
  // 填充消息数据
  motor_data.id.clear();
  motor_data.angle.clear();
  motor_data.velocity.clear();
  motor_data.torque.clear();
  motor_data.temperature.clear();

  motor_data.id.push_back(id.at(index));
  motor_data.angle.push_back(angle_now[index]);
  motor_data.velocity.push_back(vel_now[index]);
  motor_data.torque.push_back(torque_now[index]);
  motor_data.temperature.push_back(temp_now[index]);

  state_pub->publish(motor_data);
  msg_index = MAX_ID; // 重置
};

// 串口唤醒
void cybergear::send_wake()
{
  ser.flush();
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  transmit_buffer[2] = 0x2b;
  transmit_buffer[3] = CAN_FRAME_HEADER1;
  transmit_buffer[4] = CAN_FRAME_HEADER2;
  transmit_buffer[5] = CAN_FRAME_TAIL1;
  transmit_buffer[6] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("WAKE", transmit_buffer, 7).c_str());
#endif
  ser.write(transmit_buffer, 7);
  CAN_WAIT;
}

// 电机使能：功能码 0x18
void cybergear::send_enable(uint8_t id_)
{
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  uint8_t eid[4];
  build_extended_id(/*通信类型*/ 0x3,
                  /*主机ID*/   0xFD,
                  /*电机ID*/   id_,
                  eid);
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  for (int i = 7; i < 15; i++)
    transmit_buffer[i] = 0x00;
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("ENABLE", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;
}

// 设置当前位置为零位（仅在失能状态生效）：功能码 0x30，参数：0x01 0x01
void cybergear::send_set_current_zero(uint8_t id_)
{
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  uint8_t eid[4];
  build_extended_id(/*通信类型*/ 0x6,
                  /*主机ID*/   0xFD,
                  /*电机ID*/   id_,
                  eid);
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  transmit_buffer[7] = 0x01;
  transmit_buffer[8] = 0x01;
  for (int i = 9; i < 15; i++)
    transmit_buffer[i] = 0x00;
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("SET_CURRENT_ZERO", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;
}

// 速度位置控制：功能码 0x90，先发送速度帧（控制类型 0x17），再发送位置帧（控制类型 0x16）
// 速度与位置均为 IEEE754 浮点数，小端排列
void cybergear::send_speed_position_control(uint8_t id_, float speed, float pos)
{
  // 位置控制模式命令
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  uint8_t eid[4];
  build_extended_id(/*通信类型*/ 0x12,
                  /*主机ID*/   0xFD,
                  /*电机ID*/   id_,
                  eid);
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  transmit_buffer[7] = 0x05;         // 控制模式
  transmit_buffer[8] = 0x70;
  transmit_buffer[9] = 0x00;
  transmit_buffer[10] = 0x00;
  transmit_buffer[11] = 0x01;         // 位置控制模式:01
  transmit_buffer[12] = 0x00;
  transmit_buffer[13] = 0x00;
  transmit_buffer[14] = 0x00;
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("SP_CMD", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;

  // 速度命令
  if (id_ >= 5) {          
    memset(transmit_buffer, 0, sizeof(transmit_buffer));
    transmit_buffer[0] = CAN_FRAME_HEADER1;
    transmit_buffer[1] = CAN_FRAME_HEADER2;
    memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
    transmit_buffer[6] = 0x08;   // 数据长度 
    transmit_buffer[7] = 0x24;             // 控制类型：速度限制index 7024
    transmit_buffer[8] = 0x70;
    transmit_buffer[9] = 0x00;
    transmit_buffer[10] = 0x00;
    uint8_t speed_bytes[4];
    memcpy(speed_bytes, &speed, sizeof(float));
    memcpy(&transmit_buffer[11], speed_bytes, 4);
    transmit_buffer[15] = CAN_FRAME_TAIL1;
    transmit_buffer[16] = CAN_FRAME_TAIL2;
  #if REPORT_INFO
    RCLCPP_INFO(get_logger(), "%s", msg_print("SPEED_CMD", transmit_buffer, 17).c_str());
  #endif
    ser.write(transmit_buffer, 17);
    CAN_WAIT;
  }

  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度           
  transmit_buffer[7] = 0x17;  // 控制类型：速度限制index 7017
  transmit_buffer[8] = 0x70;
  transmit_buffer[9] = 0x00;
  transmit_buffer[10] = 0x00;
  uint8_t speed_bytes[4];
  memcpy(speed_bytes, &speed, sizeof(float));
  memcpy(&transmit_buffer[11], speed_bytes, 4);
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("SPEED_CMD", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;


  // 位置命令
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  transmit_buffer[7] = 0x16;         // 控制类型：位置⻆度index 7016
  transmit_buffer[8] = 0x70;
  transmit_buffer[9] = 0x00;
  transmit_buffer[10] = 0x00;
  uint8_t pos_bytes[4];
  memcpy(pos_bytes, &pos, sizeof(float));
  memcpy(&transmit_buffer[11], pos_bytes, 4);
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("POSITION_CMD", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;
}

// 运控模式
void cybergear::send_torque_control(uint8_t can_id,
                                    float torque,    // Nm
                                    float position,  // rad
                                    float speed,     // rad/s
                                    float kp, float kd)
{
    uint8_t tx[CAN_FRAME_LEN]{};
    /* ============== 帧头 ============== */
    tx[0] = CAN_FRAME_HEADER1;         // 41
    tx[1] = CAN_FRAME_HEADER2;         // 54

    /* ---------- 量化区间按 ID 分支 ---------- */
    float T_MIN, T_MAX, V_MIN, V_MAX;
    if (can_id >= 1 && can_id <= 4) {          // ★ 新增分支
        T_MIN = -12.0f;  T_MAX =  12.0f;
        V_MIN = -30.0f;  V_MAX =  30.0f;
    } else {                                   // 旧协议
        T_MIN = -17.0f;  T_MAX =  17.0f;
        V_MIN = -44.0f;  V_MAX =  44.0f;
    }

    /* ========== 29-bit 扩展 ID ========== */
    uint16_t t_u16 = static_cast<uint16_t>(
        float_to_uint(torque, T_MIN, T_MAX, 16));

    uint32_t raw_id = (0x01u << 24) |           // 通信类型 1
                      (uint32_t(t_u16) << 8) |
                       can_id;
    uint32_t ext_id = (raw_id << 3) | 0x04u;    // 左移3并 OR 0b100

    tx[2] = ext_id >> 24;
    tx[3] = ext_id >> 16;
    tx[4] = ext_id >>  8;
    tx[5] = ext_id;

    /* ============== DLC ================ */
    tx[6] = 0x08;                               // 固定 8 字节载荷

    auto put_u16 = [&](size_t idx, uint16_t v){
        tx[idx]   = v >> 8;
        tx[idx+1] = v & 0xFF;
    };

    /* ============ 数据区 =============== */
    put_u16(7,  static_cast<uint16_t>(
                   float_to_uint(position, -12.57f, 12.57f, 16))); // 角度
    put_u16(9,  static_cast<uint16_t>(
                   float_to_uint(speed, V_MIN, V_MAX, 16)));       // 速度
    put_u16(11, static_cast<uint16_t>(
                   float_to_uint(kp, 0.0f, 500.0f, 16)));          // Kp
    put_u16(13, static_cast<uint16_t>(
                   float_to_uint(kd, 0.0f,   5.0f, 16)));          // Kd

    /* ============== 帧尾 =============== */
    tx[15] = CAN_FRAME_TAIL1;  // 0D
    tx[16] = CAN_FRAME_TAIL2;  // 0A

#if REPORT_INFO
    RCLCPP_INFO(get_logger(), "%s",
        msg_print("TORQUE_CTRL", tx, sizeof(tx)).c_str());
#endif
    ser.write(tx, sizeof(tx));
    CAN_WAIT;
}



// 回零位：功能码 0x90，控制类型 0x05，参数中含 0x04
void cybergear::send_return_to_zero(uint8_t id_)
{
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  uint8_t eid[4];
  build_extended_id(/*通信类型*/ 0x12,
                  /*主机ID*/   0xFD,
                  /*电机ID*/   id_,
                  eid);
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  transmit_buffer[7] = 0x05;         // 控制类型：回零位
  transmit_buffer[8] = 0x70;
  transmit_buffer[9] = 0x00;
  transmit_buffer[10] = 0x00;
  transmit_buffer[11] = 0x04;        // 参数：0x04
  for (int i = 12; i < 15; i++)
    transmit_buffer[i] = 0x00;
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("RETURN_TO_ZERO", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;
}

// 电机失能：功能码 0x20
void cybergear::send_disable(uint8_t id_)
{
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = CAN_FRAME_HEADER1;
  transmit_buffer[1] = CAN_FRAME_HEADER2;
  uint8_t eid[4];
  build_extended_id(/*通信类型*/ 0x4,
                  /*主机ID*/   0xFD,
                  /*电机ID*/   id_,
                  eid);
  memcpy(&transmit_buffer[2], eid, 4);       // 写入序号 2-5 字节
  transmit_buffer[6] = 0x08;   // 数据长度
  for (int i = 7; i < 15; i++)
    transmit_buffer[i] = 0x00;
  transmit_buffer[15] = CAN_FRAME_TAIL1;
  transmit_buffer[16] = CAN_FRAME_TAIL2;
#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("DISABLE", transmit_buffer, 17).c_str());
#endif
  ser.write(transmit_buffer, 17);
  CAN_WAIT;
}

std::string cybergear::msg_print(const std::string &head, uint8_t *buffer, unsigned long len)
{
  std::ostringstream msg;
  msg << head << "-[";
  for (unsigned long i = 0; i < len; i++) {
    msg << std::hex << std::setw(2) << std::setfill('0') << +buffer[i] << " ";
  }
  std::string msg_out = msg.str();
  msg_out.pop_back();
  msg_out.append("]");
  return msg_out;
}
uint32_t cybergear::float_to_uint(float value, float min, float max, uint8_t bits) {
  // 限幅
  if (value < min) value = min;
  if (value > max) value = max;
  // 规范化
  float span = max - min;
  float normalized = (value - min) / span;
  uint32_t max_int = (1u << bits) - 1u;
  return static_cast<uint32_t>(std::round(normalized * max_int));
}
} // namespace cybergear
