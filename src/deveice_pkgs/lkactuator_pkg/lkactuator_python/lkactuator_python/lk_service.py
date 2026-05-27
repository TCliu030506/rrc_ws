import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from lkactuator_msg.msg import LkPosition
from x6_msg.msg import X6Jointstime
from typing import Optional
import can
import struct
import numpy as np
from collections import deque
import threading
from msg_codec_py.msg_coder import MsgCoder
from msg_codec_cpp.srv import Script

"""
----------------------------------------
lk_service.py
----------------------------------------
version 1.1.2: lk_service.py 2026-1-23

- 增加了读取并显示当前多圈绝对角度的功能（需注释掉大循环）
- 电机断电前需停在-180到180之间，否则重新上电会重置角度值（加减n圈）到此范围内
- 修复了部分注释错误
----------------------------------------
version 1.1: lk_service.py 2025-12-17
author: Yuchen Hong
email:  12425014@zju.edu.cn

新的特性：
- 优化了主循环进入轨迹运动的判断逻辑
- 优化了轨迹点存储方法，队列式可提前存储多个轨迹点，避免实时计算延迟
- 独立了定点快速运动模式 可直接打断轨迹运动
-（待添加）超出工作空间自动急停 手动急停
----------------------------------------
"""

PRINT_ENABLED = 0 #1为启用所有print 0为关闭

# 自定义 print 函数
def custom_print(*args, **kwargs):
    if PRINT_ENABLED == 1:
        print(*args, **kwargs)

class LkActuatorNode(Node):
    def __init__(self):
        super().__init__('lk_actuator_node_' + str(rclpy.clock.Clock().now().nanoseconds))
        self.loop_rate = 300
        self.topic_pub = "lkposition"
        self.serve_pub = "lk_service"
        self.channel = 'can0'
        self.can_bus: Optional[can.Bus] = None
        self.id = 1
        self.idx = 4
        self.cycle_number = np.zeros(6)
        # self.lock = threading.Lock()
        # ---- 伺服模式使用参数 ----
        self.angles_updated = np.array([180000.0,180000.0,180000.0,180000.0,180000.0,180000.0])
        self.angles_history = np.array([180000.0,180000.0,180000.0,180000.0,180000.0,180000.0])
        self.target_queue = deque()  # 存储待执行的目标角度数组
        self.current_target = None   # 当前正在执行的目标
        self.speed_limit = 720
        self.joint_speed = np.zeros(6)
        self.mode = 1
        self.goal_time = self.get_clock().now()# 当前目标的规定到达时间点
        self.frequency = self.loop_rate/6
        # self.motor_seq: list[int] = [5, 4, 3, 2, 1, 6]
        self.motor_seq: list[int] = [1, 2, 3, 4, 5, 6]
        self.info_once = False

        # ---- 初始化 CAN ----
        try:
            self.can_bus = can.interface.Bus(
                self.channel,
                bustype='socketcan',
                can_filters=[{"can_id": 0, "can_mask": 0}], 
            )
            self.get_logger().info(f"CAN 已连接: {self.channel}")
        except Exception as e:
            raise RuntimeError(f"打开 SocketCAN 失败：{e}")

        # ---- 创建发布器 ----可选，用于发布角度数据
        self.msg_pub = self.create_publisher(LkPosition, self.topic_pub, QoSProfile(depth=10))
        self.test_pub = self.create_publisher(LkPosition, 'cmd_msg', QoSProfile(depth=10))
        self.get_logger().info(f"Publisher created on topic: {self.topic_pub}")
        # ---- 创建订阅者 ----
        self.x6_sub = self.create_subscription(X6Jointstime,'x6tracking',self.callback_tracking,10)
       
        # ---- 创建服务 ----
        # 注册命令格式，只需在初始化时执行一次
        MsgCoder.cmd_script = { b'LK': {'cmd': '2s', 'id': 'b'},
                                b'UN': {'cmd': '2s', 'id': 'b'},
                                b'SP': {'cmd': '2s', 'id': 'b'},
                                b'MS': {'cmd': '2s', 'id': 'b', 'spd': 'q'},
                                b'MA': {'cmd': '2s', 'id': 'b', 'spd': 'q', 'pos': 'q'},
                                b'MB': {'cmd': '2s', 'id': 'b', 'spd': 'q', 'pos': 'q'},
                                b'RA': {'cmd': '2s', 'id': 'b'},
                                b'RP': {'cmd': '2s', 'id': 'b', 'num': 'b'},
                                
        }
        self.srv_pub = self.create_service(Script, "lk_service", self.ser_callback)
        self.get_logger().info(f"Service created on: {self.serve_pub}")
        
        # ---- 读取初始多圈绝对位置 ----
        self.cmd_read_angle(1)
        self.cmd_read_angle(2)
        self.cmd_read_angle(3)
        self.cmd_read_angle(4)
        self.cmd_read_angle(5)
        self.cmd_read_angle(6)

        # ---- 定时器 ----可选，用于发布角度数据
        tx_first = can.Message(arbitration_id=0x141, data=[0x9C, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
        self.can_bus.send(tx_first)
        self.timer = self.create_timer(1.0 / self.loop_rate, self.cyclic_tasks)

    # ==============================================================
    # 读取角度并发布（通过 CAN 请求反馈）
    # ==============================================================

    def cycle_updated(self, angle_i64, idx):
        self.angles_history[idx] = self.read_multicycle_angles()[idx]
        if angle_i64 - self.angles_updated[idx] >= 180000:
            self.cycle_number[idx] -= 1
        elif angle_i64 - self.angles_updated[idx] <= -180000:
            self.cycle_number[idx] += 1
        else:
            pass
        self.angles_updated[idx] = angle_i64

    def read_multicycle_angles(self):
        ans: np.ndarray = (self.cycle_number*360000 + self.angles_updated)
        return ans
    
    def cyclic_tasks(self):
        """
        模拟从 CAN 设备读取角度值：
        发送读取命令帧
        若收到运动指令，计算路径点并发送
        若无运动指令，设备回传反馈帧
        """
        # print(self.get_clock().now().nanoseconds)
        #收
        self.read_angle()
        # 如果有活跃目标，则计算并发送速度指令

        # if self.current_target is not None or len(self.target_queue) > 0:
        if self.get_clock().now() < self.goal_time:
             #算
            period_sec = 1.0 / self.frequency      
            # 注意：确保 time_ 为正，如果出现轻微负数,说明该去下一个目标点了
            time_ = self.goal_time - self.get_clock().now()
            time_sec = time_.nanoseconds * 1e-9  # 纳秒转秒
            # 计算每个关节速度
            planned_step = self.joint_speed[self.idx]/10/self.frequency*1000  # 电机角度单位=实际角度*1000
            angle_diffs_signed = (self.current_target[self.idx] - (self.read_multicycle_angles()[self.idx] + planned_step))  
            joint_speed = (angle_diffs_signed/1000) / time_sec*10 #带符号 速度单位度/10s
            self.joint_speed[self.idx] = max(min(joint_speed, self.speed_limit), -self.speed_limit)
            #遍历
            self.idx += 1
            if self.idx > 5:
                self.idx = 0
            motor_id = self.motor_seq[self.idx]

            if self.get_clock().now() < self.goal_time-rclpy.duration.Duration(seconds=period_sec):
                #发
                speedcontrol = self.joint_speed[self.idx] #单位度/10s
                self.cmd_SpeedControl_sendonly(motor_id, speedcontrol)
                # # 测试用，发布命令内容
                # test_pub = LkPosition()
                # test_pub.speed = int(speedcontrol)
                # test_pub.header.stamp = self.get_clock().now().to_msg()
                # test_pub.header.frame_id = str(motor_id)
                # self.test_pub.publish(test_pub)
                #
            else:
                if len(self.target_queue) > 0:
                    # 预先开始下一个目标的执行
                    self._start_next_target()
                    motor_id = self.motor_seq[self.idx]
                    self.cmd_read_encoder(motor_id)
                elif len(self.target_queue) == 0:
                    # 轨迹已走完，最后一个点以定点指令发送
                    self.cmd_AngleControl_absolute_sendonly(motor_id, np.abs(self.joint_speed[self.idx]), self.current_target[self.idx])        
        else:
            #发
            self.idx += 1
            if self.idx > 5:
                self.idx = 0
            motor_id = self.motor_seq[self.idx]
            self.cmd_read_encoder(motor_id)
            # self.id += 1
            # if self.id > 6:
            #     self.id = 1 
            # self.cmd_read_encoder(self.id)
            #清空目标队列
            self.goal_time = self.get_clock().now()
            self.current_target       
            self.joint_speed[self.idx] = 0

        # print(self.get_clock().now().nanoseconds)
        # print(motor_id)

    def read_angle(self):
        #接收电机回复解读成角度信息
        msg: can.Message = self.can_bus.recv(timeout=0.003) 
        if msg and len(msg.data) >= 8:
            cmd_type = msg.data[0]
            if cmd_type in [0x9C, 0xA4, 0xA8, 0xA2]:  #读取编码器反馈
                #解析电机速度 单位度/10s
                speed_low = msg.data[4]
                speed_high = msg.data[5]
                speed_value = (speed_high << 8) | speed_low
                if speed_value >=32767:
                    speed_i16 = speed_value - 65536
                else:
                    speed_i16 = speed_value
                #解析编码器位置 单位0.001度
                encoder_low = msg.data[6]
                encoder_high = msg.data[7]
                encoder_value = (encoder_high << 8) | encoder_low
                angle_i64 = round(encoder_value * 360000 / 65536)
                #解析回复电机id,储存当前角度
                motor_id = msg.arbitration_id - 0x140
                idx = self.motor_seq.index(int(str(motor_id)))
                self.cycle_updated(angle_i64, idx)
                self.idx = idx
                #发布读取到的角度信息    
                msg_pub = LkPosition()
                msg_pub.val = int(self.read_multicycle_angles()[idx])
                msg_pub.speed = speed_i16
                msg_pub.header.stamp = self.get_clock().now().to_msg()
                msg_pub.header.frame_id = str(motor_id)
                self.msg_pub.publish(msg_pub)
               
            else:
                self.get_logger().info("超出工作空间，自动急停")
        else:
            self.get_logger().warn("未收到角度反馈")
            self.get_logger().warn(msg)

    # ==============================================================
    # 清理资源
    # ==============================================================
    def destroy_node(self):
        if self.can_bus:
            try:
                self.get_logger().info("正在关闭 CAN bus ...")
                self.can_bus.shutdown()
                self.get_logger().info("CAN bus 已关闭")
            except Exception as e:
                self.get_logger().warn(f"关闭 CAN bus 失败: {e}")
            finally:
                self.can_bus = None
        super().destroy_node()

    # ==============================================================
    # 回调函数
    # ==============================================================
    
    def callback_tracking(self,msg3: X6Jointstime):
        """接收新的目标点，并将其加入队列"""
        new_target = np.array(msg3.angles)
        self.mode = msg3.mode
        # 如果是模式1（定点快速运动），直接在此发送所有电机的绝对位置指令
        if self.mode == 1:
            fixed_target = new_target
            #计算速度
            angle_diffs_signed = (np.array(fixed_target) - self.read_multicycle_angles())  
            joint_speed = (angle_diffs_signed/1000) / msg3.cost_time*10 #带符号 速度单位度/10s
            #清空正在执行的轨迹指令
            self.current_target = None
            self.target_queue.clear()
            self.goal_time = self.get_clock().now()
            for idx in range(6):
                motor_id = self.motor_seq[idx]
                self.cmd_AngleControl_absolute(motor_id, np.abs(joint_speed[idx]), fixed_target[idx])
        #如果是模式2（轨迹运动），加入队列
        elif self.mode == 2:
            self.target_queue.append({
                'angles': new_target,
                'cost_time': msg3.cost_time,
            })
            # self.get_logger().info(f"新目标点已加入队列，队列长度: {len(self.target_queue)}")
            # 模式：一收到轨迹点就开始运动 （二选一）
            # if self.get_clock().now() >= self.goal_time:
            #     self._start_next_target()
            # 模式：收到轨迹运动开始指令后 执行轨迹第一个目标 （二选一）
        elif self.mode == 0:
                self._start_next_target()

    def _start_next_target(self):
        """从队列中取出下一个目标并开始执行"""
        # if len(self.target_queue) == 0:
        #     self.current_target = None
        #     # self.get_logger().info("目标队列已清空")
        #     return

        # 取出队列最前端的任务
        target_data = self.target_queue.popleft()
        self.current_target = target_data['angles']
        # 设置目标时间：从当前时刻算起，经过 cost_time 秒后应到达
        self.goal_time = self.get_clock().now() + rclpy.duration.Duration(seconds=target_data['cost_time'])
        # self.get_logger().info(f"开始执行新目标，队列剩余: {len(self.target_queue)}")
      

    def ser_callback(self, req: Script.Request, res: Script.Response):
        try:
            encoded_msg = bytes(req.command.data)
            req = MsgCoder.msg_decode(encoded_msg)
            match req['cmd']:
                case b'MA':
                    res.reply=self.cmd_AngleControl_add(int(req['id']), int(req['spd']), int(req['pos']))
                case b'MS':
                    res.reply=self.cmd_SpeedControl(int(req['id']), int(req['spd']))
                case b'MB':
                    res.reply=self.cmd_AngleControl_absolute(int(req['id']), int(req['spd']), int(req['pos']))
                case b'LK':
                    res.reply=self.cmd_lock(int(req['id']))
                case b'UN':
                    res.reply=self.cmd_unlock(int(req['id']))
                case b'SP':
                    res.reply=self.cmd_stop(int(req['id']))
                case b'RA':
                    res.reply=self.cmd_read_angle(int(req['id']))
                case b'RP':
                    res.reply=self.cmd_read_control_parameter(int(req['id']), int(req['num']))
                case _:
                    res.reply=self.get_logger().warn(f"Unknown mode: {req['cmd']}")

        except Exception as e:
            self.get_logger().error(f"Lk服务执行出错: {e}")
            res.reply = False
        return res
    
    # ==============================================================
    # 动作指令和读取状态指令(读取指令由于先收再发 没有返回值)
    # ==============================================================
    def cmd_AngleControl_add(self, motor_id: int, max_speed: int, angle_control: int):
        """
        增量模式 (MODE1)
        发送格式: [0xA8, 0x00, maxSpeed(LE2), angleControl(LE4)]
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0xA8
        tx_data[1] = 0x00
        tx_data[2:4] = struct.pack("<H", int(max_speed))
        tx_data[4:8] = struct.pack("<i", int(angle_control))

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] MODE1 ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True

    def cmd_SpeedControl(self, motor_id: int, speedcontrol: int):
        """
        速度控制 (MODE2)
        发送格式: [0xA2, 0x00, 0x00, 0x00, maxSpeed(LE4)]
        36000=36度/s 已在本函数中转换为度/10s
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0xA2
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4:8] = struct.pack("<i", int(speedcontrol*100))

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] MODE2 ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True

    def cmd_SpeedControl_sendonly(self, motor_id: int, speedcontrol: int):
        """
        速度控制 (MODE2)
        发送格式: [0xA2, 0x00, 0x00, 0x00, maxSpeed(LE4)] maxSpeed单位为度/10s
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0xA2
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4:8] = struct.pack("<i", int(speedcontrol*100))

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] MODE2 ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
    
    def cmd_AngleControl_absolute(self, motor_id: int, max_speed: int, angle_control: int) -> str: 
        """
        多圈模式 (MODE3)
        发送格式: [0xA4, 0x00, maxSpeed(LE2), (angleControl)(LE4)]
        360=36度/s  180000=180度
        """
        arb_id = 0x140 + motor_id
        angle_sum = int(angle_control)

        tx_data = bytearray(8)
        tx_data[0] = 0xA4
        tx_data[1] = 0x00
        tx_data[2:4] = struct.pack("<H", int(max_speed))
        tx_data[4:8] = struct.pack("<i", int(angle_sum))

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] MODE3 ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True

    def cmd_AngleControl_absolute_sendonly(self, motor_id: int, max_speed: int, angle_control: int) -> str: 
        """
        多圈模式 (MODE3)
        发送格式: [0xA4, 0x00, maxSpeed(LE2), (angleControl)(LE4)]
        """
        arb_id = 0x140 + motor_id
        angle_sum = int(angle_control)

        tx_data = bytearray(8)
        tx_data[0] = 0xA4
        tx_data[1] = 0x00
        tx_data[2:4] = struct.pack("<H", int(max_speed))
        tx_data[4:8] = struct.pack("<i", int(angle_sum))

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] MODE3 ID=0x{arb_id:X} Data={tx_data.hex().upper()}")

    def cmd_stop(self,motor_id: int):
        """
        停止运动,但不清除电机运行状态。再次发送控制指令即可控制电机动作
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0x81
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] LOCK CMD  ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True

    def cmd_lock(self,motor_id: int):
        """
        上锁，不可移动
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0x88
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] LOCK CMD  ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True
        
    def cmd_unlock(self,motor_id: int):
        """
        解锁，可移动
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0x80
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        custom_print(f"[TX] UNLOCK ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg = self.can_bus.recv(timeout=0.2)
        if msg != None:
            return True
        
    def cmd_read_angle(self,motor_id: int):
        """
        读取多圈绝对角度
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0x92
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        # custom_print(f"[TX] READ ANGLE ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg: can.Message = self.can_bus.recv(timeout=0.2) 
        if msg != None:
            if len(msg.data) < 8 or msg.data[0] != 0x92:
                return False
            else:
                # 获取7个角度字节
                angle_bytes = bytes(msg.data[1:8])
                angle_raw = int.from_bytes(angle_bytes, byteorder='little', signed=True)
                angle_abs = angle_raw  # 转换为0.001度单位的多圈绝对角度
                print(f"[RX] READ ANGLE ID=0x{arb_id:X} angle_abs: {angle_abs}")
                return True
        
       

    def cmd_read_encoder(self,motor_id: int):
        """
        读取编码器形式的角度和速度
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0x9C
        tx_data[1] = 0x00
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        # custom_print(f"[TX] READ ENCODER ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        
    def cmd_read_control_parameter(self, motor_id: int, number: int):
        """
        读取控制参数
        """
        arb_id = 0x140 + motor_id
        tx_data = bytearray(8)
        tx_data[0] = 0xC0
        tx_data[1] = number
        tx_data[2] = 0x00
        tx_data[3] = 0x00
        tx_data[4] = 0x00
        tx_data[5] = 0x00
        tx_data[6] = 0x00
        tx_data[7] = 0x00

        order = can.Message(arbitration_id=arb_id, data=tx_data, is_extended_id=False)
        self.can_bus.send(order)
        print(order)
        custom_print(f"[TX] READ CONTROL PARAMETER ID=0x{arb_id:X} Data={tx_data.hex().upper()}")
        msg: can.Message = self.can_bus.recv(timeout=0.2)
        print(msg)
        if msg and len(msg.data) >= 8:
            cmd_type = msg.data[1]
            motor_id = msg.arbitration_id - 0x140
            if cmd_type == 0x0A:  # 输入10 读取角度环参数
                anglePidKp = int.from_bytes(msg.data[2:4], byteorder='little', signed=True)
                anglePidKi = int.from_bytes(msg.data[4:6], byteorder='little', signed=True)
                anglePidKd = int.from_bytes(msg.data[6:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 角度环参数: {anglePidKp}, {anglePidKi}, {anglePidKd}")
            
            elif cmd_type == 0x0B:  # 输入11 读取速度环参数
                speedPidKp = int.from_bytes(msg.data[2:4], byteorder='little', signed=True)
                speedPidKi = int.from_bytes(msg.data[4:6], byteorder='little', signed=True)
                speedPidKd = int.from_bytes(msg.data[6:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 速度环参数: {speedPidKp}, {speedPidKi}, {speedPidKd}")
            
            elif cmd_type == 0x0C:  # 输入12 读取电流环参数
                currentPidKp = int.from_bytes(msg.data[2:4], byteorder='little', signed=True)
                currentPidKi = int.from_bytes(msg.data[4:6], byteorder='little', signed=True)
                currentPidKd = int.from_bytes(msg.data[6:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 电流环参数: {currentPidKp}, {currentPidKi}, {currentPidKd}")  

            elif cmd_type == 0x1E:  # 输入30 读取最大力矩电流 
                inputTorqueLimit = int.from_bytes(msg.data[4:6], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 最大力矩电流: {inputTorqueLimit}")  

            elif cmd_type == 0x20:  # 输入32 读取最大速度
                inputSpeedLimit = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 最大速度: {inputSpeedLimit}")

            elif cmd_type == 0x22:  # 输入34 读取最大角度
                inputAngleLimit = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 最大角度: {inputAngleLimit}")

            elif cmd_type == 0x24:  # 输入36 读取电流斜率
                inputCurrentRamp = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 电流斜率: {inputCurrentRamp}")

            elif cmd_type == 0x26:  # 输入38 读取速度斜率
                inputSpeedRamp = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
                self.get_logger().info(f"电机{motor_id} 速度斜率: {inputSpeedRamp}")

            else:
                self.get_logger().info("请读取正确的控制参数")
                self.get_logger().info(msg.data)
                return False
            return True
        else:
            self.get_logger().warn("未收到电机反馈")
            self.get_logger().warn(msg)
            return False


# ==============================================================
# 主函数
# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = LkActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
