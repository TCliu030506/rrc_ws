#!/usr/bin/env python3

from __future__ import annotations

import rclpy
import rclpy.clock
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import Header
from typing import Dict, Any, Protocol, Optional, Type, List, Tuple
from datetime import datetime
import pandas as pd
import threading
import time
import array


import os
# 获取执行程序所在功能包路径,假设执行程序位于功能包根目录
pkg_path = os.path.dirname(os.getcwd())

class ROS2Msg(Protocol):
    """
    ROS2 消息协议接口
    """
    def get_fields_and_field_types(self) -> Dict[str, str]:
        ...

class MsgWithHeader(Protocol):
    """定义一个协议，要求实现消息类必须有 `header` 属性"""
    header: Header

class MsgFilterNode(Node):
    """
    通用消息过滤节点
    使用说明:
     - 以 ROS2 节点运行
     - 通过参数指定输入/输出话题、消息类型和目标 frame_id
    """
    def __init__(self, msg_type: Type):
        """
        初始化过滤节点
        Args:
            msg_type: ROS2 消息类型，必须包含 header 属性
        """
        super().__init__('msg_filter_'+str(rclpy.clock.Clock().now().nanoseconds))

        # 参数声明与默认值
        self.declare_parameter('input_topic', '/topic_input')
        self.declare_parameter('output_topic', '/topic_filtered')
        self.declare_parameter('target_frame_id', '')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value

        # 创建 publisher 和 subscription
        self.pub = self.create_publisher(msg_type, output_topic, 10)
        self.sub = self.create_subscription(msg_type, input_topic, self._callback, 10)

        self.get_logger().info(f"订阅: {input_topic}, 发布: {output_topic}, 目标 frame_id: '{self.target_frame_id}'")

    def _callback(self, msg: MsgWithHeader):
        # 尝试获取 header
        header = getattr(msg, 'header', None)
        if header is None:
            # 没有 header 时直接跳过（也可以选择发布或转换）
            self.get_logger().debug('接收到的消息没有 header，跳过')
            return

        if self.target_frame_id == '' or msg.header.frame_id == self.target_frame_id:
            # 符合条件则发布原消息
            self.pub.publish(msg)
            self.get_logger().debug(f'已发布符合条件的消息，frame_id={msg.header.frame_id}')
        else:
            self.get_logger().debug(f'消息 frame_id 不匹配（收到={msg.header.frame_id}，期望={self.target_frame_id}），已丢弃')

class MessageFlattener:
    """消息扁平化工具类"""
    
    def __init__(self, 
            separator: str = ".",
            flatten_arrays: bool = True,
            array_index_format: str = "[{index}]"):
        """
        初始化扁平化配置
        
        Args:
            separator: 字段分隔符
            flatten_arrays: 是否扁平化数组
            array_index_format: 数组索引格式
        """
        self.separator = separator
        self.flatten_arrays = flatten_arrays
        self.array_index_format = array_index_format
    
    def flatten(self, msg: ROS2Msg) -> Dict[str, Any]:
        """扁平化消息"""
        return self._flatten_recursive(msg, "")
    
    def _flatten_recursive(self, obj: ROS2Msg, current_path: str) -> Dict[str, Any]:
        result = {}
        
        if hasattr(obj, 'get_fields_and_field_types'):
            # ROS2消息
            for field_name in obj.get_fields_and_field_types().keys():
                field_value = getattr(obj, field_name)
                import numpy as np

                # 处理 numpy 数组
                if isinstance(field_value, np.ndarray):
                    field_value = field_value.tolist()
                if field_value == [] or field_value == {} or field_name == "data_offset":
                    continue  # 跳过空字段

                new_path = f"{current_path}{self.separator}{field_name}" if current_path else field_name
                result.update(self._flatten_recursive(field_value, new_path))
        
        elif isinstance(obj, (list, array.array)) and obj:

            # 数组处理
            if self.flatten_arrays:
                if hasattr(obj[0], 'get_fields_and_field_types'):
                    # 扁平化消息数组
                    for i, item in enumerate(obj):
                        index_str = self.array_index_format.format(index=i)
                        new_path = f"{current_path}{index_str}"
                        result.update(self._flatten_recursive(item, new_path))
                elif all(isinstance(v, (int, float, str, bool)) for v in obj):
                    # 扁平化基本类型数组
                    for i, item in enumerate(obj):
                        index_str = self.array_index_format.format(index=i)
                        new_path = f"{current_path}{index_str}"
                        result.update(self._flatten_recursive(item, new_path))
                else:
                    # 保持数组结构
                    result[current_path] = list(obj)
            else:
                # 保持数组结构
                result[current_path] = list(obj)

        else:
            # 基本类型
            result[current_path] = obj
        
        return result

class MsgListener:
    """
    消息监听器类，用于订阅 ROS2 消息，在内部更新。
    """
    def __init__(self, 
        msg_type: Type,
        topic_name: str):
        """
        初始化消息监听器
        Args:
            msg_type: ROS2 消息类型
            topic_name: 订阅的主题名称
        """
        try:    
            rclpy.init()
        except Exception:
            # 已初始化则忽略
            pass
        self.node = rclpy.create_node('msg_listener_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.subscription = self.node.create_subscription(
            msg_type,
            topic_name,
            self.msg_callback,
            10)
        
        self.latest_data: Dict[str, Any] = {}
        self.msgflattener = MessageFlattener()

        self.stop_event = threading.Event()
        thread_name = "msg_listen_thread_" + topic_name
        self.thread = threading.Thread(target=self.spin_node, name=thread_name, daemon=True)
        self.thread.start()
        self.node.get_logger().info(f"Ready to listen topic: {topic_name}")

    def stop_node(self):
        self.stop_event.set()
        self.thread.join()
        self.node.destroy_node()

    def spin_node(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        while not self.stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=1.0)
            except KeyboardInterrupt:
                break

    def msg_callback(self, msg: ROS2Msg):
        """
        消息回调函数，记录最新消息和时间戳
        """
        self.latest_data = self.msgflattener.flatten(msg)

class MsgRecorder:
    """
    消息记录器类，用于记录 ROS2 消息。
    """
    # def __init__(self, 
    #     msg_listener: List[Tuple[Type, str]],
    #     package_path: str = pkg_path,
    #     save_path: str = "record"):
    #     """
    #     初始化消息记录器
    #     Args:
    #         msg_listener: 订阅的消息类型和主题名称列表
    #         package_path: 包路径, 默认获取执行程序所在功能包路径
    #         save_path: 默认保存路径在 package_path 下的 record 文件夹
    #     """
    #     self.save_path = package_path + "/" + save_path
    #     self.recorded_df: pd.DataFrame = pd.DataFrame()
    #     self.data_lock = threading.Lock()

    #     # 记录状态控制
    #     self.is_recording = False
    #     self.periodic_timer = None

    #     self.msg_listeners: List[MsgListener] = []
    #     for topic_type, topic_name in msg_listener:
    #         instance = MsgListener(topic_type,topic_name)
    #         self.msg_listeners.append(instance)
    """
    临时修改init和add_msg:记录多个相同结构话题 同类型消息会覆盖 现在不会了
    """
    def __init__(self, 
        msg_listener: List[Tuple[Type, str, str]],  # 改为 (消息类型, 话题名称, 自定义前缀)
        package_path: str = pkg_path,
        save_path: str = "record"):
        
        self.save_path = package_path + "/" + save_path
        self.recorded_df: pd.DataFrame = pd.DataFrame()
        self.data_lock = threading.Lock()

        self.is_recording = False
        self.periodic_timer = None

        self.msg_listeners: List[MsgListener] = []
        self.topic_prefixes: List[str] = []  # 保存每个话题的前缀
        
        for item in msg_listener:
            if len(item) == 3:
                topic_type, topic_name, prefix = item
            else:
                topic_type, topic_name = item
                # 自动生成前缀：去掉开头的/，用_替换其他/
                prefix = topic_name.replace('/', '_').lstrip('_')
            
            instance = MsgListener(topic_type, topic_name)
            self.msg_listeners.append(instance)
            self.topic_prefixes.append(prefix)

    def add_msg(self):
        with self.data_lock:
            new_msg: Dict[str, Any] = {}
            new_msg['timestamp'] = datetime.now()
            
            for i, instance in enumerate(self.msg_listeners):
                prefix = self.topic_prefixes[i]
                for key, value in instance.latest_data.items():
                    prefixed_key = f"{prefix}_{key}"
                    new_msg[prefixed_key] = value
            
            self.recorded_df = pd.concat([self.recorded_df, pd.DataFrame([new_msg])], ignore_index=True)
    
    def stop_node(self):
        for instance in self.msg_listeners:
            instance.stop_node()

    # def add_msg(self):
    #     """
    #     手动添加当前最新消息到记录
    #     """
    #     with self.data_lock:
    #         new_msg: Dict[str,Any] = {}
    #         new_msg['timestamp'] = datetime.now()
    #         for instance in self.msg_listeners:
    #             new_msg.update(instance.latest_data)
    #         self.recorded_df = pd.concat([self.recorded_df, pd.DataFrame([new_msg])], ignore_index=True)

    def read_latest(self) -> Dict[str, Any]:
        """
        读取最新消息
        Returns:
            最新消息的扁平化字典
        """
        new_msg: Dict[str,Any] = {}
        new_msg['timestamp'] = datetime.now()
        for instance in self.msg_listeners:
            new_msg.update(instance.latest_data)
        return new_msg
    
    def read_recorded(self) -> pd.DataFrame:
        """
        读取已记录的消息数据框
        Returns:
            记录的消息数据框
        """
        with self.data_lock:
            return self.recorded_df.copy()
        
    def report_recorded_data(self) -> None:
        """
        打印已记录数据的摘要信息
        """
        with self.data_lock:
            if self.recorded_df.empty:
                print("No recorded data.")
                return
            info = f"Recorded {len(self.recorded_df)} messages with columns: {list(self.recorded_df.columns)}"
            print(info)
            pd.set_option('display.max_colwidth', 20)
            pd.set_option('display.float_format', '{:.4f}'.format)
            print(self.recorded_df)

    def start_recording(self, freq: float, clear_existing: bool = True):
        """
        开始周期性记录消息
        Args:
            freq: 运行频率
            clear_existing: 是否清除已有记录
        """
        with self.data_lock:
            if clear_existing:
                self.recorded_df = pd.DataFrame()
            self.is_recording = True
        
        # 启动定时器线程
        def periodic_task():
            while self.is_recording:
                time.sleep(1.0 / freq)
                self.add_msg()

        self.periodic_timer = threading.Thread(target=periodic_task, name="periodic_record_thread", daemon=True)
        self.periodic_timer.start()
        print(f"Started periodic recording in frequency {freq} hz.")

    def stop_recording(self, file_name: Optional[str] = None):
        """
        停止记录消息,并保存到 CSV 文件
        Args:
            file_name: 自定义文件名（不含路径和扩展名）
        """
        # 保存记录到 CSV 文件
        self.save_recording(file_name)

        self.is_recording = False
        if self.periodic_timer and self.periodic_timer.is_alive():
            self.periodic_timer.join()
            self.periodic_timer = None
        print("Stopped recording.")

    def save_recording(self, file_name: Optional[str] = None):
        """
        保存当前记录到 CSV 文件
        Args:
            file_name: 自定义文件名（不含路径和扩展名）
        """
        timestamp = datetime.now().strftime('_%Y%m%d_%H%M%S')

        file_path = self.save_path + "/"
        file_path += file_name if file_name else "record"
        file_path += f"{timestamp}.csv"

        with self.data_lock:
            self.recorded_df.to_csv(file_path, index=False)
        print(f"Recorded data saved to {file_path}.")

#  node_params 类：用于通过服务调用设置 ROS2 节点参数
class node_params:
    """
    通过调用 '/<node_name>/set_parameters' 服务来设置参数。
    支持 bool/int/float/str/bytes, 以及 list/tuple 类型的数组：
    空数组默认作为 STRING_ARRAY 处理。
    set_param() 方法用于设置参数值, get_param() 方法用于读取参数值, 参数需要时间更新, 使用read_value() 方法获取该参数值。
    使用实例见 main() 函数。
    """
    def __init__(self, node_name: str, auto_stop: bool = True, debug: bool = False) -> None:
        """
        初始化 node_params 实例，创建客户端连接到指定节点的参数服务。
        Args:
            node_name: 目标节点名称（不含前导斜杠）
            auto_stop: 是否在完成操作后自动停止节点, 默认启用, 即在设置或获取参数后停止节点,无法连续调用多个参数操作
            debug: 是否启用调试模式
        """
        try:    
            rclpy.init()
        except Exception:
            # 已初始化则忽略
            pass
        self.param: ParameterValue = None
        self.debug = debug
        self.auto_stop = False
        self.node = rclpy.create_node('params_node'+str(rclpy.clock.Clock().now().nanoseconds))
        self.cli_set = self.node.create_client(SetParameters,'/'+str(node_name)+'/set_parameters')
        self.cli_get = self.node.create_client(GetParameters,'/'+str(node_name)+'/get_parameters')
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.spin_node, name="param_wait_thread", daemon=True)
        self.thread.start()
        self.working = True

    def spin_node(self):
        while not self.stop_event.is_set():
            try:
                rclpy.spin_once(self.node, timeout_sec=1.0)
            except KeyboardInterrupt:
                break

    def stop_node(self):
        if self.debug:
            print("Destroying node_params...")
        self.stop_event.set()
        self.thread.join()
        self.node.destroy_node()
        self.working = False

    def set_param(self, param_name: str, param_value: Any, timeout_second: float = 1.0) -> None:
        """
        设置单个参数，支持 bool/int/float/str/bytes, 以及 list/tuple 类型的数组：
        空数组默认作为 STRING_ARRAY 处理。
        """
        if not self.working:
            raise RuntimeError("node_params instance has been stopped.")
        if not self.cli_set.wait_for_service(timeout_sec=timeout_second):
            self.stop_event.set()
            self.node.destroy_node()
            raise RuntimeError(f"Service '/{self.node.get_name()}/set_parameters' not available")
        
        req = SetParameters.Request()
        pv = self.pack_value(param_value)
        req.parameters = [Parameter(name=param_name, value=pv)]
        # 异步调用服务
        self.future_set: Future = self.cli_set.call_async(req)
        self.future_set.add_done_callback(self.cliset_callback)

    def get_param(self, param_name: str, timeout_second: float = 1.0) -> None:
        """
        获取单个参数值,读取到回复后数据会存储在 self.param 中，
        可通过 read_value() 方法获取 Python 原生类型的值。
        需要等待指定的 timeout_sec 秒数以确保收到回复并存放在 self.param 中。
        """
        if not self.working:
            raise RuntimeError("node_params instance has been stopped.")
        if not self.cli_get.wait_for_service(timeout_sec=timeout_second):
            self.stop_event.set()
            self.node.destroy_node()
            raise RuntimeError(f"Service '/{self.node.get_name()}/get_parameters' not available")

        req = GetParameters.Request()
        req.names = [param_name]
        self.future_get = self.cli_get.call_async(req)
        self.future_get.add_done_callback(self.cliget_callback)

    def read_value(self) -> Any:
        """
        读取参数值，返回 Python 原生类型。
        需要先调用 get_param() 方法获取参数值，并判断 self.param 非空, 即已被更新
        读取到后会将 self.param 置空，避免重复读取同一值。
        """
        if self.param is None:
            print("No parameter value to parse")
            return None
        msg = self.param
        t = msg.type
        self.param = None

        if t == ParameterType.PARAMETER_BOOL:
            return bool(msg.bool_value)
        elif t == ParameterType.PARAMETER_INTEGER:
            return int(msg.integer_value)
        elif t == ParameterType.PARAMETER_DOUBLE:
            return float(msg.double_value)
        elif t == ParameterType.PARAMETER_STRING:
            return str(msg.string_value)
        elif t == ParameterType.PARAMETER_BYTE_ARRAY:
            return bytes(msg.byte_array_value)
        elif t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(msg.bool_array_value)
        elif t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(msg.integer_array_value)
        elif t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(msg.double_array_value)
        elif t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(msg.string_array_value)
        else:
            return None

    def cliset_callback(self, future: Future):
        if not self.working:
            return
        try:
            if self.debug:
                response = future.result()
                print("Parameter set response:", response)
        except Exception as e:
            print("Service call failed:", e)
        finally:
            if self.auto_stop:
                self.stop_event.set()

    def cliget_callback(self, future: Future):
        if not self.working:
            return
        try:
            response: GetParameters.Response = future.result()
            if not response.values:
                raise RuntimeError(f'Parameter not found on node')
            # Store the parameter value
            self.param = response.values[0]
            if self.debug:
                print("Parameter get response:", response)
        except Exception as e:
            self.callback_exception = e
            print("Service call failed:", e)
        finally:
            if self.auto_stop:
                self.stop_event.set()

    def pack_value(self, value: Any) -> ParameterValue:
        if isinstance(value, bool):
            return ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                  bool_value=value)
        elif isinstance(value, int):
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                  integer_value=value)
        elif isinstance(value, float):
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                  double_value=value)
        elif isinstance(value, str):
            return ParameterValue(type=ParameterType.PARAMETER_STRING,
                                  string_value=value)
        elif isinstance(value, bytes):
            return ParameterValue(type=ParameterType.PARAMETER_BYTE_ARRAY,
                                  byte_array_value=list(value))
        elif isinstance(value, (list, tuple)):
            if len(value) == 0:
                return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                      string_array_value=[])
            else:
                if all(isinstance(v, bool) for v in value):
                    return ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY,
                                        bool_array_value=list(value))
                elif all(isinstance(v, int) and not isinstance(v, bool) for v in value):
                    return ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                        integer_array_value=list(value))
                elif all(isinstance(v, (float, int)) for v in value):
                    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                        double_array_value=[float(v) for v in value])
                elif all(isinstance(v, str) for v in value):
                    return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                        string_array_value=list(value))
                else:
                    self.node.destroy_node()
                    raise ValueError("Unsupported array element types")
        else:
            self.node.destroy_node()
            raise ValueError("Unsupported parameter type")



def main():
    """
    示例脚本，演示如何使用 node_params 类来设置参数。
    """
    # 步骤1：确保ROS2环境已被正确加载
    # 在终端中执行以下命令：
    # 或者 workspace 根目录下： source install/setup.bash
    # 步骤2：确认目标节点正在运行
    # 例如，运行 turtlesim 节点：
    # ros2 run turtlesim turtlesim_node
    # 步骤3：运行此脚本以修改参数:
    # python3 interfaces_py/interfaces.py

    # 要修改的目标节点名称（不含前导斜杠）
    target_node = "turtlesim"

    # 使用实例方法（你的 current 文件是通过构造函数创建 client）
    np = node_params(target_node)
    value = 1

    while rclpy.ok():
        # 读取参数(不为空表示该参数已经更新)
        if np.param is not None:
            print("Parameter read: ", np.read_value())
        # 设置一个Int参数
        np.set_param("background_b", value)
        # 读取该参数以验证
        np.get_param("background_b")
        value += 10
        if value > 255:
            value = 0
        time.sleep(2.0)

    np.node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()