"""TG-9801 ROS2 节点入口。

用于把串口通信能力封装成 ROS2 接口，对外提供夹取服务、原始字节透传和 JSON 命令调用能力。
"""

import json
from dataclasses import asdict, is_dataclass
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .client import TG9801Client, TG9801SerialConfig
from .protocol import bytes_to_hex, parse_number_string


class TG9801Node(Node):
    """TG-9801 夹爪 ROS2 节点。

    功能：
    1) 通过参数配置串口与协议；
    2) 提供 /set_grip 服务进行夹取/松开；
    3) 订阅 /raw_tx 发送原始字节；
    4) 发布 /rx_hex 回传串口接收数据（HEX）。
    """

    def __init__(self) -> None:
        super().__init__('tg_9801_node')

        # 允许 serial_parity 动态类型，避免 launch YAML 把 N 解析成布尔值时触发类型冲突。
        serial_parity_desc = ParameterDescriptor(dynamic_typing=True)

        # -------- 基础参数声明 --------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('serial_parity', 'N', serial_parity_desc)
        self.declare_parameter('serial_stopbits', 1.0)
        self.declare_parameter('serial_bytesize', 8)
        self.declare_parameter('use_rs485_mode', False)
        self.declare_parameter('rs485_rts_level_for_tx', True)
        self.declare_parameter('rs485_rts_level_for_rx', False)
        self.declare_parameter('rs485_delay_before_tx', 0.0)
        self.declare_parameter('rs485_delay_before_rx', 0.0)

        # -------- 协议参数声明 --------
        self.declare_parameter('protocol', 'modbus_rtu')
        self.declare_parameter('device_address', 1)
        self.declare_parameter('modbus_start_addr', 0x000A)
        self.declare_parameter('rs485_release_cmd', 1)

        # -------- 读取参数 --------
        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        timeout = float(self.get_parameter('timeout').value)
        serial_parity = self._normalize_serial_parity(self.get_parameter('serial_parity').value)
        serial_stopbits = float(self.get_parameter('serial_stopbits').value)
        serial_bytesize = int(self.get_parameter('serial_bytesize').value)
        use_rs485_mode = bool(self.get_parameter('use_rs485_mode').value)
        rs485_rts_level_for_tx = bool(self.get_parameter('rs485_rts_level_for_tx').value)
        rs485_rts_level_for_rx = bool(self.get_parameter('rs485_rts_level_for_rx').value)
        rs485_delay_before_tx = float(self.get_parameter('rs485_delay_before_tx').value)
        rs485_delay_before_rx = float(self.get_parameter('rs485_delay_before_rx').value)

        self._protocol = str(self.get_parameter('protocol').value)
        self._device_addr = int(self.get_parameter('device_address').value)
        self._modbus_start_addr = int(self.get_parameter('modbus_start_addr').value)
        self._rs485_release_cmd = int(self.get_parameter('rs485_release_cmd').value)

        # 创建统一通信客户端并打开串口。
        self._client = TG9801Client(TG9801SerialConfig(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=serial_parity,
            stopbits=serial_stopbits,
            bytesize=serial_bytesize,
            use_rs485_mode=use_rs485_mode,
            rs485_rts_level_for_tx=rs485_rts_level_for_tx,
            rs485_rts_level_for_rx=rs485_rts_level_for_rx,
            rs485_delay_before_tx=rs485_delay_before_tx,
            rs485_delay_before_rx=rs485_delay_before_rx,
        ))
        self._client.open()

        self._runtime_protocol = self._protocol
        self._runtime_device_addr = self._device_addr
        self._runtime_modbus_start_addr = self._modbus_start_addr
        self._runtime_rs485_release_cmd = self._rs485_release_cmd

        # 启动日志：用于确认当前生效参数。
        self.get_logger().info(
            f'TG-9801 node started | port={port} baud={baudrate} protocol={self._runtime_protocol} '
            f'parity={serial_parity} stopbits={serial_stopbits} bytesize={serial_bytesize} '
            f'use_rs485_mode={use_rs485_mode} rs485_release_cmd=0x{self._runtime_rs485_release_cmd:02x}'
        )

        # ROS 接口：发布串口接收数据、接收原始发送命令、提供夹爪服务。
        self._rx_pub = self.create_publisher(String, 'rx_hex', 10)
        self._command_result_pub = self.create_publisher(String, 'response_json', 10)
        self.create_subscription(String, 'raw_tx', self._on_raw_tx, 10)
        self.create_subscription(String, 'command_json', self._on_command_json, 10)
        self.create_service(SetBool, 'set_grip', self._on_set_grip)

        # 定时轮询串口接收缓冲。
        self._timer = self.create_timer(0.02, self._poll_rx)


    def _serialize_result(self, result: object) -> object:
        """将命令返回值转换为 JSON 兼容对象。"""
        if isinstance(result, bytes):
            return bytes_to_hex(result)
        if is_dataclass(result):
            return asdict(result)
        if isinstance(result, tuple):
            return list(result)
        return result

    def _execute_command(self, command: dict) -> object:
        """执行 command_json 中指定的动作。"""
        action = str(command.get('action', '')).strip().lower()
        if not action:
            raise ValueError('缺少 action 字段')

        protocol = str(command.get('protocol', self._runtime_protocol)).strip()
        device_addr = int(command.get('device_addr', self._runtime_device_addr))
        modbus_start_addr = int(command.get('modbus_start_addr', self._runtime_modbus_start_addr))
        rs485_release_cmd = int(command.get('rs485_release_cmd', self._runtime_rs485_release_cmd))
        speed = int(command.get('speed', 1000))
        target_force = int(command.get('target_force', 50))
        new_device_addr = int(command.get('new_device_addr', 2))
        position = int(command.get('position', 1000))
        register_addr = int(command.get('register_addr', modbus_start_addr))
        register_value = int(command.get('register_value', 1))
        register_values = [int(value) for value in command.get('register_values', [register_value])]
        read_start_addr = int(command.get('read_start_addr', 0))
        read_count = int(command.get('read_count', 1))
        reserved = int(command.get('reserved', 0))
        protocol_mode = str(command.get('protocol_mode', 'modbus')).strip().lower()
        position_mode = str(command.get('position_mode', 'position')).strip().lower()
        baudrate_value = int(command.get('baudrate_value', self._client.config.baudrate))

        if action == 'grip':
            return self._client.grip(
                protocol=protocol,
                device_addr=device_addr,
                modbus_start_addr=modbus_start_addr,
                rs485_release_cmd=rs485_release_cmd,
                speed=speed,
            )
        if action == 'release':
            return self._client.release(
                protocol=protocol,
                device_addr=device_addr,
                modbus_start_addr=modbus_start_addr,
                rs485_release_cmd=rs485_release_cmd,
                speed=speed,
            )
        if action == 'get_id':
            if protocol == 'rs485_custom':
                return self._client.rs485_get_id(device_addr, read_response=True)
            return self._client.modbus_read_holding_registers(device_addr, 0x000D, 1)[0]
        if action == 'get_serial':
            return self._client.get_factory_serial(protocol, device_addr)
        if action == 'get_status':
            return self._client.get_device_state(protocol, device_addr)
        if action == 'get_finger':
            return self._client.rs485_get_finger_data(device_addr, reserved=reserved)
        if action == 'get_force':
            return self._client.get_target_force(protocol, device_addr)
        if action == 'set_force':
            return self._client.set_target_force(protocol, device_addr, target_force)
        if action == 'set_id':
            if protocol == 'rs485_custom':
                return self._client.rs485_set_id(device_addr, new_device_addr)
            return self._client.modbus_set_device_id(device_addr, new_device_addr)
        if action == 'set_protocol':
            use_modbus = protocol_mode == 'modbus'
            if protocol == 'rs485_custom':
                return self._client.rs485_set_protocol(device_addr, use_modbus)
            return self._client.modbus_set_protocol(device_addr, use_modbus)
        if action == 'calibrate':
            if protocol == 'rs485_custom':
                return self._client.rs485_force_calibration(device_addr)
            return self._client.modbus_force_calibration(device_addr)
        if action == 'set_position_mode':
            return self._client.set_position_mode(protocol, device_addr, position_mode == 'position')
        if action == 'position':
            return self._client.send_position(protocol, device_addr, speed=speed, position=position)
        if action == 'get_follow_status':
            return self._client.modbus_get_follow_status(device_addr)
        if action == 'hardware_zero':
            if protocol == 'rs485_custom':
                return self._client.rs485_hardware_zero(device_addr)
            return self._client.modbus_hardware_zero(device_addr, register_value)
        if action == 'record_zero':
            if protocol == 'rs485_custom':
                return self._client.rs485_record_zero(device_addr)
            return self._client.modbus_record_zero(device_addr, register_value)
        if action == 'set_baudrate':
            return self._client.rs485_set_baudrate(device_addr, baudrate_value, read_response=True)
        if action == 'read_holding':
            return self._client.modbus_read_holding_registers(device_addr, read_start_addr, read_count)
        if action == 'read_input':
            return self._client.modbus_read_input_registers(device_addr, read_start_addr, read_count)
        if action == 'write_single':
            return self._client.modbus_write_single_register(device_addr, register_addr, register_value)
        if action == 'write_multi':
            return self._client.modbus_write_multiple_registers(device_addr, register_addr, register_values)
        if action == 'set_speed':
            return self._client.modbus_set_speed(device_addr, speed)
        if action == 'set_command':
            return self._client.modbus_set_command(device_addr, register_value)
        raise ValueError(f'不支持的 action: {action}')


    def _build_and_send_action(self, grip: bool) -> bytes:
        """根据 grip 标志发送夹取或松开，并返回发送帧。"""
        if grip:
            return self._client.grip(
                protocol=self._runtime_protocol,
                device_addr=self._runtime_device_addr,
                modbus_start_addr=self._runtime_modbus_start_addr,
                rs485_release_cmd=self._runtime_rs485_release_cmd,
            )
        return self._client.release(
            protocol=self._runtime_protocol,
            device_addr=self._runtime_device_addr,
            modbus_start_addr=self._runtime_modbus_start_addr,
            rs485_release_cmd=self._runtime_rs485_release_cmd,
        )

    @staticmethod
    def _normalize_serial_parity(value: object) -> str:
        """将串口 parity 参数归一化到 N/E/O。

        兼容场景：
        - YAML 将 N 解析为 False；
        - 用户输入 NONE/NO/EVEN/ODD 等别名。
        """
        if value is None:
            return 'N'
        if isinstance(value, bool):
            return 'N' if value is False else 'O'

        parity = str(value).strip().upper()
        if not parity:
            return 'N'

        alias = {
            'NONE': 'N',
            'NO': 'N',
            'FALSE': 'N',
            'EVEN': 'E',
            'ODD': 'O',
            'TRUE': 'O',
        }
        parity = alias.get(parity, parity)
        if parity not in ('N', 'E', 'O'):
            return 'N'
        return parity

    def _on_set_grip(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """/set_grip 服务回调：true=夹取，false=松开。"""
        try:
            frame = self._build_and_send_action(request.data)
            sent = len(frame)
            response.success = True
            response.message = (
                f"已发送 {sent} 字节 | {'grip' if request.data else 'release'} | {bytes_to_hex(frame)}"
            )
        except Exception as exc:
            response.success = False
            response.message = f'发送失败: {exc}'
            self.get_logger().error(response.message)
        return response

    def _on_raw_tx(self, msg: String) -> None:
        """/raw_tx 订阅回调：解析字符串数字并直接下发原始字节。"""
        try:
            data = parse_number_string(msg.data)
            if not data:
                return
            sent = self._client.send_raw(data)
            self.get_logger().info(f'raw tx {sent} bytes | {bytes_to_hex(data)}')
        except Exception as exc:
            self.get_logger().error(f'raw tx failed: {exc}')

    def _on_command_json(self, msg: String) -> None:
        """处理 JSON 命令请求并发布 JSON 结果。"""
        reply = String()
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError('command_json 需要 JSON object')
            result = self._execute_command(command)
            reply.data = json.dumps({
                'ok': True,
                'action': command.get('action'),
                'result': self._serialize_result(result),
            }, ensure_ascii=False)
            self._command_result_pub.publish(reply)
        except Exception as exc:
            reply.data = json.dumps({
                'ok': False,
                'error': str(exc),
            }, ensure_ascii=False)
            self._command_result_pub.publish(reply)
            self.get_logger().error(f'command_json failed: {exc}')

    def _poll_rx(self) -> None:
        """定时读取串口接收数据并发布到 /rx_hex。"""
        try:
            data = self._client.read_available()
            if not data:
                return
            out = String()
            out.data = bytes_to_hex(data)
            self._rx_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f'rx poll failed: {exc}')

    def destroy_node(self) -> bool:
        """节点销毁时关闭串口资源。"""
        try:
            self._client.close()
        finally:
            return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = TG9801Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
