"""TG-9801 通信客户端封装层。

用于把协议构帧、串口收发和请求响应流程封装为高层 API，供 ROS2 节点、CLI 和测试工具复用。
"""

from dataclasses import dataclass
import time
from typing import Optional

from .protocol import (
    MODBUS_FC_READ_HOLDING,
    MODBUS_FC_READ_INPUT,
    MODBUS_FC_WRITE_MULTI,
    MODBUS_FC_WRITE_SINGLE,
    MODBUS_REG_COMMAND,
    MODBUS_REG_CONTROL_MODE,
    MODBUS_REG_DEVICE_ID,
    MODBUS_REG_FACTORY_SERIAL_LEN,
    MODBUS_REG_FOLLOW_POSITION,
    MODBUS_REG_FOLLOW_SPEED,
    MODBUS_REG_FOLLOW_STATUS,
    MODBUS_REG_FORCE_CALIBRATION,
    MODBUS_REG_HARDWARE_ZERO,
    MODBUS_REG_PROTOCOL,
    MODBUS_REG_RECORD_ZERO,
    MODBUS_REG_SPEED,
    MODBUS_REG_STATUS,
    MODBUS_REG_TARGET_FORCE,
    TG9801DeviceInfo,
    TG9801FingerData,
    TG9801StatusSnapshot,
    build_modbus_read_holding_registers,
    build_modbus_read_input_registers,
    build_modbus_write_multiple_registers,
    build_modbus_write_regs,
    build_modbus_write_single_register,
    build_rs485_custom,
    build_rs485_force_calibration,
    build_rs485_get_device_info,
    build_rs485_get_factory_serial,
    build_rs485_get_finger_data,
    build_rs485_get_id,
    build_rs485_get_target_force,
    build_rs485_hardware_zero,
    build_rs485_record_zero,
    build_rs485_send_position,
    build_rs485_set_baudrate,
    build_rs485_set_id,
    build_rs485_set_position_mode,
    build_rs485_set_protocol,
    build_rs485_set_target_force,
    parse_modbus_device_info_response,
    parse_modbus_factory_serial_response,
    parse_modbus_follow_status_response,
    parse_modbus_read_registers_response,
    parse_modbus_target_force_response,
    parse_modbus_write_ack,
    parse_rs485_baudrate_response,
    parse_rs485_factory_serial_response,
    parse_rs485_finger_data_response,
    parse_rs485_get_id_response,
    parse_rs485_status_snapshot_response,
    parse_rs485_target_force_response,
)
from .serial_transport import SerialTransport


@dataclass
class TG9801SerialConfig:
    """TG-9801 串口配置项。"""

    port: str = '/dev/ttyACM0'
    baudrate: int = 1000000
    timeout: float = 1.0
    parity: str = 'N'
    stopbits: float = 1.0
    bytesize: int = 8
    use_rs485_mode: bool = False
    rs485_rts_level_for_tx: bool = True
    rs485_rts_level_for_rx: bool = False
    rs485_delay_before_tx: float = 0.0
    rs485_delay_before_rx: float = 0.0


class TG9801Client:
    """TG-9801 高层客户端。"""

    def __init__(self, config: TG9801SerialConfig):
        self.config = config
        self.transport = SerialTransport(
            port=config.port,
            baudrate=config.baudrate,
            timeout=config.timeout,
            parity=config.parity,
            stopbits=config.stopbits,
            bytesize=config.bytesize,
            use_rs485_mode=config.use_rs485_mode,
            rs485_rts_level_for_tx=config.rs485_rts_level_for_tx,
            rs485_rts_level_for_rx=config.rs485_rts_level_for_rx,
            rs485_delay_before_tx=config.rs485_delay_before_tx,
            rs485_delay_before_rx=config.rs485_delay_before_rx,
        )

    @property
    def is_open(self) -> bool:
        """当前串口是否已打开。"""
        return self.transport.is_open

    def open(self) -> None:
        """打开串口连接。"""
        self.transport.open()

    def close(self) -> None:
        """关闭串口连接。"""
        self.transport.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def read_available(self) -> bytes:
        """读取当前可用接收缓冲区字节。"""
        return self.transport.read_available()

    def clear_buffers(self) -> None:
        """清空收发缓冲区，避免旧数据影响下一次请求。"""
        self.transport.clear_buffers()

    def send_raw(self, data: bytes) -> int:
        """发送原始字节并返回写入字节数。"""
        return self.transport.send(data)

    def read_response(self, response_timeout: Optional[float] = None, idle_gap: float = 0.05) -> bytes:
        """读取一次响应，直到超时或在收到首字节后进入空闲窗口。"""
        timeout = self.config.timeout if response_timeout is None else response_timeout
        deadline = time.monotonic() + max(0.0, timeout)
        idle_deadline: Optional[float] = None
        buffer = bytearray()

        while time.monotonic() < deadline:
            chunk = self.read_available()
            if chunk:
                buffer.extend(chunk)
                idle_deadline = time.monotonic() + max(0.0, idle_gap)
                continue
            if buffer and idle_deadline is not None and time.monotonic() >= idle_deadline:
                return bytes(buffer)
            time.sleep(0.005)

        if buffer:
            return bytes(buffer)
        raise TimeoutError(f'等待串口响应超时({timeout:.3f}s)')

    def exchange(self, frame: bytes, response_timeout: Optional[float] = None, idle_gap: float = 0.05) -> bytes:
        """执行一次请求-响应交互。"""
        self.clear_buffers()
        self.send_raw(frame)
        return self.read_response(response_timeout=response_timeout, idle_gap=idle_gap)

    def rs485_set_protocol(self, device_addr: int, use_modbus: bool) -> bytes:
        """发送设置协议模式帧。"""
        frame = build_rs485_set_protocol(device_addr, use_modbus=use_modbus)
        self.send_raw(frame)
        return frame

    def rs485_get_id(self, device_addr: int = 0x00, read_response: bool = False):
        """发送或读取设备 ID。"""
        frame = build_rs485_get_id(device_addr)
        if not read_response:
            self.send_raw(frame)
            return frame
        return parse_rs485_get_id_response(self.exchange(frame))

    def rs485_set_id(self, device_addr: int, new_device_addr: int) -> bytes:
        """发送修改设备 ID 帧。"""
        frame = build_rs485_set_id(device_addr, new_device_addr)
        self.send_raw(frame)
        return frame

    def rs485_force_calibration(self, device_addr: int) -> bytes:
        """发送强制校准帧。"""
        frame = build_rs485_force_calibration(device_addr)
        self.send_raw(frame)
        return frame

    def rs485_get_factory_serial(self, device_addr: int) -> str:
        """读取 RS485 出厂编号。"""
        frame = build_rs485_get_factory_serial(device_addr)
        return parse_rs485_factory_serial_response(self.exchange(frame))

    def rs485_set_target_force(self, device_addr: int, target_force: int) -> bytes:
        """发送设置目标夹取力帧。"""
        frame = build_rs485_set_target_force(device_addr, target_force)
        self.send_raw(frame)
        return frame

    def rs485_get_target_force(self, device_addr: int) -> int:
        """读取 RS485 目标夹取力。"""
        frame = build_rs485_get_target_force(device_addr)
        return parse_rs485_target_force_response(self.exchange(frame))

    def rs485_get_device_info(self, device_addr: int) -> TG9801StatusSnapshot:
        """读取 RS485 基础状态。"""
        frame = build_rs485_get_device_info(device_addr)
        return parse_rs485_status_snapshot_response(self.exchange(frame))

    def rs485_get_finger_data(self, device_addr: int, reserved: int = 0x00) -> TG9801FingerData:
        """读取 RS485 夹指六维数据。"""
        frame = build_rs485_get_finger_data(device_addr, reserved=reserved)
        return parse_rs485_finger_data_response(self.exchange(frame))

    def rs485_set_position_mode(self, device_addr: int, position_mode: bool) -> bytes:
        """发送位置模式切换帧。"""
        frame = build_rs485_set_position_mode(device_addr, position_mode)
        self.send_raw(frame)
        return frame

    def rs485_send_position(self, device_addr: int, speed: int = 1000, position: int = 1000) -> bytes:
        """发送 RS485 位置模式运动指令。"""
        frame = build_rs485_send_position(device_addr, speed=speed, position=position)
        self.send_raw(frame)
        return frame

    def rs485_hardware_zero(self, device_addr: int) -> bytes:
        """发送 RS485 末端硬件清零帧。"""
        frame = build_rs485_hardware_zero(device_addr)
        self.send_raw(frame)
        return frame

    def rs485_set_baudrate(self, device_addr: int, baudrate: int, read_response: bool = True):
        """设置 RS485 波特率。"""
        frame = build_rs485_set_baudrate(device_addr, baudrate)
        if not read_response:
            self.send_raw(frame)
            return frame
        return parse_rs485_baudrate_response(self.exchange(frame))

    def rs485_record_zero(self, device_addr: int) -> bytes:
        """发送 RS485 记录零点帧。"""
        frame = build_rs485_record_zero(device_addr)
        self.send_raw(frame)
        return frame

    def modbus_read_holding_registers(self, device_addr: int, start_addr: int, count: int) -> list[int]:
        """读取保持寄存器。"""
        frame = build_modbus_read_holding_registers(device_addr, start_addr, count)
        return parse_modbus_read_registers_response(
            self.exchange(frame),
            expected_function=MODBUS_FC_READ_HOLDING,
        )

    def modbus_read_input_registers(self, device_addr: int, start_addr: int, count: int) -> list[int]:
        """读取输入寄存器。"""
        frame = build_modbus_read_input_registers(device_addr, start_addr, count)
        return parse_modbus_read_registers_response(
            self.exchange(frame),
            expected_function=MODBUS_FC_READ_INPUT,
        )

    def modbus_write_single_register(self, device_addr: int, register_addr: int, value: int) -> tuple[int, int]:
        """写单寄存器并校验回显。"""
        frame = build_modbus_write_single_register(device_addr, register_addr, value)
        return parse_modbus_write_ack(
            self.exchange(frame),
            expected_function=MODBUS_FC_WRITE_SINGLE,
        )

    def modbus_write_multiple_registers(self, device_addr: int, start_addr: int, values: list[int]) -> tuple[int, int]:
        """写多个寄存器并校验回显。"""
        frame = build_modbus_write_multiple_registers(device_addr, start_addr, values)
        return parse_modbus_write_ack(
            self.exchange(frame),
            expected_function=MODBUS_FC_WRITE_MULTI,
        )

    def modbus_set_protocol(self, device_addr: int, use_modbus: bool) -> tuple[int, int]:
        """通过 Modbus 设置协议寄存器。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_PROTOCOL, 0x0002 if use_modbus else 0x0001)

    def modbus_set_device_id(self, device_addr: int, new_device_addr: int) -> tuple[int, int]:
        """通过 Modbus 设置设备地址。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_DEVICE_ID, new_device_addr)

    def modbus_set_speed(self, device_addr: int, speed: int) -> tuple[int, int]:
        """通过 Modbus 设置夹取速度寄存器。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_SPEED, speed)

    def modbus_set_command(self, device_addr: int, command_value: int) -> tuple[int, int]:
        """通过 Modbus 设置命令寄存器。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_COMMAND, command_value)

    def modbus_force_calibration(self, device_addr: int) -> tuple[int, int]:
        """通过 Modbus 触发强制校准。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_FORCE_CALIBRATION, 0x0001)

    def modbus_set_target_force(self, device_addr: int, target_force: int) -> tuple[int, int]:
        """通过 Modbus 设置目标夹取力。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_TARGET_FORCE, target_force)

    def modbus_get_target_force(self, device_addr: int) -> int:
        """读取 Modbus 目标夹取力。"""
        frame = build_modbus_read_holding_registers(device_addr, MODBUS_REG_TARGET_FORCE, 1)
        return parse_modbus_target_force_response(self.exchange(frame))

    def modbus_get_device_info(self, device_addr: int) -> TG9801DeviceInfo:
        """读取 Modbus 设备信息。"""
        frame = build_modbus_read_holding_registers(device_addr, MODBUS_REG_STATUS - 6, 10)
        return parse_modbus_device_info_response(self.exchange(frame))

    def modbus_get_factory_serial(self, device_addr: int) -> str:
        """读取 Modbus 出厂编号。"""
        frame = build_modbus_read_input_registers(device_addr, MODBUS_REG_FACTORY_SERIAL_LEN, 6)
        return parse_modbus_factory_serial_response(self.exchange(frame))

    def modbus_set_position_mode(self, device_addr: int, position_mode: bool) -> tuple[int, int]:
        """设置 Modbus 控制模式。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_CONTROL_MODE, 0x0001 if position_mode else 0x0000)

    def modbus_send_position(self, device_addr: int, speed: int = 1000, position: int = 1000) -> tuple[int, int]:
        """发送 Modbus 位置模式运动指令。"""
        return self.modbus_write_multiple_registers(device_addr, MODBUS_REG_FOLLOW_SPEED, [speed, position])

    def modbus_get_follow_status(self, device_addr: int) -> int:
        """读取 Modbus 随动状态。"""
        frame = build_modbus_read_holding_registers(device_addr, MODBUS_REG_FOLLOW_STATUS, 1)
        return parse_modbus_follow_status_response(self.exchange(frame))

    def modbus_hardware_zero(self, device_addr: int, value: int = 0x0001) -> tuple[int, int]:
        """触发 Modbus 末端硬件清零。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_HARDWARE_ZERO, value)

    def modbus_record_zero(self, device_addr: int, value: int = 0x0001) -> tuple[int, int]:
        """触发 Modbus 记录零点。"""
        return self.modbus_write_single_register(device_addr, MODBUS_REG_RECORD_ZERO, value)

    def grip(
        self,
        protocol: str = 'rs485_custom',
        device_addr: int = 1,
        modbus_start_addr: int = MODBUS_REG_SPEED,
        rs485_release_cmd: int = 0x01,
        speed: int = 1000,
    ) -> bytes:
        """发送夹取命令并返回发送帧。"""
        if protocol == 'rs485_custom':
            frame = build_rs485_custom(device_addr, True, release_cmd=rs485_release_cmd, speed=speed)
            self.send_raw(frame)
            return frame
        if protocol == 'modbus_rtu':
            frame = build_modbus_write_regs(device_addr, modbus_start_addr, True, speed=speed)
            self.send_raw(frame)
            return frame
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def release(
        self,
        protocol: str = 'rs485_custom',
        device_addr: int = 1,
        modbus_start_addr: int = MODBUS_REG_SPEED,
        rs485_release_cmd: int = 0x01,
        speed: int = 1000,
    ) -> bytes:
        """发送松开命令并返回发送帧。"""
        if protocol == 'rs485_custom':
            frame = build_rs485_custom(device_addr, False, release_cmd=rs485_release_cmd, speed=speed)
            self.send_raw(frame)
            return frame
        if protocol == 'modbus_rtu':
            frame = build_modbus_write_regs(device_addr, modbus_start_addr, False, speed=speed)
            self.send_raw(frame)
            return frame
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def set_target_force(self, protocol: str, device_addr: int, target_force: int):
        """跨协议设置目标夹取力。"""
        if protocol == 'rs485_custom':
            return self.rs485_set_target_force(device_addr, target_force)
        if protocol == 'modbus_rtu':
            return self.modbus_set_target_force(device_addr, target_force)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def get_target_force(self, protocol: str, device_addr: int) -> int:
        """跨协议读取目标夹取力。"""
        if protocol == 'rs485_custom':
            return self.rs485_get_target_force(device_addr)
        if protocol == 'modbus_rtu':
            return self.modbus_get_target_force(device_addr)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def get_factory_serial(self, protocol: str, device_addr: int) -> str:
        """跨协议读取出厂编号。"""
        if protocol == 'rs485_custom':
            return self.rs485_get_factory_serial(device_addr)
        if protocol == 'modbus_rtu':
            return self.modbus_get_factory_serial(device_addr)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def get_device_state(self, protocol: str, device_addr: int):
        """跨协议读取设备状态。"""
        if protocol == 'rs485_custom':
            return self.rs485_get_device_info(device_addr)
        if protocol == 'modbus_rtu':
            return self.modbus_get_device_info(device_addr)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def set_position_mode(self, protocol: str, device_addr: int, position_mode: bool):
        """跨协议切换控制模式。"""
        if protocol == 'rs485_custom':
            return self.rs485_set_position_mode(device_addr, position_mode)
        if protocol == 'modbus_rtu':
            return self.modbus_set_position_mode(device_addr, position_mode)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")

    def send_position(self, protocol: str, device_addr: int, speed: int = 1000, position: int = 1000):
        """跨协议发送位置模式运动指令。"""
        if protocol == 'rs485_custom':
            return self.rs485_send_position(device_addr, speed=speed, position=position)
        if protocol == 'modbus_rtu':
            return self.modbus_send_position(device_addr, speed=speed, position=position)
        raise ValueError("protocol 仅支持 'rs485_custom' 或 'modbus_rtu'")
