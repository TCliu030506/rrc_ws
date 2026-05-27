"""TG-9801 协议构帧与响应解析工具。

用于集中定义 RS485 自定义协议和 Modbus RTU 的报文构造、CRC/校验计算以及响应解析逻辑。
"""

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


RS485_HEADER = bytes([0xA3, 0xB4])

RS485_CMD_GET_ID = 0x6A
RS485_CMD_SET_ID = 0x6B
RS485_CMD_SET_PROTOCOL = 0x6F
RS485_CMD_GRIP = 0x03
RS485_CMD_RELEASE = 0x01
RS485_CMD_GET_FACTORY_SERIAL = 0x75
RS485_CMD_FORCE_CALIBRATION = 0x64
RS485_CMD_SET_TARGET_FORCE = 0x7A
RS485_CMD_GET_TARGET_FORCE = 0x7B
RS485_CMD_GET_DEVICE_INFO = 0x08
RS485_CMD_GET_FINGER_DATA = 0x52
RS485_CMD_GET_FINGER_DATA_RESPONSE = 0x53
RS485_CMD_SET_POSITION_MODE = 0x02
RS485_CMD_SEND_POSITION = 0x10
RS485_CMD_HARDWARE_ZERO = 0x12
RS485_CMD_SET_BAUDRATE = 0x7F
RS485_CMD_RECORD_ZERO = 0x6C

MODBUS_FC_READ_HOLDING = 0x03
MODBUS_FC_READ_INPUT = 0x04
MODBUS_FC_WRITE_SINGLE = 0x06
MODBUS_FC_WRITE_MULTI = 0x10

MODBUS_REG_X1 = 0x0000
MODBUS_REG_Y1 = 0x0001
MODBUS_REG_Z1 = 0x0002
MODBUS_REG_X2 = 0x0003
MODBUS_REG_Y2 = 0x0004
MODBUS_REG_Z2 = 0x0005
MODBUS_REG_STATUS = 0x0006
MODBUS_REG_HARDNESS = 0x0007
MODBUS_REG_POSITION = 0x0008
MODBUS_REG_CURRENT = 0x0009
MODBUS_REG_SPEED = 0x000A
MODBUS_REG_COMMAND = 0x000B
MODBUS_REG_PROTOCOL = 0x000C
MODBUS_REG_DEVICE_ID = 0x000D
MODBUS_REG_CONTROL_MODE = 0x000E
MODBUS_REG_FOLLOW_SPEED = 0x000F
MODBUS_REG_FOLLOW_POSITION = 0x0010
MODBUS_REG_FORCE_CALIBRATION = 0x0011
MODBUS_REG_TARGET_FORCE = 0x0012
MODBUS_REG_FACTORY_SERIAL_LEN = 0x0013
MODBUS_REG_FOLLOW_STATUS = 0x001A
MODBUS_REG_HARDWARE_ZERO = 0x001D
MODBUS_REG_RECORD_ZERO = 0x001F


@dataclass(frozen=True)
class TG9801RS485Response:
    """RS485 自定义协议响应帧。"""

    addr: int
    command: int
    payload: bytes
    checksum: int
    raw: bytes


@dataclass(frozen=True)
class TG9801ModbusResponse:
    """Modbus RTU 响应帧。"""

    unit_id: int
    function_code: int
    data: bytes
    crc: int
    raw: bytes


@dataclass(frozen=True)
class TG9801StatusSnapshot:
    """夹爪基础状态快照。"""

    status: int
    hardness: int
    position: int
    current: int


@dataclass(frozen=True)
class TG9801FingerData:
    """夹指六维数据。"""

    x1: int
    x2: int
    y1: int
    y2: int
    z1: int
    z2: int
    reserved: int = 0


@dataclass(frozen=True)
class TG9801DeviceInfo:
    """Modbus 设备信息寄存器块解析结果。"""

    x1: int
    y1: int
    z1: int
    x2: int
    y2: int
    z2: int
    status: int
    hardness: int
    position: int
    current: int


def _require_byte(name: str, value: int, upper: int = 0xFF) -> None:
    if not (0 <= value <= upper):
        raise ValueError(f'{name} 范围应为 0..{upper}')


def _require_u16(name: str, value: int) -> None:
    if not (0 <= value <= 0xFFFF):
        raise ValueError(f'{name} 范围应为 0..65535')


def _require_range(name: str, value: int, lower: int, upper: int) -> None:
    if not (lower <= value <= upper):
        raise ValueError(f'{name} 范围应为 {lower}..{upper}')


def _u16_to_be(value: int) -> bytes:
    _require_u16('u16', value)
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def _u16_to_le(value: int) -> bytes:
    _require_u16('u16', value)
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def _u32_to_le(value: int) -> bytes:
    if not (0 <= value <= 0xFFFFFFFF):
        raise ValueError('u32 范围应为 0..4294967295')
    return bytes([
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ])


def _decode_u16_be(high: int, low: int) -> int:
    return ((high & 0xFF) << 8) | (low & 0xFF)


def _decode_u16_le(low: int, high: int) -> int:
    return ((high & 0xFF) << 8) | (low & 0xFF)


def _decode_i16_be(high: int, low: int) -> int:
    value = _decode_u16_be(high, low)
    return value - 0x10000 if value & 0x8000 else value


def _decode_i16_le(low: int, high: int) -> int:
    value = _decode_u16_le(low, high)
    return value - 0x10000 if value & 0x8000 else value


def _rs485_checksum(length: int, addr: int, command: int, payload: bytes) -> int:
    return (length + addr + command + sum(payload)) & 0xFF


def build_rs485_frame(addr: int, command: int, payload: bytes = b'') -> bytes:
    """构造一帧 RS485 自定义协议报文。"""
    _require_byte('地址', addr)
    _require_byte('命令', command)
    if len(payload) > 253:
        raise ValueError('payload 过长')

    length = 2 + len(payload)
    checksum = _rs485_checksum(length, addr, command, payload)
    return RS485_HEADER + bytes([length, addr, command]) + payload + bytes([checksum])


def parse_rs485_response(frame: bytes) -> TG9801RS485Response:
    """解析并校验 RS485 自定义协议响应帧。"""
    if len(frame) < 6:
        raise ValueError('RS485 响应长度不足')
    if frame[:2] != RS485_HEADER:
        raise ValueError('RS485 帧头错误')

    length = frame[2]
    expected_size = 4 + length
    if len(frame) != expected_size:
        raise ValueError(f'RS485 响应长度不匹配: expected={expected_size}, actual={len(frame)}')

    addr = frame[3]
    command = frame[4]
    payload = frame[5:-1]
    checksum = frame[-1]
    expected_checksum = _rs485_checksum(length, addr, command, payload)
    if checksum != expected_checksum:
        raise ValueError(
            f'RS485 校验和错误: expected=0x{expected_checksum:02X}, actual=0x{checksum:02X}'
        )
    return TG9801RS485Response(addr=addr, command=command, payload=payload, checksum=checksum, raw=frame)


def build_rs485_custom(addr: int, grip: bool, release_cmd: int = 0x01, speed: int = 1000) -> bytes:
    """构造 RS485 自定义协议动作帧（夹取/松开）。"""
    if grip:
        _require_range('速度', speed, 200, 1500)
        return build_rs485_frame(addr, RS485_CMD_GRIP, _u16_to_le(speed))
    if release_cmd not in (0x00, 0x01):
        raise ValueError('release_cmd 仅支持 0x00 或 0x01')
    return build_rs485_frame(addr, release_cmd, b'')


def build_rs485_set_protocol(addr: int, use_modbus: bool) -> bytes:
    """构造设置协议模式帧。"""
    return build_rs485_frame(addr, RS485_CMD_SET_PROTOCOL, bytes([0x02 if use_modbus else 0x01, 0x00]))


def build_rs485_get_id(addr: int = 0x00) -> bytes:
    """构造获取设备 ID 帧。"""
    return build_rs485_frame(addr, RS485_CMD_GET_ID)


def build_rs485_set_id(addr: int, new_addr: int) -> bytes:
    """构造修改设备 ID 帧。"""
    _require_byte('新地址', new_addr)
    return build_rs485_frame(addr, RS485_CMD_SET_ID, bytes([new_addr]))


def build_rs485_force_calibration(addr: int) -> bytes:
    """构造强制校准帧。"""
    return build_rs485_frame(addr, RS485_CMD_FORCE_CALIBRATION)


def build_rs485_get_factory_serial(addr: int) -> bytes:
    """构造获取出厂编号帧。"""
    return build_rs485_frame(addr, RS485_CMD_GET_FACTORY_SERIAL)


def build_rs485_set_target_force(addr: int, target_force: int) -> bytes:
    """构造设置目标夹取力帧。"""
    _require_range('目标夹取力', target_force, 1, 100)
    return build_rs485_frame(addr, RS485_CMD_SET_TARGET_FORCE, _u16_to_le(target_force))


def build_rs485_get_target_force(addr: int) -> bytes:
    """构造读取目标夹取力帧。"""
    return build_rs485_frame(addr, RS485_CMD_GET_TARGET_FORCE)


def build_rs485_get_device_info(addr: int) -> bytes:
    """构造读取夹爪设备信息帧。"""
    return build_rs485_frame(addr, RS485_CMD_GET_DEVICE_INFO)


def build_rs485_get_finger_data(addr: int, reserved: int = 0x00) -> bytes:
    """构造读取末端夹指数据帧。"""
    _require_byte('保留字段', reserved)
    return build_rs485_frame(addr, RS485_CMD_GET_FINGER_DATA, bytes([reserved]))


def build_rs485_set_position_mode(addr: int, position_mode: bool) -> bytes:
    """构造位置模式切换帧。"""
    return build_rs485_frame(addr, RS485_CMD_SET_POSITION_MODE, bytes([0x01 if position_mode else 0x00]))


def build_rs485_send_position(addr: int, speed: int = 1000, position: int = 1000) -> bytes:
    """构造位置模式指令帧。"""
    _require_range('位置模式速度', speed, 200, 1500)
    _require_range('目标位置', position, 0, 1000)
    payload = bytes([0x01]) + _u16_to_le(speed) + _u16_to_le(position) + bytes([0xFF, 0xFF])
    return build_rs485_frame(addr, RS485_CMD_SEND_POSITION, payload)


def build_rs485_hardware_zero(addr: int) -> bytes:
    """构造末端硬件清零帧。"""
    return build_rs485_frame(addr, RS485_CMD_HARDWARE_ZERO)


def build_rs485_set_baudrate(addr: int, baudrate: int) -> bytes:
    """构造修改波特率帧。"""
    _require_range('波特率', baudrate, 115200, 1000000)
    return build_rs485_frame(addr, RS485_CMD_SET_BAUDRATE, _u32_to_le(baudrate))


def build_rs485_record_zero(addr: int) -> bytes:
    """构造记录末端零点帧。"""
    return build_rs485_frame(addr, RS485_CMD_RECORD_ZERO)


def parse_rs485_get_id_response(frame: bytes) -> int:
    """解析获取设备 ID 响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_GET_ID or response.payload:
        raise ValueError('不是有效的获取设备 ID 响应')
    return response.addr


def parse_rs485_factory_serial_response(frame: bytes) -> str:
    """解析 RS485 出厂编号响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_GET_FACTORY_SERIAL:
        raise ValueError('不是有效的获取出厂编号响应')
    return response.payload.decode('ascii', errors='replace').rstrip('\x00')


def parse_rs485_target_force_response(frame: bytes) -> int:
    """解析 RS485 目标力响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_GET_TARGET_FORCE or len(response.payload) != 2:
        raise ValueError('不是有效的目标夹取力响应')
    return _decode_u16_le(response.payload[0], response.payload[1])


def parse_rs485_status_snapshot_response(frame: bytes) -> TG9801StatusSnapshot:
    """解析 RS485 设备信息响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_GET_DEVICE_INFO or len(response.payload) != 7:
        raise ValueError('不是有效的设备信息响应')
    return TG9801StatusSnapshot(
        status=response.payload[0],
        hardness=_decode_u16_le(response.payload[1], response.payload[2]),
        position=_decode_u16_le(response.payload[3], response.payload[4]),
        current=_decode_u16_le(response.payload[5], response.payload[6]),
    )


def parse_rs485_finger_data_response(frame: bytes) -> TG9801FingerData:
    """解析 RS485 末端夹指数据响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_GET_FINGER_DATA_RESPONSE or len(response.payload) != 13:
        raise ValueError('不是有效的夹指数据响应')
    payload = response.payload
    return TG9801FingerData(
        reserved=payload[0],
        x1=_decode_i16_le(payload[1], payload[2]),
        x2=_decode_i16_le(payload[3], payload[4]),
        y1=_decode_i16_le(payload[5], payload[6]),
        y2=_decode_i16_le(payload[7], payload[8]),
        z1=_decode_i16_le(payload[9], payload[10]),
        z2=_decode_i16_le(payload[11], payload[12]),
    )


def parse_rs485_baudrate_response(frame: bytes) -> bool:
    """解析 RS485 修改波特率响应。"""
    response = parse_rs485_response(frame)
    if response.command != RS485_CMD_SET_BAUDRATE or len(response.payload) != 1:
        raise ValueError('不是有效的波特率设置响应')
    return response.payload[0] == 0x01


def modbus_crc16(data: bytes) -> int:
    """计算 Modbus RTU CRC16（低字节在前）。"""
    crc = 0xFFFF
    for byte_value in data:
        crc ^= byte_value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _build_modbus_pdu(unit_id: int, function_code: int, body: bytes) -> bytes:
    _require_byte('Modbus 设备地址', unit_id, upper=247)
    _require_byte('功能码', function_code)
    payload = bytes([unit_id, function_code]) + body
    crc = modbus_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_modbus_response(frame: bytes) -> TG9801ModbusResponse:
    """解析并校验 Modbus RTU 响应帧。"""
    if len(frame) < 5:
        raise ValueError('Modbus 响应长度不足')

    payload = frame[:-2]
    crc = frame[-2] | (frame[-1] << 8)
    expected_crc = modbus_crc16(payload)
    if crc != expected_crc:
        raise ValueError(f'Modbus CRC 错误: expected=0x{expected_crc:04X}, actual=0x{crc:04X}')

    unit_id = payload[0]
    function_code = payload[1]
    data = payload[2:]
    if function_code & 0x80:
        error_code = data[0] if data else -1
        raise ValueError(f'Modbus 异常响应: function=0x{function_code:02X}, error=0x{error_code:02X}')

    return TG9801ModbusResponse(unit_id=unit_id, function_code=function_code, data=data, crc=crc, raw=frame)


def build_modbus_read_holding_registers(unit_id: int, start_addr: int, count: int) -> bytes:
    """构造读取保持寄存器报文。"""
    _require_u16('起始地址', start_addr)
    _require_range('寄存器数量', count, 1, 125)
    return _build_modbus_pdu(unit_id, MODBUS_FC_READ_HOLDING, _u16_to_be(start_addr) + _u16_to_be(count))


def build_modbus_read_input_registers(unit_id: int, start_addr: int, count: int) -> bytes:
    """构造读取输入寄存器报文。"""
    _require_u16('起始地址', start_addr)
    _require_range('寄存器数量', count, 1, 125)
    return _build_modbus_pdu(unit_id, MODBUS_FC_READ_INPUT, _u16_to_be(start_addr) + _u16_to_be(count))


def build_modbus_write_single_register(unit_id: int, register_addr: int, value: int) -> bytes:
    """构造写单寄存器报文。"""
    _require_u16('寄存器地址', register_addr)
    _require_u16('寄存器值', value)
    return _build_modbus_pdu(unit_id, MODBUS_FC_WRITE_SINGLE, _u16_to_be(register_addr) + _u16_to_be(value))


def build_modbus_write_multiple_registers(unit_id: int, start_addr: int, values: Sequence[int]) -> bytes:
    """构造写多个寄存器报文。"""
    if not values:
        raise ValueError('values 不能为空')
    if len(values) > 123:
        raise ValueError('一次最多写 123 个寄存器')
    _require_u16('起始地址', start_addr)
    body = bytearray()
    body.extend(_u16_to_be(start_addr))
    body.extend(_u16_to_be(len(values)))
    body.append(len(values) * 2)
    for value in values:
        _require_u16('寄存器值', value)
        body.extend(_u16_to_be(value))
    return _build_modbus_pdu(unit_id, MODBUS_FC_WRITE_MULTI, bytes(body))


def build_modbus_write_regs(unit_id: int, start_addr: int, grip: bool, speed: int = 1000) -> bytes:
    """构造 Modbus RTU 夹取/松开帧。"""
    _require_range('速度', speed, 200, 1500)
    return build_modbus_write_multiple_registers(unit_id, start_addr, [speed, 0x0001 if grip else 0x0000])


def parse_modbus_read_registers_response(frame: bytes, expected_function: Optional[int] = None) -> list[int]:
    """解析 Modbus 读寄存器响应，返回寄存器列表。"""
    response = parse_modbus_response(frame)
    if expected_function is not None and response.function_code != expected_function:
        raise ValueError(
            f'功能码不匹配: expected=0x{expected_function:02X}, actual=0x{response.function_code:02X}'
        )
    if len(response.data) < 1:
        raise ValueError('读寄存器响应缺少字节计数')
    byte_count = response.data[0]
    register_bytes = response.data[1:]
    if byte_count != len(register_bytes) or byte_count % 2 != 0:
        raise ValueError('读寄存器响应字节计数非法')
    return [
        _decode_u16_be(register_bytes[index], register_bytes[index + 1])
        for index in range(0, len(register_bytes), 2)
    ]


def parse_modbus_write_ack(frame: bytes, expected_function: Optional[int] = None) -> tuple[int, int]:
    """解析 Modbus 写寄存器回显。"""
    response = parse_modbus_response(frame)
    if expected_function is not None and response.function_code != expected_function:
        raise ValueError(
            f'功能码不匹配: expected=0x{expected_function:02X}, actual=0x{response.function_code:02X}'
        )
    if len(response.data) != 4:
        raise ValueError('写寄存器响应长度非法')
    return (
        _decode_u16_be(response.data[0], response.data[1]),
        _decode_u16_be(response.data[2], response.data[3]),
    )


def parse_modbus_device_info_response(frame: bytes) -> TG9801DeviceInfo:
    """解析读取设备信息响应。"""
    registers = parse_modbus_read_registers_response(frame, expected_function=MODBUS_FC_READ_HOLDING)
    if len(registers) != 10:
        raise ValueError(f'设备信息寄存器数量错误: {len(registers)}')
    return TG9801DeviceInfo(
        x1=_decode_i16_be((registers[0] >> 8) & 0xFF, registers[0] & 0xFF),
        y1=_decode_i16_be((registers[1] >> 8) & 0xFF, registers[1] & 0xFF),
        z1=_decode_i16_be((registers[2] >> 8) & 0xFF, registers[2] & 0xFF),
        x2=_decode_i16_be((registers[3] >> 8) & 0xFF, registers[3] & 0xFF),
        y2=_decode_i16_be((registers[4] >> 8) & 0xFF, registers[4] & 0xFF),
        z2=_decode_i16_be((registers[5] >> 8) & 0xFF, registers[5] & 0xFF),
        status=registers[6],
        hardness=registers[7],
        position=registers[8],
        current=registers[9],
    )


def parse_modbus_factory_serial_response(frame: bytes) -> str:
    """解析 Modbus 读取出厂编号响应。"""
    registers = parse_modbus_read_registers_response(frame, expected_function=MODBUS_FC_READ_INPUT)
    if not registers:
        raise ValueError('出厂编号响应为空')
    serial_length = registers[0]
    raw = bytearray()
    for value in registers[1:]:
        raw.extend(_u16_to_be(value))
    if serial_length > len(raw):
        raise ValueError('出厂编号长度字段超过有效数据长度')
    return bytes(raw[:serial_length]).decode('ascii', errors='replace').rstrip('\x00')


def parse_modbus_target_force_response(frame: bytes) -> int:
    """解析 Modbus 目标力读取响应。"""
    registers = parse_modbus_read_registers_response(frame, expected_function=MODBUS_FC_READ_HOLDING)
    if len(registers) != 1:
        raise ValueError('目标夹取力寄存器数量错误')
    return registers[0]


def parse_modbus_follow_status_response(frame: bytes) -> int:
    """解析 Modbus 随动状态读取响应。"""
    registers = parse_modbus_read_registers_response(frame, expected_function=MODBUS_FC_READ_HOLDING)
    if len(registers) != 1:
        raise ValueError('随动状态寄存器数量错误')
    return registers[0]


def parse_number_string(num_str: str) -> bytes:
    """把数字字符串解析为 bytes。"""
    text = num_str.strip()
    if not text:
        return b''

    text = text.replace(',', ' ').replace(';', ' ')
    parts = [part for part in text.split() if part]
    values = []
    for part in parts:
        try:
            value = int(part, 0)
        except ValueError as exc:
            raise ValueError(f'无法解析数字: {part}') from exc
        if not (0 <= value <= 255):
            raise ValueError(f'数值超出字节范围(0-255): {value}')
        values.append(value)
    return bytes(values)


def bytes_to_hex(data: Iterable[int]) -> str:
    """将字节序列转换为空格分隔 HEX 字符串。"""
    return bytes(data).hex(' ')
