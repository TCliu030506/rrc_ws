"""TG-9801 协议层单元测试。

用于校验关键构帧结果和响应解析是否与当前协议文档保持一致。
"""

from tg_9801_pkg.protocol import (
    build_modbus_read_input_registers,
    build_modbus_write_regs,
    build_rs485_custom,
    modbus_crc16,
    parse_modbus_device_info_response,
    parse_modbus_factory_serial_response,
    parse_rs485_finger_data_response,
    parse_rs485_status_snapshot_response,
)


def _with_crc(payload: bytes) -> bytes:
    crc = modbus_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def test_build_rs485_grip_frame_matches_protocol_sheet() -> None:
    frame = build_rs485_custom(addr=1, grip=True, speed=1000)
    assert frame == bytes.fromhex('A3 B4 04 01 03 E8 03 F3')


def test_build_modbus_grip_frame_matches_protocol_sheet() -> None:
    frame = build_modbus_write_regs(unit_id=1, start_addr=0x000A, grip=True, speed=1000)
    assert frame == bytes.fromhex('01 10 00 0A 00 02 04 03 E8 00 01 32 60')


def test_parse_rs485_status_snapshot_response() -> None:
    response = bytes.fromhex('A3 B4 09 01 08 00 64 00 AC 0D 32 00 61')
    parsed = parse_rs485_status_snapshot_response(response)
    assert parsed.status == 0
    assert parsed.hardness == 100
    assert parsed.position == 3500
    assert parsed.current == 50


def test_parse_rs485_finger_data_response() -> None:
    response = bytes.fromhex('A3 B4 0F 01 53 01 F4 FF 19 00 7E 00 A3 FF 55 00 06 01 EC')
    parsed = parse_rs485_finger_data_response(response)
    assert parsed.reserved == 1
    assert parsed.x1 == -12
    assert parsed.x2 == 25
    assert parsed.y1 == 126
    assert parsed.y2 == -93
    assert parsed.z1 == 85
    assert parsed.z2 == 262


def test_build_modbus_read_input_registers_matches_protocol_sheet() -> None:
    frame = build_modbus_read_input_registers(unit_id=1, start_addr=0x0013, count=6)
    assert frame == bytes.fromhex('01 04 00 13 00 06 81 CD')


def test_parse_modbus_factory_serial_response() -> None:
    response = bytes.fromhex('01 04 0C 00 0A 54 47 2D 39 38 30 31 30 30 30 D5 D1')
    parsed = parse_modbus_factory_serial_response(response)
    assert parsed == 'TG-9801000'


def test_parse_modbus_device_info_response() -> None:
    payload = bytes.fromhex('01 03 14 FF F4 00 7E 00 55 00 19 FF A3 01 02 00 02 00 64 03 E8 00 32')
    response = _with_crc(payload)
    parsed = parse_modbus_device_info_response(response)
    assert parsed.x1 == -12
    assert parsed.y1 == 126
    assert parsed.z1 == 85
    assert parsed.x2 == 25
    assert parsed.y2 == -93
    assert parsed.z2 == 258
    assert parsed.status == 2
    assert parsed.hardness == 100
    assert parsed.position == 1000
    assert parsed.current == 50
