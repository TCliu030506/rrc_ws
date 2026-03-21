"""TG-9801 交互式串口 CLI。

用于在终端交互选择串口、协议和动作，手动测试夹爪的控制、参数设置和状态读取功能。
"""

import json
import sys
from dataclasses import asdict, is_dataclass
from typing import List, Optional

from .client import TG9801Client, TG9801SerialConfig
from .protocol import MODBUS_REG_DEVICE_ID, bytes_to_hex, parse_number_string
from .serial_transport import SerialTransport


def _select_port() -> str:
    """枚举串口并通过交互选择目标端口。"""
    ports = SerialTransport.available_ports()
    if not ports:
        raise RuntimeError('未发现可用串口，请检查设备连接。')

    print('发现以下串口设备：')
    for idx, (dev, desc) in enumerate(ports):
        print(f'[{idx}] {dev} - {desc}')

    while True:
        sel = input('请选择要连接的编号(或 q 退出): ').strip().lower()
        if sel in ('q', 'quit', 'exit'):
            raise SystemExit(0)
        try:
            port_idx = int(sel)
        except ValueError:
            print('请输入有效数字编号。')
            continue
        if 0 <= port_idx < len(ports):
            return ports[port_idx][0]
        print(f'编号超出范围(0..{len(ports)-1})。')


def _render_result(result) -> str:
    """把结果格式化为便于终端查看的文本。"""
    if isinstance(result, bytes):
        return bytes_to_hex(result)
    if is_dataclass(result):
        return json.dumps(asdict(result), ensure_ascii=False)
    return str(result)


def _ask_int(prompt: str, default: Optional[int] = None) -> int:
    """读取整数输入，支持 0x 前缀。"""
    while True:
        raw = input(prompt).strip()
        if not raw and default is not None:
            return default
        try:
            return int(raw, 0)
        except ValueError:
            print('请输入合法整数，可用十进制或 0x 十六进制。')


def _ask_protocol() -> str:
    """选择协议。"""
    while True:
        print('\n协议选择: [1] RS485自定义 [2] ModbusRTU [q] 退出')
        proto = input('协议 > ').strip().lower()
        if proto in ('q', 'quit', 'exit'):
            raise SystemExit(0)
        if proto == '1':
            return 'rs485_custom'
        if proto == '2':
            return 'modbus_rtu'
        print('请输入 1/2/q。')


def _show_menu(protocol: str) -> None:
    """打印动作菜单。"""
    print('\n动作菜单:')
    print('[1] 夹取')
    print('[2] 松开')
    print('[3] 夹取->松开 cycle')
    print('[4] 读取设备ID')
    print('[5] 读取出厂编号')
    print('[6] 读取状态')
    print('[7] 读取目标夹取力')
    print('[8] 设置目标夹取力')
    print('[9] 设置协议模式')
    print('[10] 修改设备ID')
    print('[11] 强制校准')
    print('[12] 切换位置模式')
    print('[13] 发送位置指令')
    print('[14] 末端硬件清零')
    print('[15] 记录零点')
    if protocol == 'rs485_custom':
        print('[16] 读取夹指六维数据')
        print('[17] 设置波特率')
    else:
        print('[16] 读取随动状态')
        print('[17] 读保持寄存器')
        print('[18] 读输入寄存器')
        print('[19] 写单寄存器')
        print('[20] 写多寄存器')
    print('[raw] 手动字节透传')
    print('[b] 返回协议选择')
    print('[q] 退出')


def _handle_rs485_action(client: TG9801Client, action: str, device_addr: int, speed: int) -> None:
    """处理 RS485 自定义协议动作。"""
    if action == '1':
        print(f'[TX] {_render_result(client.grip(protocol="rs485_custom", device_addr=device_addr, speed=speed))}')
        return
    if action == '2':
        print(f'[TX] {_render_result(client.release(protocol="rs485_custom", device_addr=device_addr, speed=speed))}')
        return
    if action == '3':
        print(f'[TX] {_render_result(client.grip(protocol="rs485_custom", device_addr=device_addr, speed=speed))}')
        hold_sec = float(input('夹持时长秒(默认1.0): ').strip() or '1.0')
        if hold_sec > 0:
            import time
            time.sleep(hold_sec)
        print(f'[TX] {_render_result(client.release(protocol="rs485_custom", device_addr=device_addr, speed=speed))}')
        return
    if action == '4':
        print(f'[RESULT] {client.rs485_get_id(device_addr, read_response=True)}')
        return
    if action == '5':
        print(f'[RESULT] {client.rs485_get_factory_serial(device_addr)}')
        return
    if action == '6':
        print(f'[RESULT] {_render_result(client.rs485_get_device_info(device_addr))}')
        return
    if action == '7':
        print(f'[RESULT] {client.rs485_get_target_force(device_addr)}')
        return
    if action == '8':
        target_force = _ask_int('目标夹取力(1~100, 默认50): ', 50)
        print(f'[TX] {_render_result(client.rs485_set_target_force(device_addr, target_force))}')
        return
    if action == '9':
        mode = input('协议模式 [custom/modbus] (默认modbus): ').strip().lower() or 'modbus'
        print(f'[TX] {_render_result(client.rs485_set_protocol(device_addr, use_modbus=mode == "modbus"))}')
        return
    if action == '10':
        new_device_addr = _ask_int('新设备地址(默认2): ', 2)
        print(f'[TX] {_render_result(client.rs485_set_id(device_addr, new_device_addr))}')
        return
    if action == '11':
        print(f'[TX] {_render_result(client.rs485_force_calibration(device_addr))}')
        return
    if action == '12':
        mode = input('模式 [auto/position] (默认position): ').strip().lower() or 'position'
        print(f'[TX] {_render_result(client.rs485_set_position_mode(device_addr, position_mode=mode == "position"))}')
        return
    if action == '13':
        position = _ask_int('目标位置(0~1000, 默认1000): ', 1000)
        print(f'[TX] {_render_result(client.rs485_send_position(device_addr, speed=speed, position=position))}')
        return
    if action == '14':
        print(f'[TX] {_render_result(client.rs485_hardware_zero(device_addr))}')
        return
    if action == '15':
        print(f'[TX] {_render_result(client.rs485_record_zero(device_addr))}')
        return
    if action == '16':
        print(f'[RESULT] {_render_result(client.rs485_get_finger_data(device_addr))}')
        return
    if action == '17':
        baudrate = _ask_int('新波特率(默认1000000): ', 1000000)
        print(f'[RESULT] {client.rs485_set_baudrate(device_addr, baudrate, read_response=True)}')
        return
    raise ValueError('无效动作。')


def _handle_modbus_action(client: TG9801Client, action: str, device_addr: int, speed: int) -> None:
    """处理 Modbus RTU 动作。"""
    if action == '1':
        print(f'[TX] {_render_result(client.grip(protocol="modbus_rtu", device_addr=device_addr, speed=speed))}')
        return
    if action == '2':
        print(f'[TX] {_render_result(client.release(protocol="modbus_rtu", device_addr=device_addr, speed=speed))}')
        return
    if action == '3':
        print(f'[TX] {_render_result(client.grip(protocol="modbus_rtu", device_addr=device_addr, speed=speed))}')
        hold_sec = float(input('夹持时长秒(默认1.0): ').strip() or '1.0')
        if hold_sec > 0:
            import time
            time.sleep(hold_sec)
        print(f'[TX] {_render_result(client.release(protocol="modbus_rtu", device_addr=device_addr, speed=speed))}')
        return
    if action == '4':
        print(f'[RESULT] {client.modbus_read_holding_registers(device_addr, MODBUS_REG_DEVICE_ID, 1)[0]}')
        return
    if action == '5':
        print(f'[RESULT] {client.modbus_get_factory_serial(device_addr)}')
        return
    if action == '6':
        print(f'[RESULT] {_render_result(client.modbus_get_device_info(device_addr))}')
        return
    if action == '7':
        print(f'[RESULT] {client.modbus_get_target_force(device_addr)}')
        return
    if action == '8':
        target_force = _ask_int('目标夹取力(1~100, 默认50): ', 50)
        print(f'[RESULT] {_render_result(client.modbus_set_target_force(device_addr, target_force))}')
        return
    if action == '9':
        mode = input('协议模式 [custom/modbus] (默认modbus): ').strip().lower() or 'modbus'
        print(f'[RESULT] {_render_result(client.modbus_set_protocol(device_addr, use_modbus=mode == "modbus"))}')
        return
    if action == '10':
        new_device_addr = _ask_int('新设备地址(默认2): ', 2)
        print(f'[RESULT] {_render_result(client.modbus_set_device_id(device_addr, new_device_addr))}')
        return
    if action == '11':
        print(f'[RESULT] {_render_result(client.modbus_force_calibration(device_addr))}')
        return
    if action == '12':
        mode = input('模式 [auto/position] (默认position): ').strip().lower() or 'position'
        print(f'[RESULT] {_render_result(client.modbus_set_position_mode(device_addr, position_mode=mode == "position"))}')
        return
    if action == '13':
        position = _ask_int('目标位置(0~1000, 默认1000): ', 1000)
        print(f'[RESULT] {_render_result(client.modbus_send_position(device_addr, speed=speed, position=position))}')
        return
    if action == '14':
        value = _ask_int('清零值(默认1): ', 1)
        print(f'[RESULT] {_render_result(client.modbus_hardware_zero(device_addr, value))}')
        return
    if action == '15':
        value = _ask_int('记录零点值(默认1): ', 1)
        print(f'[RESULT] {_render_result(client.modbus_record_zero(device_addr, value))}')
        return
    if action == '16':
        print(f'[RESULT] {client.modbus_get_follow_status(device_addr)}')
        return
    if action == '17':
        start_addr = _ask_int('起始寄存器(默认0x0000): ', 0x0000)
        count = _ask_int('寄存器数量(默认1): ', 1)
        print(f'[RESULT] {_render_result(client.modbus_read_holding_registers(device_addr, start_addr, count))}')
        return
    if action == '18':
        start_addr = _ask_int('起始寄存器(默认0x0013): ', 0x0013)
        count = _ask_int('寄存器数量(默认1): ', 1)
        print(f'[RESULT] {_render_result(client.modbus_read_input_registers(device_addr, start_addr, count))}')
        return
    if action == '19':
        register_addr = _ask_int('寄存器地址: ')
        register_value = _ask_int('寄存器值: ')
        print(f'[RESULT] {_render_result(client.modbus_write_single_register(device_addr, register_addr, register_value))}')
        return
    if action == '20':
        register_addr = _ask_int('起始寄存器地址: ')
        values_text = input('多个寄存器值(空格分隔，例如 0x03E8 0x0001): ').strip()
        values = [int(item, 0) for item in values_text.split() if item]
        print(f'[RESULT] {_render_result(client.modbus_write_multiple_registers(device_addr, register_addr, values))}')
        return
    raise ValueError('无效动作。')


def main(argv: List[str] = None) -> int:
    """交互式命令行入口。"""
    _ = argv or []

    try:
        port = _select_port()
    except SystemExit:
        print('已退出。')
        return 0
    except Exception as exc:
        print(str(exc))
        return 1

    baudrate = _ask_int('波特率(默认1000000): ', 1000000)
    timeout_raw = input('超时秒(默认1.0): ').strip()
    timeout = float(timeout_raw) if timeout_raw else 1.0

    client = TG9801Client(
        TG9801SerialConfig(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
        )
    )
    client.open()
    print(f'已连接 {port} @ {baudrate}。')

    try:
        while True:
            try:
                protocol = _ask_protocol()
            except SystemExit:
                break

            device_addr = _ask_int('设备地址(默认1): ', 1)
            speed = _ask_int('默认动作速度(默认1000): ', 1000)

            while True:
                _show_menu(protocol)
                action = input('动作 > ').strip().lower()
                if action == 'b':
                    break
                if action in ('q', 'quit', 'exit'):
                    return 0
                if action == 'raw':
                    line = input('输入字节行 > ').strip()
                    data = parse_number_string(line)
                    sent = client.send_raw(data)
                    print(f'[TX] 已发送 {sent} 字节 | {bytes_to_hex(data)}')
                    rx = client.read_available()
                    if rx:
                        print(f'[RX] 收到 {len(rx)} 字节 | {bytes_to_hex(rx)}')
                    continue
                try:
                    if protocol == 'rs485_custom':
                        _handle_rs485_action(client, action, device_addr, speed)
                    else:
                        _handle_modbus_action(client, action, device_addr, speed)
                except Exception as exc:
                    print(f'动作失败: {exc}')
    finally:
        client.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
