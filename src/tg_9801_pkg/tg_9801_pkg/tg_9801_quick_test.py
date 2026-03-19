"""TG-9801 全功能快速测试工具。

用于通过命令行直接调用夹爪的 RS485/Modbus 功能，便于联调、扫参和逐项验证协议行为。
"""

import argparse
from dataclasses import asdict, is_dataclass
import json
import time

from .client import TG9801Client, TG9801SerialConfig
from .protocol import MODBUS_REG_DEVICE_ID, bytes_to_hex


def _normalize_parity(value: str) -> str:
    """将用户输入的 parity 归一化为 N/E/O。"""
    parity = value.strip().upper()
    alias = {
        'NONE': 'N',
        'NO': 'N',
        'EVEN': 'E',
        'ODD': 'O',
    }
    parity = alias.get(parity, parity)
    if parity not in ('N', 'E', 'O'):
        raise ValueError('parity 仅支持 N/E/O')
    return parity


def _print_result(tag: str, result) -> None:
    """统一打印结果。"""
    if isinstance(result, bytes):
        print(f'[{tag}] {bytes_to_hex(result)}')
        return
    if is_dataclass(result):
        print(f'[{tag}] {json.dumps(asdict(result), ensure_ascii=False)}')
        return
    if isinstance(result, tuple):
        print(f'[{tag}] {result}')
        return
    if isinstance(result, list):
        print(f'[{tag}] {json.dumps(result, ensure_ascii=False)}')
        return
    print(f'[{tag}] {result}')


def _try_receive_after_write(client: TG9801Client, rx_wait: float, tag: str) -> None:
    """对于可能有回包的写操作，做一次可选接收打印。"""
    if rx_wait > 0:
        time.sleep(rx_wait)
    rx = client.read_available()
    if rx:
        print(f'[RX][{tag}] {len(rx)} bytes | {bytes_to_hex(rx)}')


def _run_cycle(
    client: TG9801Client,
    protocol: str,
    device_addr: int,
    modbus_start_addr: int,
    release_cmd: int,
    hold_sec: float,
    rx_wait: float,
    speed: int,
) -> None:
    """执行夹取 -> 等待 -> 松开。"""
    grip_frame = client.grip(
        protocol=protocol,
        device_addr=device_addr,
        modbus_start_addr=modbus_start_addr,
        rs485_release_cmd=release_cmd,
        speed=speed,
    )
    print(f'[TX][grip] {len(grip_frame)} bytes | {bytes_to_hex(grip_frame)}')
    _try_receive_after_write(client, rx_wait, 'grip')

    if hold_sec > 0:
        time.sleep(hold_sec)

    release_frame = client.release(
        protocol=protocol,
        device_addr=device_addr,
        modbus_start_addr=modbus_start_addr,
        rs485_release_cmd=release_cmd,
        speed=speed,
    )
    print(f'[TX][release] {len(release_frame)} bytes | {bytes_to_hex(release_frame)}')
    _try_receive_after_write(client, rx_wait, 'release')


def _scan_id(client: TG9801Client, protocol: str, max_id: int) -> None:
    """扫描地址。"""
    print(f'[SCAN] 0..{max_id}')
    for addr in range(0, max_id + 1):
        try:
            if protocol == 'rs485_custom':
                result = client.rs485_get_id(addr, read_response=True)
            else:
                result = client.modbus_read_holding_registers(addr, MODBUS_REG_DEVICE_ID, 1)[0]
            print(f'[SCAN-HIT] addr={addr} result={result}')
        except Exception as exc:
            print(f'[SCAN-MISS] addr={addr} reason={exc}')


def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数定义。"""
    parser = argparse.ArgumentParser(description='TG-9801 串口全功能快速测试工具')
    parser.add_argument('--port', required=True, help='串口，如 /dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=1000000, help='波特率，默认1000000')
    parser.add_argument('--timeout', type=float, default=1.0, help='串口超时秒，默认1.0')
    parser.add_argument('--parity', default='N', help='校验位 N/E/O，默认N')
    parser.add_argument('--stopbits', type=float, default=1.0, help='停止位 1 或 2，默认1')
    parser.add_argument('--bytesize', type=int, default=8, help='数据位 7 或 8，默认8')
    parser.add_argument('--use-rs485-mode', action='store_true', help='启用 pyserial RS485 自动方向控制')
    parser.add_argument('--rts-level-for-tx', type=int, choices=[0, 1], default=1)
    parser.add_argument('--rts-level-for-rx', type=int, choices=[0, 1], default=0)
    parser.add_argument('--delay-before-tx', type=float, default=0.0)
    parser.add_argument('--delay-before-rx', type=float, default=0.0)

    parser.add_argument('--protocol', choices=['rs485_custom', 'modbus_rtu'], default='modbus_rtu')
    parser.add_argument('--device-addr', type=int, default=1, help='设备地址，默认1')
    parser.add_argument('--new-device-addr', type=lambda x: int(x, 0), default=2, help='修改后的设备地址')
    parser.add_argument('--modbus-start-addr', type=lambda x: int(x, 0), default=0x000A)
    parser.add_argument('--rs485-release-cmd', type=lambda x: int(x, 0), default=0x01)
    parser.add_argument('--speed', type=int, default=1000, help='动作速度，默认1000')
    parser.add_argument('--position', type=int, default=1000, help='目标位置，默认1000')
    parser.add_argument('--target-force', type=int, default=50, help='目标夹取力，默认50')
    parser.add_argument('--baudrate-value', type=int, default=1000000, help='设置设备波特率时使用的值')
    parser.add_argument('--reserved', type=lambda x: int(x, 0), default=0x00, help='保留字节')
    parser.add_argument('--position-mode', choices=['auto', 'position'], default='position')
    parser.add_argument('--protocol-mode', choices=['custom', 'modbus'], default='modbus')
    parser.add_argument('--register-addr', type=lambda x: int(x, 0), default=0x000A)
    parser.add_argument('--register-value', type=lambda x: int(x, 0), default=0x0001)
    parser.add_argument('--register-values', nargs='*', type=lambda x: int(x, 0), default=None)
    parser.add_argument('--read-start-addr', type=lambda x: int(x, 0), default=0x0000)
    parser.add_argument('--read-count', type=int, default=1)

    parser.add_argument(
        '--set-protocol',
        choices=['none', 'custom', 'modbus'],
        default='none',
        help='执行主动作前先发送一次切换协议命令',
    )
    parser.add_argument(
        '--action',
        choices=[
            'grip',
            'release',
            'cycle',
            'get_id',
            'scan_id',
            'calibrate',
            'get_serial',
            'get_status',
            'get_finger',
            'get_force',
            'set_force',
            'set_id',
            'set_position_mode',
            'position',
            'get_follow_status',
            'hardware_zero',
            'record_zero',
            'set_baudrate',
            'read_holding',
            'read_input',
            'write_single',
            'write_multi',
            'set_speed',
            'set_command',
            'set_protocol',
        ],
        default='cycle',
        help='测试动作',
    )
    parser.add_argument('--hold-sec', type=float, default=1.0, help='cycle时夹持持续时间，默认1.0秒')
    parser.add_argument('--rx-wait', type=float, default=0.15, help='每次发送后等待回包时间，默认0.15秒')
    parser.add_argument('--scan-max-id', type=int, default=10, help='scan_id 的最大地址，默认10')
    parser.add_argument(
        '--sweep-serial',
        action='store_true',
        help='按常见串口参数组合扫测（1000000/N/8/1、115200/N/8/1、115200/E/8/1）',
    )
    return parser


def _run_single(args: argparse.Namespace, baudrate: int, parity: str, stopbits: float, bytesize: int) -> None:
    """在一组串口参数下执行一次测试流程。"""
    client = TG9801Client(TG9801SerialConfig(
        port=args.port,
        baudrate=baudrate,
        timeout=args.timeout,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        use_rs485_mode=args.use_rs485_mode,
        rs485_rts_level_for_tx=bool(args.rts_level_for_tx),
        rs485_rts_level_for_rx=bool(args.rts_level_for_rx),
        rs485_delay_before_tx=args.delay_before_tx,
        rs485_delay_before_rx=args.delay_before_rx,
    ))

    client.open()
    print(
        f'connected: port={args.port} baud={baudrate} protocol={args.protocol} '
        f'parity={parity} stopbits={stopbits} bytesize={bytesize} '
        f'use_rs485_mode={args.use_rs485_mode}'
    )

    try:
        if args.set_protocol != 'none':
            set_to_modbus = args.set_protocol == 'modbus'
            frame = client.rs485_set_protocol(args.device_addr, use_modbus=set_to_modbus)
            print(f'[TX][set_protocol_{args.set_protocol}] {len(frame)} bytes | {bytes_to_hex(frame)}')
            _try_receive_after_write(client, args.rx_wait, f'set_protocol_{args.set_protocol}')
            time.sleep(0.2)

        if args.action == 'cycle':
            _run_cycle(
                client=client,
                protocol=args.protocol,
                device_addr=args.device_addr,
                modbus_start_addr=args.modbus_start_addr,
                release_cmd=args.rs485_release_cmd,
                hold_sec=args.hold_sec,
                rx_wait=args.rx_wait,
                speed=args.speed,
            )
            return

        if args.action == 'grip':
            frame = client.grip(
                protocol=args.protocol,
                device_addr=args.device_addr,
                modbus_start_addr=args.modbus_start_addr,
                rs485_release_cmd=args.rs485_release_cmd,
                speed=args.speed,
            )
            print(f'[TX][grip] {len(frame)} bytes | {bytes_to_hex(frame)}')
            _try_receive_after_write(client, args.rx_wait, 'grip')
            return

        if args.action == 'release':
            frame = client.release(
                protocol=args.protocol,
                device_addr=args.device_addr,
                modbus_start_addr=args.modbus_start_addr,
                rs485_release_cmd=args.rs485_release_cmd,
                speed=args.speed,
            )
            print(f'[TX][release] {len(frame)} bytes | {bytes_to_hex(frame)}')
            _try_receive_after_write(client, args.rx_wait, 'release')
            return

        if args.action == 'get_id':
            if args.protocol == 'rs485_custom':
                _print_result('get_id', client.rs485_get_id(args.device_addr, read_response=True))
            else:
                _print_result('get_id', client.modbus_read_holding_registers(args.device_addr, MODBUS_REG_DEVICE_ID, 1)[0])
            return

        if args.action == 'scan_id':
            _scan_id(client, args.protocol, max(0, min(255, int(args.scan_max_id))))
            return

        if args.action == 'calibrate':
            result = client.rs485_force_calibration(args.device_addr) if args.protocol == 'rs485_custom' else client.modbus_force_calibration(args.device_addr)
            _print_result('calibrate', result)
            _try_receive_after_write(client, args.rx_wait, 'calibrate')
            return

        if args.action == 'get_serial':
            _print_result('get_serial', client.get_factory_serial(args.protocol, args.device_addr))
            return

        if args.action == 'get_status':
            _print_result('get_status', client.get_device_state(args.protocol, args.device_addr))
            return

        if args.action == 'get_finger':
            if args.protocol != 'rs485_custom':
                raise ValueError('get_finger 仅支持 rs485_custom 协议')
            _print_result('get_finger', client.rs485_get_finger_data(args.device_addr, reserved=args.reserved))
            return

        if args.action == 'get_force':
            _print_result('get_force', client.get_target_force(args.protocol, args.device_addr))
            return

        if args.action == 'set_force':
            _print_result('set_force', client.set_target_force(args.protocol, args.device_addr, args.target_force))
            _try_receive_after_write(client, args.rx_wait, 'set_force')
            return

        if args.action == 'set_id':
            result = client.rs485_set_id(args.device_addr, args.new_device_addr) if args.protocol == 'rs485_custom' else client.modbus_set_device_id(args.device_addr, args.new_device_addr)
            _print_result('set_id', result)
            _try_receive_after_write(client, args.rx_wait, 'set_id')
            return

        if args.action == 'set_position_mode':
            position_mode = args.position_mode == 'position'
            _print_result('set_position_mode', client.set_position_mode(args.protocol, args.device_addr, position_mode))
            _try_receive_after_write(client, args.rx_wait, 'set_position_mode')
            return

        if args.action == 'position':
            _print_result('position', client.send_position(args.protocol, args.device_addr, speed=args.speed, position=args.position))
            _try_receive_after_write(client, args.rx_wait, 'position')
            return

        if args.action == 'get_follow_status':
            if args.protocol != 'modbus_rtu':
                raise ValueError('get_follow_status 仅支持 modbus_rtu 协议')
            _print_result('get_follow_status', client.modbus_get_follow_status(args.device_addr))
            return

        if args.action == 'hardware_zero':
            result = client.rs485_hardware_zero(args.device_addr) if args.protocol == 'rs485_custom' else client.modbus_hardware_zero(args.device_addr, args.register_value)
            _print_result('hardware_zero', result)
            _try_receive_after_write(client, args.rx_wait, 'hardware_zero')
            return

        if args.action == 'record_zero':
            result = client.rs485_record_zero(args.device_addr) if args.protocol == 'rs485_custom' else client.modbus_record_zero(args.device_addr, args.register_value)
            _print_result('record_zero', result)
            _try_receive_after_write(client, args.rx_wait, 'record_zero')
            return

        if args.action == 'set_baudrate':
            if args.protocol != 'rs485_custom':
                raise ValueError('set_baudrate 仅支持 rs485_custom 协议')
            _print_result('set_baudrate', client.rs485_set_baudrate(args.device_addr, args.baudrate_value, read_response=True))
            return

        if args.action == 'read_holding':
            _print_result('read_holding', client.modbus_read_holding_registers(args.device_addr, args.read_start_addr, args.read_count))
            return

        if args.action == 'read_input':
            _print_result('read_input', client.modbus_read_input_registers(args.device_addr, args.read_start_addr, args.read_count))
            return

        if args.action == 'write_single':
            _print_result('write_single', client.modbus_write_single_register(args.device_addr, args.register_addr, args.register_value))
            return

        if args.action == 'write_multi':
            values = args.register_values if args.register_values else [args.register_value]
            _print_result('write_multi', client.modbus_write_multiple_registers(args.device_addr, args.register_addr, values))
            return

        if args.action == 'set_speed':
            _print_result('set_speed', client.modbus_set_speed(args.device_addr, args.speed))
            return

        if args.action == 'set_command':
            _print_result('set_command', client.modbus_set_command(args.device_addr, args.register_value))
            return

        if args.action == 'set_protocol':
            if args.protocol == 'rs485_custom':
                _print_result('set_protocol', client.rs485_set_protocol(args.device_addr, use_modbus=args.protocol_mode == 'modbus'))
                _try_receive_after_write(client, args.rx_wait, 'set_protocol')
            else:
                _print_result('set_protocol', client.modbus_set_protocol(args.device_addr, use_modbus=args.protocol_mode == 'modbus'))
            return
    finally:
        client.close()


def main() -> int:
    """快速测试入口。"""
    args = build_arg_parser().parse_args()
    if not args.sweep_serial:
        parity = _normalize_parity(args.parity)
        _run_single(args, args.baudrate, parity, args.stopbits, args.bytesize)
        return 0

    combos = [
        (1000000, 'N', 1.0, 8),
        (115200, 'N', 1.0, 8),
        (115200, 'E', 1.0, 8),
    ]
    for baudrate, parity, stopbits, bytesize in combos:
        print(f'\n[SWEEP] try baud={baudrate} parity={parity} stopbits={stopbits} bytesize={bytesize}')
        try:
            _run_single(args, baudrate, parity, stopbits, bytesize)
        except Exception as exc:
            print(f'[SWEEP] failed: {exc}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
