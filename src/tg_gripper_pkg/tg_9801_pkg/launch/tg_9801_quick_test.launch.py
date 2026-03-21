"""TG-9801 快速测试启动文件。

用于通过 launch 参数拼装 tg_9801_quick_test 命令，方便用 ROS2 launch 方式执行串口测试。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _build_quick_test_cmd(context):
    cmd = [
        'ros2', 'run', 'tg_9801_pkg', 'tg_9801_quick_test',
        '--port', LaunchConfiguration('port').perform(context),
        '--baudrate', LaunchConfiguration('baudrate').perform(context),
        '--timeout', LaunchConfiguration('timeout').perform(context),
        '--parity', LaunchConfiguration('parity').perform(context),
        '--stopbits', LaunchConfiguration('stopbits').perform(context),
        '--bytesize', LaunchConfiguration('bytesize').perform(context),
        '--protocol', LaunchConfiguration('protocol').perform(context),
        '--device-addr', LaunchConfiguration('device_addr').perform(context),
        '--modbus-start-addr', LaunchConfiguration('modbus_start_addr').perform(context),
        '--rs485-release-cmd', LaunchConfiguration('rs485_release_cmd').perform(context),
        '--set-protocol', LaunchConfiguration('set_protocol').perform(context),
        '--action', LaunchConfiguration('action').perform(context),
        '--hold-sec', LaunchConfiguration('hold_sec').perform(context),
        '--rx-wait', LaunchConfiguration('rx_wait').perform(context),
        '--scan-max-id', LaunchConfiguration('scan_max_id').perform(context),
    ]

    if _to_bool(LaunchConfiguration('use_rs485_mode').perform(context)):
        cmd.append('--use-rs485-mode')
        cmd.extend(['--rts-level-for-tx', LaunchConfiguration('rts_level_for_tx').perform(context)])
        cmd.extend(['--rts-level-for-rx', LaunchConfiguration('rts_level_for_rx').perform(context)])
        cmd.extend(['--delay-before-tx', LaunchConfiguration('delay_before_tx').perform(context)])
        cmd.extend(['--delay-before-rx', LaunchConfiguration('delay_before_rx').perform(context)])

    if _to_bool(LaunchConfiguration('sweep_serial').perform(context)):
        cmd.append('--sweep-serial')

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('timeout', default_value='1.0'),
        DeclareLaunchArgument('parity', default_value='N'),
        DeclareLaunchArgument('stopbits', default_value='1.0'),
        DeclareLaunchArgument('bytesize', default_value='8'),
        DeclareLaunchArgument('protocol', default_value='modbus_rtu'),
        DeclareLaunchArgument('device_addr', default_value='1'),
        DeclareLaunchArgument('modbus_start_addr', default_value='0x000A'),
        DeclareLaunchArgument('rs485_release_cmd', default_value='0x01'),
        DeclareLaunchArgument('set_protocol', default_value='none'),
        DeclareLaunchArgument('action', default_value='cycle'),
        DeclareLaunchArgument('hold_sec', default_value='1.0'),
        DeclareLaunchArgument('rx_wait', default_value='0.2'),
        DeclareLaunchArgument('scan_max_id', default_value='20'),
        DeclareLaunchArgument('use_rs485_mode', default_value='false'),
        DeclareLaunchArgument('rts_level_for_tx', default_value='1'),
        DeclareLaunchArgument('rts_level_for_rx', default_value='0'),
        DeclareLaunchArgument('delay_before_tx', default_value='0.0'),
        DeclareLaunchArgument('delay_before_rx', default_value='0.0'),
        DeclareLaunchArgument('sweep_serial', default_value='false'),
        OpaqueFunction(function=_build_quick_test_cmd),
    ])
