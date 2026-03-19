"""TG-9801 ROS2 节点启动文件。

用于声明串口和协议相关参数，并启动 tg_9801_node 作为夹爪的 ROS2 接口节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    timeout = LaunchConfiguration('timeout')
    protocol = LaunchConfiguration('protocol')
    device_address = LaunchConfiguration('device_address')
    modbus_start_addr = LaunchConfiguration('modbus_start_addr')
    serial_parity = LaunchConfiguration('serial_parity')
    serial_stopbits = LaunchConfiguration('serial_stopbits')
    serial_bytesize = LaunchConfiguration('serial_bytesize')
    use_rs485_mode = LaunchConfiguration('use_rs485_mode')
    rs485_rts_level_for_tx = LaunchConfiguration('rs485_rts_level_for_tx')
    rs485_rts_level_for_rx = LaunchConfiguration('rs485_rts_level_for_rx')
    rs485_delay_before_tx = LaunchConfiguration('rs485_delay_before_tx')
    rs485_delay_before_rx = LaunchConfiguration('rs485_delay_before_rx')
    rs485_release_cmd = LaunchConfiguration('rs485_release_cmd')
    enable_state_broadcaster = LaunchConfiguration('enable_state_broadcaster')
    state_prefix = LaunchConfiguration('state_prefix')
    state_poll_period = LaunchConfiguration('state_poll_period')
    state_request_timeout = LaunchConfiguration('state_request_timeout')
    state_read_speed_register = LaunchConfiguration('state_read_speed_register')
    state_read_follow_status = LaunchConfiguration('state_read_follow_status')
    state_read_finger_data = LaunchConfiguration('state_read_finger_data')
    state_finger_publish_hz = LaunchConfiguration('state_finger_publish_hz')
    state_motion_publish_hz = LaunchConfiguration('state_motion_publish_hz')
    state_static_publish_hz = LaunchConfiguration('state_static_publish_hz')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('timeout', default_value='1.0'),
        DeclareLaunchArgument('protocol', default_value='modbus_rtu'),
        DeclareLaunchArgument('device_address', default_value='1'),
        DeclareLaunchArgument('modbus_start_addr', default_value='10'),
        DeclareLaunchArgument('serial_parity', default_value='N'),
        DeclareLaunchArgument('serial_stopbits', default_value='1.0'),
        DeclareLaunchArgument('serial_bytesize', default_value='8'),
        DeclareLaunchArgument('use_rs485_mode', default_value='false'),
        DeclareLaunchArgument('rs485_rts_level_for_tx', default_value='true'),
        DeclareLaunchArgument('rs485_rts_level_for_rx', default_value='false'),
        DeclareLaunchArgument('rs485_delay_before_tx', default_value='0.0'),
        DeclareLaunchArgument('rs485_delay_before_rx', default_value='0.0'),
        DeclareLaunchArgument('rs485_release_cmd', default_value='1'),
        DeclareLaunchArgument('enable_state_broadcaster', default_value='true'),
        DeclareLaunchArgument('state_prefix', default_value='tg_state'),
        DeclareLaunchArgument('state_poll_period', default_value='1.0'),
        DeclareLaunchArgument('state_request_timeout', default_value='0.8'),
        DeclareLaunchArgument('state_read_speed_register', default_value='true'),
        DeclareLaunchArgument('state_read_follow_status', default_value='true'),
        DeclareLaunchArgument('state_read_finger_data', default_value='false'),
        DeclareLaunchArgument('state_finger_publish_hz', default_value='50.0'),
        DeclareLaunchArgument('state_motion_publish_hz', default_value='10.0'),
        DeclareLaunchArgument('state_static_publish_hz', default_value='1.0'),
        Node(
            package='tg_9801_pkg',
            executable='tg_9801_node',
            name='tg_9801_node',
            output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=str),
                'baudrate': ParameterValue(baudrate, value_type=int),
                'timeout': ParameterValue(timeout, value_type=float),
                'protocol': ParameterValue(protocol, value_type=str),
                'device_address': ParameterValue(device_address, value_type=int),
                'modbus_start_addr': ParameterValue(modbus_start_addr, value_type=int),
                'serial_parity': ParameterValue(serial_parity, value_type=str),
                'serial_stopbits': ParameterValue(serial_stopbits, value_type=float),
                'serial_bytesize': ParameterValue(serial_bytesize, value_type=int),
                'use_rs485_mode': ParameterValue(use_rs485_mode, value_type=bool),
                'rs485_rts_level_for_tx': ParameterValue(rs485_rts_level_for_tx, value_type=bool),
                'rs485_rts_level_for_rx': ParameterValue(rs485_rts_level_for_rx, value_type=bool),
                'rs485_delay_before_tx': ParameterValue(rs485_delay_before_tx, value_type=float),
                'rs485_delay_before_rx': ParameterValue(rs485_delay_before_rx, value_type=float),
                'rs485_release_cmd': ParameterValue(rs485_release_cmd, value_type=int),
            }],
        ),
        Node(
            package='tg_9801_pkg',
            executable='tg_state_broadcaster',
            name='tg_state_broadcaster',
            output='screen',
            condition=IfCondition(enable_state_broadcaster),
            parameters=[{
                'protocol': ParameterValue(protocol, value_type=str),
                'device_addr': ParameterValue(device_address, value_type=int),
                'state_prefix': ParameterValue(state_prefix, value_type=str),
                'poll_period': ParameterValue(state_poll_period, value_type=float),
                'request_timeout': ParameterValue(state_request_timeout, value_type=float),
                'read_speed_register': ParameterValue(state_read_speed_register, value_type=bool),
                'read_follow_status': ParameterValue(state_read_follow_status, value_type=bool),
                'read_finger_data': ParameterValue(state_read_finger_data, value_type=bool),
                'finger_publish_hz': ParameterValue(state_finger_publish_hz, value_type=float),
                'motion_publish_hz': ParameterValue(state_motion_publish_hz, value_type=float),
                'static_publish_hz': ParameterValue(state_static_publish_hz, value_type=float),
            }],
        ),
    ])
