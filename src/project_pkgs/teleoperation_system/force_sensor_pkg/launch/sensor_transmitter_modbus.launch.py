from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='串口波特率',
    )
    slave_arg = DeclareLaunchArgument(
        'slave',
        default_value='1',
        description='Modbus 从站地址',
    )
    interval_arg = DeclareLaunchArgument(
        'interval',
        default_value='0.5',
        description='读取与发布周期（秒）',
    )
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='force_sensor_1',
        description='PressureData 发布话题名',
    )

    sensor_node = Node(
        package='force_sensor_pkg',
        executable='sensor_transmitter_modbus_node',
        name='sensor_transmitter_modbus_node',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'slave': ParameterValue(LaunchConfiguration('slave'), value_type=int),
            'interval': ParameterValue(LaunchConfiguration('interval'), value_type=float),
            'topic_name': ParameterValue(LaunchConfiguration('topic_name'), value_type=str),
        }],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        slave_arg,
        interval_arg,
        topic_name_arg,
        sensor_node,
    ])
