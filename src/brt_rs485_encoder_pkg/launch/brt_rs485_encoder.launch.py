from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_bool(text: str) -> bool:
    lowered = text.lower()
    if lowered in {'1', 'true', 'yes', 'on'}:
        return True
    if lowered in {'0', 'false', 'no', 'off'}:
        return False
    raise ValueError(f'Cannot parse boolean launch argument: {text}')


def _optional_param(context, name: str, converter):
    text = LaunchConfiguration(name).perform(context).strip()
    if text == '':
        return None
    return converter(text)


def _launch_encoder_node(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    overrides = {}
    param_specs = {
        'port': str,
        'baudrate': int,
        'address': int,
        'timeout': float,
        'publish_rate_hz': float,
        'resolution': int,
        'sample_time_ms': int,
        'position_sign': float,
        'angle_offset_rad': float,
        'zero_offset_count': int,
        'reconnect_interval_sec': float,
        'dry_run': _parse_bool,
        'verbose': _parse_bool,
        'publish_joint_state': _parse_bool,
        'joint_name': str,
        'frame_id': str,
        'joint_state_topic': str,
        'joint_state_position_source': str,
        'joint_state_velocity_source': str,
        'publish_raw_counts': _parse_bool,
        'publish_position': _parse_bool,
        'position_topic': str,
        'position_raw_count_topic': str,
        'publish_position2': _parse_bool,
        'position2_topic': str,
        'position2_raw_count_topic': str,
        'publish_multiturn': _parse_bool,
        'multiturn_topic': str,
        'multiturn_raw_count_topic': str,
        'publish_turns': _parse_bool,
        'turns_topic': str,
        'publish_speed16': _parse_bool,
        'speed16_topic': str,
        'speed16_raw_count_topic': str,
        'publish_speed32': _parse_bool,
        'speed32_topic': str,
        'speed32_raw_count_topic': str,
    }

    for name, converter in param_specs.items():
        value = _optional_param(context, name, converter)
        if value is not None:
            overrides[name] = value

    return [
        Node(
            package='brt_rs485_encoder_pkg',
            executable='brt_rs485_encoder_node',
            name='brt_rs485_encoder_node',
            output='screen',
            parameters=[config_file, overrides],
        )
    ]


def _declare_override(name: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        name,
        default_value='',
        description=f'{description} Leave empty to use config_file.',
    )


def generate_launch_description() -> LaunchDescription:
    default_config_file = PathJoinSubstitution([
        FindPackageShare('brt_rs485_encoder_pkg'),
        'config',
        'brt_rs485_encoder.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='YAML parameter file for the encoder node.',
        ),
        _declare_override(
            'port',
            'Serial port connected to the RS485 adapter.',
        ),
        _declare_override('baudrate', 'Encoder baudrate.'),
        _declare_override('address', 'Modbus slave address of the encoder.'),
        _declare_override(
            'timeout',
            'Serial transaction timeout in seconds.',
        ),
        _declare_override(
            'publish_rate_hz',
            'Encoder polling and publication rate.',
        ),
        _declare_override(
            'resolution',
            'Counts per revolution for angle conversion.',
        ),
        _declare_override(
            'sample_time_ms',
            'Encoder speed sampling interval in milliseconds.',
        ),
        _declare_override(
            'position_sign',
            'Sign multiplier applied to position and velocity.',
        ),
        _declare_override(
            'angle_offset_rad',
            'Offset added to published angle in radians.',
        ),
        _declare_override(
            'zero_offset_count',
            'Raw count offset subtracted before angle conversion.',
        ),
        _declare_override(
            'reconnect_interval_sec',
            'Delay before reconnecting after an error.',
        ),
        _declare_override('dry_run', 'Print requests without opening serial.'),
        _declare_override('verbose', 'Print TX/RX frames.'),
        _declare_override('publish_joint_state', 'Whether to publish JointState.'),
        _declare_override('joint_name', 'JointState name field.'),
        _declare_override('frame_id', 'JointState header frame_id.'),
        _declare_override('joint_state_topic', 'JointState output topic.'),
        _declare_override(
            'joint_state_position_source',
            'Position source for JointState: position, position2, multiturn, none.',
        ),
        _declare_override(
            'joint_state_velocity_source',
            'Velocity source for JointState: speed16, speed32, none.',
        ),
        _declare_override(
            'publish_raw_counts',
            'Whether to publish raw counts for enabled scalar readouts.',
        ),
        _declare_override('publish_position', 'Whether to publish position.'),
        _declare_override('position_topic', 'Float64 position topic in radians.'),
        _declare_override(
            'position_raw_count_topic',
            'Int64 raw count topic for position.',
        ),
        _declare_override('publish_position2', 'Whether to publish position2.'),
        _declare_override('position2_topic', 'Float64 position2 topic in radians.'),
        _declare_override(
            'position2_raw_count_topic',
            'Int64 raw count topic for position2.',
        ),
        _declare_override('publish_multiturn', 'Whether to publish multiturn.'),
        _declare_override(
            'multiturn_topic',
            'Float64 virtual multiturn topic in radians.',
        ),
        _declare_override(
            'multiturn_raw_count_topic',
            'Int64 raw count topic for virtual multiturn.',
        ),
        _declare_override('publish_turns', 'Whether to publish virtual turns.'),
        _declare_override('turns_topic', 'Int64 virtual turns topic.'),
        _declare_override('publish_speed16', 'Whether to publish 16-bit speed.'),
        _declare_override('speed16_topic', 'Float64 speed16 topic in rad/s.'),
        _declare_override(
            'speed16_raw_count_topic',
            'Int64 raw delta-count topic for speed16.',
        ),
        _declare_override('publish_speed32', 'Whether to publish 32-bit speed.'),
        _declare_override('speed32_topic', 'Float64 speed32 topic in rad/s.'),
        _declare_override(
            'speed32_raw_count_topic',
            'Int64 raw delta-count topic for speed32.',
        ),
        OpaqueFunction(function=_launch_encoder_node),
    ])
