from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 硬件和 TF 相关配置集中在这里修改，避免通过 ros2 launch 参数层层转发。
UR_TYPE = 'ur5'
ROBOT_IP = '192.168.1.102'
FORCE_SENSOR_PORT = '/dev/ttyUSB0'

BASE_FRAME = 'base'
TOOL_FRAME = 'tool0'
ASM_EE_FRAME = 'asm_ee_site'
RAW_WRENCH_TOPIC = '/external_force_torque_wrench'
EE_POSE_TOPIC = '/asm_ee_site/pose'
EE_TWIST_TOPIC = '/asm_ee_site/twist'


def generate_launch_description() -> LaunchDescription:
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': UR_TYPE,
            'robot_ip': ROBOT_IP,
            'launch_rviz': 'false',
        }.items(),
    )

    encoder_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('brt_rs485_encoder_pkg'),
                'launch',
                'brt_rs485_encoder_dual.launch.py',
            ])
        ),
    )

    force_sensor_node = Node(
        package='force_sensor',
        executable='force_sensor_axis_6',
        name='force_sensor_axis_6',
        output='screen',
        parameters=[{
            'forcesensorport': FORCE_SENSOR_PORT,
            'forcesensor_rate': 200,
            'baudrate': 115200,
            'frame_id': 'asm_force_sensor_link',
            'topic_name': RAW_WRENCH_TOPIC,
            'auto_zero': False,
        }],
    )

    asm_tool_tf_broadcaster = Node(
        package='ultra_scanning_system',
        executable='asm_tool_tf_broadcaster',
        name='asm_tool_tf_broadcaster',
        output='screen',
        parameters=[{
            'parent_frame': TOOL_FRAME,
            'publish_rate': 125.0,
        }],
    )

    ee_state_from_tf_node = Node(
        package='ultra_scanning_sim',
        executable='ee_state_from_tf_node',
        name='asm_ee_state_from_tf_node',
        output='screen',
        parameters=[{
            'source_frame': BASE_FRAME,
            'target_frame': ASM_EE_FRAME,
            'output_pose_topic': EE_POSE_TOPIC,
            'output_twist_topic': EE_TWIST_TOPIC,
            'publish_rate': 125.0,
            'max_angular_speed': 10.0,
        }],
    )

    return LaunchDescription([
        force_sensor_node,
        ur_driver_launch,
        encoder_dual_launch,
        asm_tool_tf_broadcaster,
        ee_state_from_tf_node,
    ])
