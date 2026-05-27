from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    asm_parent_frame = LaunchConfiguration('asm_parent_frame')
    asm_tf_publish_rate = LaunchConfiguration('asm_tf_publish_rate')

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': launch_rviz,
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

    asm_tool_tf_broadcaster = Node(
        package='ultra_scanning_system',
        executable='asm_tool_tf_broadcaster',
        name='asm_tool_tf_broadcaster',
        output='screen',
        parameters=[{
            'parent_frame': asm_parent_frame,
            'publish_rate': asm_tf_publish_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument(
            'asm_parent_frame',
            default_value='tool0',
        ),
        DeclareLaunchArgument('asm_tf_publish_rate', default_value='50.0'),
        ur_driver_launch,
        encoder_dual_launch,
        asm_tool_tf_broadcaster,
    ])
