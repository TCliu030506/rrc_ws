from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only')
    target_cmd_topic = LaunchConfiguration('target_cmd_topic')
    tcp_state_topic = LaunchConfiguration('tcp_state_topic')
    robot_ip = LaunchConfiguration('robot_ip')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='10'),
        DeclareLaunchArgument('ros_localhost_only', default_value='0'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('target_cmd_topic', default_value='/ur5/target_cmd'),
        DeclareLaunchArgument('tcp_state_topic', default_value='/ur5/tcp_state'),
        DeclareLaunchArgument('publish_rate_hz', default_value='125.0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.2'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', ros_localhost_only),
        Node(
            package='ur5_rtde_control',
            executable='teleoperation_control_ui_executor',
            output='screen',
            name='teleoperation_executor_node',
            parameters=[
                {
                    'robot_ip': robot_ip,
                    'target_cmd_topic': target_cmd_topic,
                    'tcp_state_topic': tcp_state_topic,
                    'publish_rate_hz': publish_rate_hz,
                    'command_timeout_sec': command_timeout_sec,
                },
            ],
        ),
    ])
