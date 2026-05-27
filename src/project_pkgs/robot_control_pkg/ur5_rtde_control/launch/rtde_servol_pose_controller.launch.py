from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_controller_node(context):
    robot_ip = LaunchConfiguration('robot_ip')
    topic_cmd_pose = LaunchConfiguration('topic_cmd_pose')
    speed = LaunchConfiguration('speed')
    acceleration = LaunchConfiguration('acceleration')
    lookahead_time = LaunchConfiguration('lookahead_time')
    gain = LaunchConfiguration('gain')
    enable_servo_verify = LaunchConfiguration('enable_servo_verify')
    verify_log_interval_sec = LaunchConfiguration('verify_log_interval_sec')
    verify_topic = LaunchConfiguration('verify_topic')
    verify_pos_err_warn_m = LaunchConfiguration('verify_pos_err_warn_m')
    verify_rot_err_warn_rad = LaunchConfiguration('verify_rot_err_warn_rad')

    tcp_offset = [
        float(LaunchConfiguration('tcp_x').perform(context)),
        float(LaunchConfiguration('tcp_y').perform(context)),
        float(LaunchConfiguration('tcp_z').perform(context)),
        float(LaunchConfiguration('tcp_rx').perform(context)),
        float(LaunchConfiguration('tcp_ry').perform(context)),
        float(LaunchConfiguration('tcp_rz').perform(context)),
    ]

    return [
        Node(
            package='ur5_rtde_control',
            executable='rtde_servol_pose_controller_node',
            name='rtde_servol_pose_controller_node',
            output='screen',
            parameters=[
                {
                    'robot_ip': robot_ip,
                    'topic_cmd_pose': topic_cmd_pose,
                    'speed': speed,
                    'acceleration': acceleration,
                    'lookahead_time': lookahead_time,
                    'gain': gain,
                    'enable_servo_verify': enable_servo_verify,
                    'verify_log_interval_sec': verify_log_interval_sec,
                    'verify_topic': verify_topic,
                    'verify_pos_err_warn_m': verify_pos_err_warn_m,
                    'verify_rot_err_warn_rad': verify_rot_err_warn_rad,
                    'tcp_offset': tcp_offset,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('topic_cmd_pose', default_value='/arm_desired_pose'),
        DeclareLaunchArgument('speed', default_value='0.15'),
        DeclareLaunchArgument('acceleration', default_value='0.1'),
        DeclareLaunchArgument('lookahead_time', default_value='0.1'),
        DeclareLaunchArgument('gain', default_value='300.0'),
        DeclareLaunchArgument('enable_servo_verify', default_value='true'),
        DeclareLaunchArgument('verify_log_interval_sec', default_value='0.5'),
        DeclareLaunchArgument('verify_topic', default_value='/ur5/servol_verify'),
        DeclareLaunchArgument('verify_pos_err_warn_m', default_value='0.02'),
        DeclareLaunchArgument('verify_rot_err_warn_rad', default_value='0.15'),
        DeclareLaunchArgument('tcp_x', default_value='0.0'),
        DeclareLaunchArgument('tcp_y', default_value='0.0'),
        DeclareLaunchArgument('tcp_z', default_value='0.0'),
        DeclareLaunchArgument('tcp_rx', default_value='0.0'),
        DeclareLaunchArgument('tcp_ry', default_value='0.0'),
        DeclareLaunchArgument('tcp_rz', default_value='0.0'),
        OpaqueFunction(function=_create_controller_node),
    ])
