from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_control_wrench = LaunchConfiguration('topic_control_wrench')
    topic_compensated_wrench = LaunchConfiguration('topic_compensated_wrench')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    tf_lookup_timeout_sec = LaunchConfiguration('tf_lookup_timeout_sec')
    normal_axis = LaunchConfiguration('normal_axis')
    normal_axis_sign = LaunchConfiguration('normal_axis_sign')

    contact_force_threshold = LaunchConfiguration('contact_force_threshold')
    release_force_threshold = LaunchConfiguration('release_force_threshold')
    force_target = LaunchConfiguration('force_target')
    force_rate_limit = LaunchConfiguration('force_rate_limit')
    force_tracking_epsilon = LaunchConfiguration('force_tracking_epsilon')

    force_x = LaunchConfiguration('force_x')
    force_y = LaunchConfiguration('force_y')
    force_z = LaunchConfiguration('force_z')
    torque_x = LaunchConfiguration('torque_x')
    torque_y = LaunchConfiguration('torque_y')
    torque_z = LaunchConfiguration('torque_z')

    return LaunchDescription([
        DeclareLaunchArgument('topic_control_wrench', default_value='/arm_admittance_control/control_wrench'),
        DeclareLaunchArgument('topic_compensated_wrench', default_value='/wrench_compensated'),
        DeclareLaunchArgument('frame_id', default_value='asm_ee_site'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),
        DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.05'),
        DeclareLaunchArgument('normal_axis', default_value='z'),
        DeclareLaunchArgument('normal_axis_sign', default_value='1.0'),
        DeclareLaunchArgument('contact_force_threshold', default_value='2.0'),
        DeclareLaunchArgument('release_force_threshold', default_value='1.0'),
        DeclareLaunchArgument('force_target', default_value='10.0'),
        DeclareLaunchArgument('force_rate_limit', default_value='20.0'),
        DeclareLaunchArgument('force_tracking_epsilon', default_value='0.5'),
        DeclareLaunchArgument('force_x', default_value='0.0'),
        DeclareLaunchArgument('force_y', default_value='0.0'),
        DeclareLaunchArgument('force_z', default_value='0.0'),
        DeclareLaunchArgument('torque_x', default_value='0.0'),
        DeclareLaunchArgument('torque_y', default_value='0.0'),
        DeclareLaunchArgument('torque_z', default_value='0.0'),
        Node(
            package='robot_admittance_control',
            executable='adaptive_control_wrench_publisher_node',
            name='adaptive_control_wrench_publisher_node',
            output='screen',
            parameters=[{
                'topic_control_wrench': topic_control_wrench,
                'topic_compensated_wrench': topic_compensated_wrench,
                'frame_id': frame_id,
                'publish_rate': publish_rate,
                'tf_lookup_timeout_sec': tf_lookup_timeout_sec,
                'normal_axis': normal_axis,
                'normal_axis_sign': normal_axis_sign,
                'contact_force_threshold': contact_force_threshold,
                'release_force_threshold': release_force_threshold,
                'force_target': force_target,
                'force_rate_limit': force_rate_limit,
                'force_tracking_epsilon': force_tracking_epsilon,
                'force_x': force_x,
                'force_y': force_y,
                'force_z': force_z,
                'torque_x': torque_x,
                'torque_y': torque_y,
                'torque_z': torque_z,
            }],
        ),
    ])