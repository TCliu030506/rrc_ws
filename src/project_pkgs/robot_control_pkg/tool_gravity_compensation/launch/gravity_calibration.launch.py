from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tool_gravity_compensation')
    params = os.path.join(pkg_share, 'config', 'gravity_calibration_params.yaml')

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    ur_type = 'ur5'
    robot_ip = '192.168.1.102'
    launch_rviz = 'false'

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': launch_rviz,
        }.items()
    )

    return LaunchDescription([
        ur_driver_launch,
        Node(
            package='force_sensor',
            executable='force_sensor_axis_6',
            name='force_sensor_axis_6',
            output='screen',
            parameters=[{
                'forcesensorport': '/dev/ttyUSB0',
                'forcesensor_rate': 100,
                'baudrate': 115200,
                'frame_id': 'base_frame',
                'topic_name': 'external_force_torque_wrench',
                'auto_zero': False,
            }],
        ),
        Node(
            package='robot_admittance_control',
            executable='flange_to_sensor_static_tf',
            name='flange_to_sensor_static_tf'
        ),
        Node(
            package='tool_gravity_compensation',
            executable='gravity_calibration_node',
            name='gravity_calibration_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='tool_gravity_compensation',
            executable='auto_gravity_calibration_node',
            name='auto_gravity_calibration_node',
            output='screen',
        ),
    ])
