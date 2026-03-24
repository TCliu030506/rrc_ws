from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tool_gravity_compensation')
    calib_params = os.path.join(pkg_share, 'config', 'gravity_calibration_params.yaml')
    comp_params = os.path.join(pkg_share, 'config', 'gravity_compensation_params.yaml')

    return LaunchDescription([
        Node(
            package='tool_gravity_compensation',
            executable='gravity_calibration_node',
            name='gravity_calibration_node',
            output='screen',
            parameters=[calib_params],
        ),
        Node(
            package='tool_gravity_compensation',
            executable='gravity_compensation_node',
            name='gravity_compensation_node',
            output='screen',
            parameters=[comp_params],
        ),
    ])
