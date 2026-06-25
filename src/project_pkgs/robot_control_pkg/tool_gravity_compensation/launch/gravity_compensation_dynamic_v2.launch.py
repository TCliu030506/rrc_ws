from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tool_gravity_compensation')
    params = os.path.join(pkg_share, 'config', 'gravity_compensation_dynamic_v2_params.yaml')

    return LaunchDescription([
        Node(
            package='tool_gravity_compensation',
            executable='dynamic_gravity_compensation_node',
            name='dynamic_gravity_compensation_node',
            output='screen',
            parameters=[params],
        ),
    ])
