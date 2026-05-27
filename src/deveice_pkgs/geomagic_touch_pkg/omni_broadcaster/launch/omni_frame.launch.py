from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "/map"},
                {"units": "mm"}
           ]
        ),
        Node(
            package='omni_broadcaster',
            executable='omni_broadcaster',
            name='broadcaster1',
            parameters=[
                {'framename': 'frame1'}
            ]
        ),
    ])
