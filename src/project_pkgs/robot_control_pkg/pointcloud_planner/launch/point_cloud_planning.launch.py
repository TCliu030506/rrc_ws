from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pcl_cloudslam_params = PathJoinSubstitution([
        FindPackageShare('pointcloud_planner'),
        'config',
        'pcl_cloudslam_params.yaml',
    ])

    pcl_cloudslam_node = Node(
        package='pointcloud_planner',
        executable='pcl_cloudslam',
        output='screen',
        parameters=[pcl_cloudslam_params],
    )

    return LaunchDescription([
        TimerAction(period=0.2, actions=[pcl_cloudslam_node]),
    ])
