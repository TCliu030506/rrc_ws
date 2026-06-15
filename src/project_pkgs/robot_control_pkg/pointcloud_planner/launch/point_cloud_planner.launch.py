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

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            'pointcloud.enable': True,
            'enable_gyro': False,
            'enable_accel': False,
        }],
    )

    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5_msg'),
                'launch',
                'ur5.launch.py',
            ])
        ),
    )

    pcl_cloudslam_node = Node(
        package='pointcloud_planner',
        executable='pcl_cloudslam',
        output='screen',
        parameters=[pcl_cloudslam_params],
    )

    return LaunchDescription([
        realsense_node,
        ur5_launch,
        TimerAction(period=2.0, actions=[pcl_cloudslam_node]),
    ])
