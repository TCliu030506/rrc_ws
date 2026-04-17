from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    ros_localhost_only = LaunchConfiguration('ros_localhost_only')
    target_cmd_topic = LaunchConfiguration('target_cmd_topic')
    tcp_state_topic = LaunchConfiguration('tcp_state_topic')

    ui_node = Node(
        package='tus_ui',
        executable='tus_ui',
        name='tus_ui_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='10'),
        DeclareLaunchArgument('ros_localhost_only', default_value='0'),
        DeclareLaunchArgument('target_cmd_topic', default_value='/ur5/target_cmd'),
        DeclareLaunchArgument('tcp_state_topic', default_value='/ur5/tcp_state'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', ros_localhost_only),
        Node(
            package='tus_ui',
            executable='img_capture_node',
            name='img_capture_node',
            output='screen',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_camera_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 1280},
                {'image_height': 720},
                {'framerate': 15.0},
                {'pixel_format': 'mjpeg2rgb'},
            ],
            remappings=[
                ('image_raw', 'usb_images'),
            ],
        ),
        Node(
            package='omni_common',
            executable='omni_state',
            output='screen',
            name='teleoperation_master_node',
            parameters=[
                {'omni_name': 'phantom'},
                {'publish_rate': 1000},
                {'reference_frame': '/map'},
                {'units': 'mm'},
            ],
        ),
        Node(
            package='ur5_rtde_control',
            executable='teleoperation_control_ui_pub',
            output='screen',
            name='teleoperation_target_pose_pub_node',
            parameters=[
                {
                    'target_cmd_topic': target_cmd_topic,
                    'tcp_state_topic': tcp_state_topic,
                },
            ],
        ),
        ui_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[EmitEvent(event=Shutdown(reason='tus_ui_node exited, shutting down launch'))],
            )
        ),
    ])
