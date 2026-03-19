from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    topic_master_state = LaunchConfiguration('topic_master_state')
    topic_ui_control = LaunchConfiguration('topic_ui_control')
    topic_desired_pose = LaunchConfiguration('topic_desired_pose')
    topic_desired_twist = LaunchConfiguration('topic_desired_twist')
    topic_desired_accel = LaunchConfiguration('topic_desired_accel')

    publish_rate = LaunchConfiguration('publish_rate')
    omni_name = LaunchConfiguration('omni_name')
    reference_frame = LaunchConfiguration('reference_frame')
    units = LaunchConfiguration('units')
    tg_button_topic = LaunchConfiguration('tg_button_topic')
    tg_service_name = LaunchConfiguration('tg_service_name')
    tg_port = LaunchConfiguration('tg_port')
    tg_baudrate = LaunchConfiguration('tg_baudrate')
    tg_protocol = LaunchConfiguration('tg_protocol')
    tg_device_address = LaunchConfiguration('tg_device_address')

    tg_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tg_9801_pkg'),
                'launch',
                'tg_9801.launch.py',
            ])
        ),
        launch_arguments={
            'port': tg_port,
            'baudrate': tg_baudrate,
            'protocol': tg_protocol,
            'device_address': tg_device_address,
        }.items(),
    )

    teleop_master_node = Node(
        package='omni_common',
        executable='omni_state',
        output='screen',
        name='teleoperation_master_node',
        parameters=[
            {'omni_name': omni_name},
            {'publish_rate': 1000},
            {'reference_frame': reference_frame},
            {'units': units},
        ],
    )

    teleop_trajectory_node = Node(
        package='robot_trajectory_planner',
        executable='teleoperation_trajectory_node',
        output='screen',
        name='teleoperation_trajectory_node',
        parameters=[
            {
                'topic_master_state': topic_master_state,
                'topic_ui_control': topic_ui_control,
                'topic_desired_pose': topic_desired_pose,
                'topic_desired_twist': topic_desired_twist,
                'topic_desired_accel': topic_desired_accel,
                'publish_rate': publish_rate,
            }
        ],
    )

    ui_node = Node(
        package='tus_ui',
        executable='tus_ui',
        name='tus_ui_node',
        output='screen',
    )

    img_capture_node = Node(
        package='tus_ui',
        executable='img_capture_node',
        name='img_capture_node',
        output='screen',
    )

    usb_camera_node = Node(
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
    )

    tg_teleoperation_node = Node(
        package='tg_9801_pkg',
        executable='tg_9801_teleoperation_node',
        name='tg_9801_teleoperation_node',
        output='screen',
        parameters=[
            {
                'button_topic': tg_button_topic,
                'service_name': tg_service_name,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('omni_name', default_value='phantom'),
        DeclareLaunchArgument('reference_frame', default_value='/map'),
        DeclareLaunchArgument('units', default_value='mm'),

        DeclareLaunchArgument('topic_master_state', default_value='/phantom/state'),
        DeclareLaunchArgument('topic_ui_control', default_value='tus_control'),
        DeclareLaunchArgument('topic_desired_pose', default_value='/desired_pose'),
        DeclareLaunchArgument('topic_desired_twist', default_value='/desired_twist'),
        DeclareLaunchArgument('topic_desired_accel', default_value='/desired_accel'),
        DeclareLaunchArgument('publish_rate', default_value='125.0'),
        
        DeclareLaunchArgument('tg_button_topic', default_value='/phantom/button'),
        DeclareLaunchArgument('tg_service_name', default_value='set_grip'),
        DeclareLaunchArgument('tg_port', default_value='/dev/tg_gripper'),
        DeclareLaunchArgument('tg_baudrate', default_value='1000000'),
        DeclareLaunchArgument('tg_protocol', default_value='modbus_rtu'),
        DeclareLaunchArgument('tg_device_address', default_value='1'),

        tg_server_launch,
        img_capture_node,
        usb_camera_node,
        teleop_master_node,
        teleop_trajectory_node,
        tg_teleoperation_node,
        ui_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='tus_ui_node exited, shutting down launch'))
                ],
            )
        ),
    ])
