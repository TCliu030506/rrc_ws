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
    feedback_wrench_topic = LaunchConfiguration('feedback_wrench_topic')
    feedback_topic = LaunchConfiguration('feedback_topic')
    feedback_position = LaunchConfiguration('feedback_position')
    feedback_force_scale = LaunchConfiguration('feedback_force_scale')
    feedback_publish_frequency = LaunchConfiguration('feedback_publish_frequency')
    feedback_enable_force_filter = LaunchConfiguration('feedback_enable_force_filter')
    feedback_deadband_force = LaunchConfiguration('feedback_deadband_force')
    feedback_lowpass_cutoff_hz = LaunchConfiguration('feedback_lowpass_cutoff_hz')
    feedback_max_force_abs = LaunchConfiguration('feedback_max_force_abs')
    feedback_max_force_rate = LaunchConfiguration('feedback_max_force_rate')

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

    teleoperation_feedback_node = Node(
        package='robot_trajectory_planner',
        executable='teleoperation_feedback_node',
        output='screen',
        name='teleoperation_feedback_node',
        parameters=[
            {
                'wrench_topic': feedback_wrench_topic,
                'feedback_topic': feedback_topic,
                'position': feedback_position,
                'force_scale': feedback_force_scale,
                'publish_frequency': feedback_publish_frequency,
                'enable_force_filter': feedback_enable_force_filter,
                'deadband_force': feedback_deadband_force,
                'lowpass_cutoff_hz': feedback_lowpass_cutoff_hz,
                'max_force_abs': feedback_max_force_abs,
                'max_force_rate': feedback_max_force_rate,
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
        DeclareLaunchArgument('tg_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('tg_baudrate', default_value='1000000'),
        DeclareLaunchArgument('tg_protocol', default_value='modbus_rtu'),
        DeclareLaunchArgument('tg_device_address', default_value='1'),

        DeclareLaunchArgument('feedback_wrench_topic', default_value='/external_force_torque_wrench_compensated'),
        DeclareLaunchArgument('feedback_topic', default_value='/phantom/force_feedback'),
        DeclareLaunchArgument('feedback_position', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('feedback_force_scale', default_value='0.05'),
        DeclareLaunchArgument('feedback_publish_frequency', default_value='1000.0'),
        DeclareLaunchArgument('feedback_enable_force_filter', default_value='true'),
        DeclareLaunchArgument('feedback_deadband_force', default_value='0.001'),
        DeclareLaunchArgument('feedback_lowpass_cutoff_hz', default_value='120.0'),
        DeclareLaunchArgument('feedback_max_force_abs', default_value='20.0'),
        DeclareLaunchArgument('feedback_max_force_rate', default_value='400.0'),

        tg_server_launch,
        # img_capture_node,
        # usb_camera_node,
        teleop_master_node,
        teleop_trajectory_node,
        tg_teleoperation_node,
        ui_node,
        teleoperation_feedback_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='tus_ui_node exited, shutting down launch'))
                ],
            )
        ),
    ])
