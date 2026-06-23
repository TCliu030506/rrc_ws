from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 硬件和 TF 相关配置集中在这里修改，避免通过 ros2 launch 参数层层转发。
UR_TYPE = 'ur5'
ROBOT_IP = '192.168.1.102'
FORCE_SENSOR_PORT = '/dev/ttyUSB0'

BASE_FRAME = 'base'
TOOL_FRAME = 'tool0'
ASM_EE_FRAME = 'asm_ee_site'
ASM_VERSION = 3
RAW_WRENCH_TOPIC = '/external_force_torque_wrench'
EE_POSE_TOPIC = '/asm_ee_site/pose'
EE_TWIST_TOPIC = '/asm_ee_site/twist'


def generate_launch_description() -> LaunchDescription:

    # Realense2 相机节点
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

    # UR 机器人驱动节点
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': UR_TYPE,
            'robot_ip': ROBOT_IP,
            'launch_rviz': 'false',
        }.items(),
    )

    # UR 机器人额外状态发布节点()
    ur_msg_pub_node = Node(
        package= 'ur5_msg',
        executable= 'ur5_msg_pub',
        output= 'screen'
      )

    # 双编码器节点
    encoder_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('brt_rs485_encoder_pkg'),
                'launch',
                'brt_rs485_encoder_dual.launch.py',
            ])
        ),
    )

    # 力传感器节点
    force_sensor_node = Node(
        package='force_sensor',
        executable='force_sensor_axis_6',
        name='force_sensor_axis_6',
        output='screen',
        parameters=[{
            'forcesensorport': FORCE_SENSOR_PORT,
            'forcesensor_rate': 200,
            'baudrate': 115200,
            'frame_id': 'asm_force_sensor_link',
            'topic_name': RAW_WRENCH_TOPIC,
            'auto_zero': False,
        }],
    )

    # ASM 工具 TF 广播节点
    asm_tool_tf_broadcaster = Node(
        package='ultra_scanning_system',
        executable='asm_tool_tf_broadcaster',
        name='asm_tool_tf_broadcaster',
        output='screen',
        parameters=[{
            'asm_version': ASM_VERSION,
            'parent_frame': TOOL_FRAME,
            'publish_rate': 125.0,
        }],
    )

    # ASM 工具位姿节点：从 TF 计算末端执行器位姿并发布到话题
    ee_state_from_tf_node = Node(
        package='ultra_scanning_sim',
        executable='ee_state_from_tf_node',
        name='asm_ee_state_from_tf_node',
        output='screen',
        parameters=[{
            'source_frame': BASE_FRAME,
            'target_frame': ASM_EE_FRAME,
            'output_pose_topic': EE_POSE_TOPIC,
            'output_twist_topic': EE_TWIST_TOPIC,
            'publish_rate': 125.0,
            'max_angular_speed': 10.0,
        }],
    )

    return LaunchDescription([
        TimerAction(period=0.0, actions=[ur_driver_launch]),
        TimerAction(period=3.0, actions=[realsense_node]),
        TimerAction(period=3.0, actions=[force_sensor_node]),
        TimerAction(period=3.0, actions=[ur_msg_pub_node]),
        TimerAction(period=3.0, actions=[encoder_dual_launch]),
        TimerAction(period=3.0, actions=[asm_tool_tf_broadcaster]),
        TimerAction(period=3.0, actions=[ee_state_from_tf_node]),
    ])
