import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('asm_description')
    xacro_file = os.path.join(package_dir, 'urdf', 'ur5_with_asm.urdf.xacro')

    ur_type = LaunchConfiguration('ur_type').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context)
    asm_prefix = LaunchConfiguration('asm_prefix').perform(context)
    asm_parent_link = LaunchConfiguration('asm_parent_link').perform(context)
    asm_mount_xyz = LaunchConfiguration('asm_mount_xyz').perform(context)
    asm_mount_rpy = LaunchConfiguration('asm_mount_rpy').perform(context)

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'ur_type': ur_type,
            'tf_prefix': tf_prefix,
            'asm_prefix': asm_prefix,
            'asm_parent_link': asm_parent_link,
            'asm_mount_xyz': asm_mount_xyz,
            'asm_mount_rpy': asm_mount_rpy,
        },
    )

    robot_description = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }
        ],
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [robot_state_publisher_node, joint_state_publisher_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub',
            default_value='true',
            description='Whether to launch joint_state_publisher_gui',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(get_package_share_directory('asm_description'), 'rviz', 'asm_description.rviz'),
            description='Full path to the RViz config file to use',
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5',
            description='UR model type to load',
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='TF prefix for the UR robot',
        ),
        DeclareLaunchArgument(
            'asm_prefix',
            default_value='asm_',
            description='Prefix for the attached ASM model',
        ),
        DeclareLaunchArgument(
            'asm_parent_link',
            default_value='tool0',
            description='UR link used as the ASM mount parent',
        ),
        DeclareLaunchArgument(
            'asm_mount_xyz',
            default_value='0 0 0',
            description='ASM mount translation relative to the parent link',
        ),
        DeclareLaunchArgument(
            'asm_mount_rpy',
            default_value='0 0 0',
            description='ASM mount rotation relative to the parent link',
        ),
        OpaqueFunction(function=launch_setup),
    ])
