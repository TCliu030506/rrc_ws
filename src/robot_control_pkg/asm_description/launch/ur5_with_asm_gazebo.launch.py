from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5',
            description='Type/series of used UR robot.',
        ),
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='asm_description',
            description='Package with controller configuration.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='ur5_with_asm_controllers.yaml',
            description='YAML file with the controllers configuration.',
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='asm_description',
            description='Package containing the combined UR5+ASM description.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ur5_with_asm.urdf.xacro',
            description='Combined UR5+ASM xacro file.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz?',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Start gazebo with GUI?',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('asm_description'),
                'worlds',
                'myworld.world',
            ]),
            description='Gazebo world file to load.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ur_simulation_gazebo'),
                    'launch',
                    'ur_sim_control.launch.py',
                ])
            ),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'runtime_config_package': LaunchConfiguration('runtime_config_package'),
                'controllers_file': LaunchConfiguration('controllers_file'),
                'description_package': LaunchConfiguration('description_package'),
                'description_file': LaunchConfiguration('description_file'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'world': LaunchConfiguration('world'),
                # 添加初始位置文件参数
                'initial_positions_file': PathJoinSubstitution([
                    FindPackageShare('asm_description'),
                    'config',
                    'my_initial_positions.yaml',
                ]),
            }.items(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['asm_arm_controller', '-c', '/controller_manager'],
            output='screen',
        ),
    ])
