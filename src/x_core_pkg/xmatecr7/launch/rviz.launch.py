import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定义包名和URDF文件名
    package_name = 'xmatecr7'
    urdf_name = "xmatecr7.urdf"

    # 创建LaunchDescription对象
    ld = LaunchDescription()

    # 获取包的路径
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)

    # 读取URDF文件内容并传递给robot_state_publisher
    with open(urdf_model_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz2节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'urdf', 'config.rviz')]  # 提供配置文件路径
    )

    # 添加节点到launch描述
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld