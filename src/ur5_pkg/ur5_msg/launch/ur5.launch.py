import os

from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription      # 声明launch文件内使用的Argument,Include类
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():          
    #Argument declare   
    robot_ip_arg = DeclareLaunchArgument('ip', default_value= '192.168.1.102')
    ur_type_arg = DeclareLaunchArgument('robot_type',default_value='ur5')
    launch_rviz_arg = DeclareLaunchArgument('rviz',default_value='false',description = "launch rviz?")

    robot_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ur_robot_driver'),'launch','ur_control.launch.py')),
        launch_arguments={'robot_ip':LaunchConfiguration('ip'),'ur_type':LaunchConfiguration('robot_type'),'launch_rviz': LaunchConfiguration('rviz')}.items(),
      )
    ur_msg_pub_action = Node(
        package= 'ur5_msg',
        executable= 'ur5_msg_pub',
        output= 'screen'
      )
    
    ld = LaunchDescription()
    ld.add_action(robot_ip_arg)
    ld.add_action(ur_type_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(robot_launch_action)
    ld.add_action(ur_msg_pub_action)

    return ld

