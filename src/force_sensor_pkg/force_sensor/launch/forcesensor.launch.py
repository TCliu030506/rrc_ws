import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument       # 声明launch文件内使用的Argument类


def generate_launch_description():             # 自动生成launch文件的函数
    #config = os.path.expanduser('~/zzrobot_ws/src/force_sensor_pkg/force_sensor/config/saaam_calibration.yaml') 
    return LaunchDescription([                 # 返回launch文件的描述信息
        DeclareLaunchArgument('pubrate', default_value= '100'), # 创建一个Launch文件内参数（arg）
        DeclareLaunchArgument('forcesensorport', default_value= '/dev/ttyUSB0'),
        Node(                                  # 配置一个节点的启动
            package='force_sensor',          # 节点所在的功能包
            executable='force_sensor_node', # 节点的可执行文件
            parameters=[{"forcesensor_rate": LaunchConfiguration('pubrate')},{LaunchConfiguration('forcesensorport')}],
        ),
    ])