import os

from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch.actions import IncludeLaunchDescription      # 声明launch文件内使用的Argument,Include类
from launch.launch_description_sources import PythonLaunchDescriptionSource 


MARKER_ID = 0
MARKER_SIZE = 0.04
ARUCO_DICTIONARY_ID = 'DICT_6X6_250'

def generate_launch_description():          
    realsense_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'))
      )
    aruco_launch_action = Node(
        package='ros2_aruco',
        executable='aruco_node',
        output='screen',
        parameters=[{
            'marker_size': MARKER_SIZE,
            'aruco_dictionary_id': ARUCO_DICTIONARY_ID,
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }],
      )
    aruco_pose_relay_action = Node(
        package='hand-eye',
        executable='aruco_marker_pose_relay.py',
        output='screen',
        parameters=[{
            'target_marker_id': MARKER_ID,
            'input_topic': 'aruco_markers',
            'output_topic': 'aruco_single/pose',
        }],
      )
    hand_eye_action = Node(
        package= 'hand-eye',
        executable= 'handeye_calibration',
        output= 'screen'
      )
    
    rqt_image_action = Node(
        package= 'rqt_image_view',
        executable= 'rqt_image_view',
        output= 'screen'
      )
    
    ld = LaunchDescription()
    ld.add_action(realsense_launch_action)
    ld.add_action(aruco_launch_action)
    ld.add_action(aruco_pose_relay_action)
    ld.add_action(hand_eye_action)
    ld.add_action(rqt_image_action)

    return ld
