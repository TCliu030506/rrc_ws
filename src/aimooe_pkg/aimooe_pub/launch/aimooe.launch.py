from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
   return LaunchDescription([
      # 启动节点，并传递环境变量
      Node(
          package='aimooe_pub',
          executable='pub_trackers',
          name='aimooe_node',
          output='screen',
      )      
      # 添加其他启动节点或参数等
   ])