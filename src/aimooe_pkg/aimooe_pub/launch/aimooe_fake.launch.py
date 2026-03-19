from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'frame',
         default_value='fake_tool',
         description='pubed msgs frame name'
      ),

      # 启动节点，并传递环境变量
      Node(
          package='aimooe_pub',
          executable='pub_test',
          name='pub_test_node',
          output='screen',
          parameters=[{'frame':LaunchConfiguration('frame')}]
      )
      
      # 添加其他启动节点或参数等
   ])