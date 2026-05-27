import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
  
  package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

  print(package_path)

  # Declare launch arguments
  # pub_rate  = DeclareLaunchArgument('pub_rate_',  default_value='30')
  # port      = DeclareLaunchArgument('port_',      default_value='/dev/ttyUSB0')

  # read params from YAML
  set_param_cmd = DeclareLaunchArgument(
    'params_file',  # 声明了一个可从命令行传入的参数，用于指定参数文件的路径
    default_value=os.path.join(package_path,'param', 'cybergear_server_node.yaml'),  # 默认值为param目录下的cybergear_server_node.yaml文件
    description='Path to parameters YAML file'
  )
  
  # Define node
  cybergear_node = Node(
    package='cybergear_actuator',
    executable='cybergear_server_node',
    name='cybergear_server_node',
    output='screen',
    parameters=[
      # {'pub_rate': LaunchConfiguration('pub_rate_')},
      # {'port': LaunchConfiguration('port_')}
      LaunchConfiguration('params_file')
    ]
  )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Add the actions to the launch description
  # ld.add_action(pub_rate)
  # ld.add_action(port)
  ld.add_action(set_param_cmd)
  ld.add_action(cybergear_node)

  return ld
