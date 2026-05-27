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
  # port      = DeclareLaunchArgument('port_',      default_value='/dev/ttyUSBinspire')

  # read params from YAML
  set_param_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(package_path,'param', 'insactuator_server_node.yaml'),
    description='Path to parameters YAML file'
  )
  
  # Define node
  insactuator_node = Node(
    package='inspire_actuator',
    executable='insactuator_server_node',
    name='insactuator_server_node',
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
  ld.add_action(insactuator_node)

  return ld
