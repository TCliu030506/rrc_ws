from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
  package_name = 'x6_control_python'
  
  # read params from YAML
  x6param_path = PathJoinSubstitution([
    FindPackageShare(package_name),
    'param',
    'x6param.yaml'
  ])
  
  # Define node
  x6_server_node = Node(
    package='x6_control_python',
    executable='x6_server',
    name='x6_server_node',
    output='screen',
    parameters=[
      x6param_path
    ]
  )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Add the actions to the launch description
  ld.add_action(x6_server_node)

  return ld