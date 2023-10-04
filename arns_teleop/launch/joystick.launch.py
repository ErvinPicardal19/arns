from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   pkg_teleop = get_package_share_directory("arns_teleop")
   
   joy_params = os.path.join(pkg_teleop, 'config', 'joy_params.yaml')
   
   joy_node = Node(
      package="joy",
      executable="joy_node",
      output="screen",
      parameters=[joy_params]
   )
   
   teleop_node = Node(
      package="teleop_twist_joy",
      executable="teleop_node",
      name="teleop_node",
      parameters=[joy_params],
      remappings=[("/cmd_vel","/cmd_vel_joy")]
   )
   
   twist_mux_params = os.path.join(pkg_teleop, 'config', 'twist_mux_params.yaml')
   
   twist_mux = Node(
      package="twist_mux",
      executable="twist_mux",
      parameters=[twist_mux_params],
      remappings=[("cmd_vel_out", "diff_cont/cmd_vel_unstamped")]
   )
   
   return LaunchDescription([
      joy_node,
      teleop_node,
      twist_mux
   ])