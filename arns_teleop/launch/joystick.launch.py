from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )
   
   arns_teleop_pkg = get_package_share_directory("arns_teleop")
   joystick_config_file = os.path.join(arns_teleop_pkg, "config/joystick.yaml") 
   
   start_joy_node = Node(
      package="joy",
      executable="joy_node",
      parameters=[joystick_config_file, {"use_sim_time": use_sim_time}]
   )
   
   start_teleop_twist_joy = Node(
      package="teleop_twist_joy",
      executable="teleop_node",
      name="teleop_node",
      parameters=[joystick_config_file, {"use_sim_time": use_sim_time}],
      remappings=[
         ("/cmd_vel", "/diff_controller/cmd_vel_unstamped")
      ]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      
      start_joy_node,
      start_teleop_twist_joy
   ])