import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
   
   use_sim_time = LaunchConfiguration("use_sim_time");
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use gazebo simulation time"
   )
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control", 
      default_value="True",
      description="Enable ros2_control if true")
   
   pkg_path = os.path.join(get_package_share_directory("arns_description"))
   xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
   # urdf_file = Command(["xacro ", xacro_file])
   robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])
   
   params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
   start_robot_state_publisher = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="screen",
      parameters=[params]
   )
   
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_ros2_control,

      start_robot_state_publisher
      
   ])
