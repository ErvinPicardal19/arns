from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
import os
import xacro 

def generate_launch_description():
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   use_ros2_control = LaunchConfiguration("use_ros2_control")
   
   arns_description_pkg = get_package_share_directory("arns_description")
   urdf_file = os.path.join(arns_description_pkg, "urdf/robot.urdf.xacro")
   robot_description_raw = Command(['xacro ', urdf_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
   
   declare_use_sim_time_arg = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="True",
      description="Use Gazebo clock if True"
   )
   
   declare_use_ros2_control_arg = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
   )

   params = {"robot_description": robot_description_raw, "use_sim_time": use_sim_time}
   start_robot_state_publisher = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      parameters=[params]
   )
   
   return LaunchDescription([
      declare_use_sim_time_arg,
      declare_use_ros2_control_arg,
      
      start_robot_state_publisher
   ])

