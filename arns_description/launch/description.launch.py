import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
   
   use_sim_time = LaunchConfiguration("use_sim_time");
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use gazebo simulation time"
   )
   
   use_gui = LaunchConfiguration("use_gui");
   declare_use_joint_state_publisher_gui = DeclareLaunchArgument(
      name="use_gui",
      default_value="False",
      description="Use joint state publisher gui"
   )
   
   pkg_path = os.path.join(get_package_share_directory("arns_description"))
   
   start_joint_state_publisher = Node(
      package="joint_state_publisher",
      executable="joint_state_publisher",
      condition=UnlessCondition(use_gui)
   )
   
   start_joint_state_publisher_gui = Node(
      package="joint_state_publisher_gui",
      executable="joint_state_publisher_gui",
      condition=IfCondition(use_gui)
   )
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(pkg_path, "launch/rsp.launch.py")]),
      launch_arguments={'use_sim_time': use_sim_time }.items()
   )
   
   start_rviz2 = Node(
      package="rviz2",
      executable="rviz2",
      arguments=['-d', os.path.join(pkg_path, "rviz/default.rviz")],
      parameters=[{"use_sim_time": use_sim_time}],
      output=["log"]
   )
   
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_use_joint_state_publisher_gui,
      
      
      start_joint_state_publisher_gui,
      start_joint_state_publisher,
      start_robot_state_publisher,
      start_rviz2
   ])
