from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
   
   gui = LaunchConfiguration("gui")
   use_sim_time = LaunchConfiguration("use_sim_time")
   rviz_config_file = LaunchConfiguration("rviz_config_file")
   
   
   arns_description_pkg = get_package_share_directory("arns_description")
   rviz_config_path = os.path.join(arns_description_pkg, 'rviz/description.rviz')
   

   declare_gui = DeclareLaunchArgument(
      name="gui",
      default_value="False",
      description="Use joint state publisher GUI if True"
   )
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config_file",
      default_value=rviz_config_path,
      description="rviz2 config file path"
   )
   
   start_rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_description_pkg, 'launch/rsp.launch.py')]),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   start_rviz = Node(
      package="rviz2",
      executable="rviz2",
      arguments=["-d", rviz_config_file],
      parameters=[
         {"use_sim_time": use_sim_time}
      ],
      output="screen"
   )
   
   start_joint_state_publisher = Node(
      condition=UnlessCondition(gui),
      package="joint_state_publisher",
      executable="joint_state_publisher",
   )
   
   start_joint_state_publisher_gui = Node(
      condition=IfCondition(gui),
      package="joint_state_publisher_gui",
      executable="joint_state_publisher_gui",
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_rviz_config_file,
      declare_gui,
      
      start_rsp,
      start_joint_state_publisher,
      start_joint_state_publisher_gui,
      start_rviz,
   ])