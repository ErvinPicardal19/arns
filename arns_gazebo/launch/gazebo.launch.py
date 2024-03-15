from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   world = LaunchConfiguration("world")
   
   arns_gazebo_pkg = get_package_share_directory("arns_gazebo")
   arns_description_pkg = get_package_share_directory("arns_description")
   gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
   gazebo_params_file = os.path.join(arns_gazebo_pkg, "config/gazebo_params.yaml")
   default_world_path = os.path.join(arns_gazebo_pkg, "worlds/indoor.world")
   
   
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="True",
      description="Use Gazebo clock if True"
   )
   
   declare_world_path = DeclareLaunchArgument(
      name="world",
      default_value=default_world_path,
      description="Path to your gazebo world file"
   )
   
   start_rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_description_pkg, 'launch/rsp.launch.py')]),
      launch_arguments={"use_sim_time": use_sim_time}.items()
   )
   
   # start_gazebo
   
   start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(gazebo_ros_pkg, 'launch/gazebo.launch.py')]),
      launch_arguments={"world": world,"extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file}.items()
   )
   
   #start_spawner
   
   start_spawner = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      arguments=[
         "-topic", "robot_description",
         "-entity", "alexbot"
      ]
   )

   
   return LaunchDescription([
      declare_use_sim_time,
      declare_world_path,
      
      start_rsp,
      start_gazebo,
      start_spawner,
   ])