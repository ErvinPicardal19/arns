import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   
   pkg_path = os.path.join(get_package_share_directory("arns_gazebo"))
   pkg_description = os.path.join(get_package_share_directory("arns_description"))
   
   gazebo_directory = os.path.join(get_package_share_directory("gazebo_ros"))
   gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value='True',
      description="Use gazebo simulation time"
   )
   
   world_path = LaunchConfiguration("world")
   declare_world_path = DeclareLaunchArgument(
      name="world",
      default_value=os.path.join(pkg_path, "worlds", "empty.world"),
      description="Gazebo world path to load")
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'rsp.launch.py')]),
      launch_arguments={'use_sim_time': use_sim_time}.items()
   )
   
   start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(gazebo_directory, 'launch', 'gazebo.launch.py')]),
      launch_arguments={"world": world_path, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
   )
   
   spawn_entity = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      arguments=['-topic', 'robot_description',
                 '-entity', 'robot'],
      output='screen'
   )
   
   start_rviz2 = Node(
      package="rviz2",
      executable="rviz2",
      arguments=['-d', os.path.join(pkg_description, "rviz/default.rviz")],
      parameters=[{"use_sim_time": use_sim_time}]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_world_path,
      
      start_robot_state_publisher,
      start_gazebo,
      spawn_entity,
      start_rviz2
   ])
