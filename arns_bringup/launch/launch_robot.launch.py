import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
   
   pkg_bringup = os.path.join(get_package_share_directory("arns_bringup"))
   pkg_description = os.path.join(get_package_share_directory("arns_description"))
   
   # gazebo_directory = os.path.join(get_package_share_directory("gazebo_ros"))
   # gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value='True',
      description="Use gazebo simulation time"
   )
   
   # world_path = LaunchConfiguration("world")
   # declare_world_path = DeclareLaunchArgument(
   #    name="world", 
   #    default_value=os.path.join(pkg_path, "worlds", "empty.world"),
   #    description="Gazebo world path to load")
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control", 
      default_value="True",
      description="Enable ros2_control if true")
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'rsp.launch.py')]),
      launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
   )
   
   robot_description = Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])
   
   controller_params = os.path.join(pkg_bringup, "config", "controllers.yaml")
   
   controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      arguments=[{'robot_description': robot_description}, controller_params]
   )
   
   delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
   
   # start_rviz2 = Node(
   #    package="rviz2",
   #    executable="rviz2",
   #    arguments=['-d', os.path.join(pkg_description, "rviz/default.rviz")],
   #    parameters=[{"use_sim_time": use_sim_time}]
   # )
   
   spawn_diff_controller = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      arguments=["diff_cont"],
   )
   
   delayed_spawn_diff_controller = RegisterEventHandler(
      event_handler=OnProcessStart(
         target_action=controller_manager,
         on_start=[spawn_diff_controller]
      )
   )
   
   spawn_joint_broadcaster = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      arguments=["joint_broad"],
   )
   
   delayed_spawn_joint_broad_controller = RegisterEventHandler(
      event_handler=OnProcessStart(
         target_action=controller_manager,
         on_start=[spawn_joint_broadcaster]
      )
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      # declare_world_path,
      declare_ros2_control,
      
      start_robot_state_publisher,
      delayed_controller_manager,
      
      delayed_spawn_diff_controller,
      delayed_spawn_joint_broad_controller
   ])
