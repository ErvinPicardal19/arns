import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   use_sim_time = LaunchConfiguration('use_sim_time')
   
   rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('arns_description'), 'launch', 'rsp.launch.py'
      )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
   )
   
   robot_description = Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])
   controllers_param = os.path.join(get_package_share_directory('arns_bringup'), "config", "controllers.yaml")
   
   controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[{"robot_description": robot_description}, controllers_param]
   )
   
   start_delayed_controller_manager = TimerAction(period=2.0, actions=[controller_manager])
   
   diff_cont_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['diff_cont']
   )
   
   start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_cont_controller]))
   
   joint_broad_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['joint_broad']
   )
   
   start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_controller]))
   
   return LaunchDescription([
      DeclareLaunchArgument(
         name='use_ros2_control',
         default_value='true',
         description="Use ros2_control if true"
      ),
      
      DeclareLaunchArgument(
         name='use_sim_time',
         default_value='false',
         description='Use sim time if true'),
      
      rsp,
      
      start_delayed_controller_manager,
      start_delayed_diff_drive_spawner,
      start_delayed_joint_broadcaster_spawner
      
      # RegisterEventHandler(
      #    event_handler=OnProcessExit(
      #       target_action=diff_cont_controller,
      #       on_exit=[]
      #    )
      # ),
      
   ])