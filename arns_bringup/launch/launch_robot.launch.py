import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   use_sim_time = LaunchConfiguration('use_sim_time')
   
   
   pkg_description = get_package_share_directory('arns_description')
   pkg_bringup = get_package_share_directory('arns_bringup')
   pkg_navigation = get_package_share_directory('arns_navigation')
   pkg_teleop = os.path.join(get_package_share_directory("arns_teleop"))
   
   ekf_param_file = os.path.join(pkg_navigation, "config/ekf_params.yaml")
   
   # start_joy_teleop = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(pkg_teleop, 'launch', 'joystick.launch.py')])
   # )
   
   rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_description, 'launch', 'rsp.launch.py'
      )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
   )
   
   robot_description = Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])
   controllers_param = os.path.join(pkg_bringup, "config", "controllers.yaml")
   
   controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[{"robot_description": robot_description}, controllers_param]
   )
   
   joint_broad_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['joint_broad'],
      parameters=[controllers_param]
   )
   
   diff_cont_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['diff_cont'],
      parameters=[controllers_param]
   )
   
   start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_param_file,
                    {'use_sim_time': use_sim_time}])
    
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
      
      
      RegisterEventHandler(
         event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_controller]
         )
      ),
      
      RegisterEventHandler(
         event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_cont_controller]
         )
      ),
      
      # RegisterEventHandler(
      #    event_handler=OnProcessExit(
      #       target_action=diff_cont_controller,
      #       on_exit=[]
      #    )
      # ),
      
      # start_joy_teleop,
      start_robot_localization_cmd,
      rsp,
      TimerAction(period=3.0, actions=[controller_manager])
   ])