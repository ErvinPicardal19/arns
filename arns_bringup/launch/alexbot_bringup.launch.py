from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

   arns_description_pkg = get_package_share_directory("arns_description")
   arns_bringup_pkg = get_package_share_directory("arns_bringup")
   controllers_config_file = os.path.join(arns_bringup_pkg, "config/controllers.yaml")
   
   # use_sim_time is always False as this launch file will not use Gazebo
   start_rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_description_pkg, 'launch/rsp.launch.py')]),
      launch_arguments={"use_sim_time": "False", "use_ros2_control": "True"}.items()
   )
   
   start_controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      output="screen",
      parameters=[controllers_config_file],
      remappings=[
         ("~/robot_description", "/robot_description"),
      ]
   )
   
   start_diff_controller = Node(
      package="controller_manager",
      executable="spawner",
      name="diff_controller",
      arguments=["diff_controller", "--controller-manager", "/controller_manager"]
   )
   
   start_joint_broadcaster = Node(
      package="controller_manager",
      executable="spawner",
      name="joint_broadcaster",
      arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"]
   )
   
   start_rplidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_bringup_pkg, "launch/rplidar.launch.py")])
   )
   
   start_camera = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_bringup_pkg, "launch/camera.launch.py")])
   )

   
   return LaunchDescription([
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      start_controller_manager,
      start_rsp,
      start_rplidar,
      start_camera,
      start_joint_broadcaster
      
   ])