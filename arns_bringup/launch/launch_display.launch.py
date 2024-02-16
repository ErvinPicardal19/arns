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
   pkg_description = os.path.join(get_package_share_directory("arns_description"))
   pkg_teleop = os.path.join(get_package_share_directory("arns_teleop"))
   pkg_navigation = os.path.join(get_package_share_directory("arns_navigation"))
   pkg_bringup = os.path.join(get_package_share_directory("arns_bringup"))
   
   pkg_slam = os.path.join(get_package_share_directory("arns_slam"))
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value='False',
      description="Use gazebo simulation time"
   )
   
   # map = LaunchConfiguration("map")
   # declare_map_path = DeclareLaunchArgument(
   #    name="map", 
   #    default_value=os.path.join(pkg_slam, "maps", "map_real_save.yaml"),
   #    description="amcl map to load")
   
   # start_joy_teleop = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(pkg_teleop, 'launch', 'joystick.launch.py')])
   # )
   
   # start_amcl = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(
   #       pkg_slam, 'launch', 'localization_launch.py'
   #    )]), launch_arguments={'use_sim_time': use_sim_time, 'map': map}.items()
   # )
   
   # start_navigation = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(
   #       pkg_navigation, 'launch', 'navigation_launch.py'
   #    )]), launch_arguments={'use_sim_time': use_sim_time}.items()
   # )
   
   start_voice_cmd = Node(
      package='arns_voice_cmd',
         executable='voice_cmd_node',
         output='screen'
   )
   
   
   start_rviz2 = Node(
      package="rviz2",
      executable="rviz2",
      arguments=['-d', os.path.join(pkg_description, "rviz/default.rviz")],
      parameters=[{"use_sim_time": use_sim_time}]
   )
   
   # start_camera = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(
   #       pkg_bringup, 'launch', 'camera.launch.py'
   #    )])
   # )
   
   return LaunchDescription([
      declare_use_sim_time,
      # declare_map_path,
      
      # start_joy_teleop,
      # start_amcl,
      start_voice_cmd,
      # start_camera,
      # start_navigation,
      start_rviz2
   ])
