import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
   
   pkg_gazebo = os.path.join(get_package_share_directory("arns_gazebo"))
   pkg_description = os.path.join(get_package_share_directory("arns_description"))
   pkg_teleop = os.path.join(get_package_share_directory("arns_teleop"))
   pkg_navigation = os.path.join(get_package_share_directory("arns_navigation"))
   
   gazebo_directory = os.path.join(get_package_share_directory("gazebo_ros"))
   gazebo_params_file = os.path.join(pkg_gazebo, "config/gazebo_params.yaml")
   
   ekf_param_file = os.path.join(pkg_navigation, "config/ekf_params.yaml")
   
   use_sim_time = LaunchConfiguration("use_sim_time")
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value='True',
      description="Use gazebo simulation time"
   )
   
   world_path = LaunchConfiguration("world")
   declare_world_path = DeclareLaunchArgument(
      name="world", 
      default_value=os.path.join(pkg_gazebo, "worlds", "empty.world"),
      description="Gazebo world path to load")
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   declare_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control", 
      default_value="True",
      description="Enable ros2_control if true")
   
   start_robot_state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'rsp.launch.py')]),
      launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
   )
   
   start_joy_teleop = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(pkg_teleop, 'launch', 'joystick.launch.py')])
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
   
   spawn_diff_controller = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      arguments=["diff_cont"],
   )
   
   spawn_joint_broadcaster = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      arguments=["joint_broad"],
   )
   
   start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_param_file])
   
   return LaunchDescription([
      declare_use_sim_time,
      declare_world_path,
      declare_ros2_control,
      
      start_robot_state_publisher,
      start_joy_teleop,
      start_gazebo,
      spawn_entity,
      start_rviz2,
      spawn_diff_controller,
      spawn_joint_broadcaster,
      start_robot_localization_cmd
   ])
