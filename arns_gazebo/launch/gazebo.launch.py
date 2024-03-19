from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
   
   arns_gazebo_pkg = get_package_share_directory("arns_gazebo")
   arns_description_pkg = get_package_share_directory("arns_description")
   gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
   gazebo_params_file = os.path.join(arns_gazebo_pkg, "config/gazebo_params.yaml")
   default_world_path = os.path.join(arns_gazebo_pkg, "worlds/indoor.world")
   
   use_ros2_control = LaunchConfiguration("use_ros2_control")
   world = LaunchConfiguration("world")


   declare_use_ros2_control = DeclareLaunchArgument(
      name="use_ros2_control",
      default_value="True",
      description="Use ros2_control if True"
   )
   
   declare_world_path = DeclareLaunchArgument(
      name="world",
      default_value=default_world_path,
      description="Path to your gazebo world file"
   )
   
   # use_sim_time is always True for this launch file
   start_rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_description_pkg, 'launch/rsp.launch.py')]),
      launch_arguments={"use_sim_time": "True", "use_ros2_control": use_ros2_control}.items()
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
   
   start_diff_controller = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      name="diff_controller",
      arguments=["diff_controller", "--controller-manager", "/controller_manager"]
   )
   
   start_joint_broadcaster = Node(
      condition=IfCondition(use_ros2_control),
      package="controller_manager",
      executable="spawner",
      name="joint_broadcaster",
      arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"]
   )
   
   twist_mux_config = os.path.join(get_package_share_directory("arns_teleop"), "config/twist_mux.yaml")
   start_twist_mux = Node(
      package="twist_mux",
      executable="twist_mux",
      parameters=[twist_mux_config],
      remappings=[
         ("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")
      ]
   )

   
   return LaunchDescription([
      declare_world_path,
      declare_use_ros2_control,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      start_rsp,
      start_gazebo,
      start_spawner,
      start_joint_broadcaster,
      start_twist_mux
   ])