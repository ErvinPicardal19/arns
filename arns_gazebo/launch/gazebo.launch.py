from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
   
   arns_gazebo_pkg = get_package_share_directory("arns_gazebo")
   arns_description_pkg = get_package_share_directory("arns_description")
   arns_teleop_pkg = get_package_share_directory("arns_teleop")
   gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
   
   gazebo_params_file = os.path.join(arns_gazebo_pkg, "config/gazebo_params.yaml")
   default_world_path = os.path.join(arns_gazebo_pkg, "worlds/indoor.world")
   
   use_ros2_control = LaunchConfiguration("use_ros2_control")
   world = LaunchConfiguration("world")
   use_rviz = LaunchConfiguration("use_rviz")
   rviz_config_file = LaunchConfiguration("rviz_config")
   
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

   declare_use_rviz = DeclareLaunchArgument(
      name="use_rviz",
      default_value="False",
      description="Start rviz2 if True"
   )
   
   declare_rviz_config_file = DeclareLaunchArgument(
      name="rviz_config",
      default_value=os.path.join(arns_description_pkg, "rviz/real.rviz"),
      description="Full file path for rviz2 config file"
   )
   
   # ROBOT_STATE_PUBLISHER
   
   # use_sim_time is always True for this launch file
   start_rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_description_pkg, 'launch/rsp.launch.py')]),
      launch_arguments={"use_sim_time": "True", "use_ros2_control": use_ros2_control}.items()
   )
   
   # TELEOP NODES
   
   twist_mux_config = os.path.join(get_package_share_directory("arns_teleop"), "config/twist_mux.yaml")
   start_twist_mux = Node(
      package="twist_mux",
      executable="twist_mux",
      parameters=[twist_mux_config],
      remappings=[
         ("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")
      ]
   )
   
   start_joystick = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(arns_teleop_pkg, "launch/joystick.launch.py")]),
      launch_arguments={"use_sim_time": "True"}.items()
   )
   
   # GAZEBO_ROS
   
   start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(gazebo_ros_pkg, 'launch/gazebo.launch.py')]),
      launch_arguments={"world": world,"extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file}.items()
   )
   
   spawn_robot = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      arguments=[
         "-topic", "robot_description",
         "-entity", "alexbot"
      ]
   )
   
   # ROS2_CONTROL

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

   # RVIZ2

   start_rviz = Node(
      condition=IfCondition(use_rviz),
      package="rviz2",
      executable="rviz2",
      arguments=["-d", rviz_config_file],
      parameters=[
         {"use_sim_time": True}
      ]
   )
   
   # ROBOT_LOCALIZATION
   ekf_params = os.path.join(get_package_share_directory("arns_navigation"), "params/ekf.yaml")
   start_robot_localization = Node(
      package="robot_localization",
      executable="ekf_node",
      parameters=[
         ekf_params,
         {"use_sim_time": True}
      ]
   )

   return LaunchDescription([
      declare_world_path,
      declare_use_ros2_control,
      declare_use_rviz,
      declare_rviz_config_file,
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_diff_controller]
         )
      ),
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=start_joint_broadcaster,
            on_exit=[start_rviz]
         )
      ),
      
      start_rsp,
      start_twist_mux,
      start_joystick,
      start_gazebo,
      spawn_robot,
      start_joint_broadcaster,
      start_robot_localization
   ])