from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

   use_sim_time = LaunchConfiguration("use_sim_time")
   
   declare_use_sim_time = DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use Gazebo clock if True"
   )
   
   start_teleop_keyboard = Node(
      package="teleop_twist_keyboard",
      executable="teleop_twist_keyboard",
      parameters=[
         {"use_sim_time": use_sim_time}
      ],
      remappings=[
         ("/cmd_vel", "/cmd_vel_key")
      ]
   )
   
   return LaunchDescription([
      declare_use_sim_time,
      
      start_teleop_keyboard
   ])