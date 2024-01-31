# Launch file to start the RPLidar A1 sensor

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   image_size_arg = DeclareLaunchArgument(
      name='image_size',
      default_value="[640, 480]",
      description='Camera resolution'
   )
   camera_frame_id_arg = DeclareLaunchArgument(
        name='camera_frame_id',
        default_value='camera_link_optical',
        description='Camera transform frame'
    )
   # time_per_frame = DeclareLaunchArgument(
   #    name='time_per_frame',
   #    default_value='[1, 6]',
   #    description=''
   # )

   return LaunchDescription([
      image_size_arg,
      camera_frame_id_arg,
      # time_per_frame,

      Node(
         package='v4l2_camera',
         executable='v4l2_camera_node',
         output='screen',
         parameters=[{
               'image_size': LaunchConfiguration('image_size'),
               'camera_frame_id': LaunchConfiguration('camera_frame_id'),
               # 'time_per_frame': LaunchConfiguration('time_per_frame')
         }]
      )
   ])