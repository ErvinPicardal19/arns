from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
   
   serial_port_path = LaunchConfiguration("serial_port")
   frame_id = LaunchConfiguration("frame_id")
   angle_compensate = LaunchConfiguration("angle_compensate")
   scan_mode = LaunchConfiguration("scan_mode")
   serial_baudrate = LaunchConfiguration("serial_baudrate")
   
   declare_serial_port_path = DeclareLaunchArgument(
      name="serial_port",
      default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
      description="Full path to the LIDAR serial port"
   )
   
   declare_frame_id = DeclareLaunchArgument(
      name="frame_id",
      default_value="lidar_link",
      description="Frame link of the lidar in robot description"
   )
   
   declare_angle_compensate = DeclareLaunchArgument(
      name="angle_compensate",
      default_value="True",
      description="Enable angle_compensate if True"
   )
   
   declare_scan_mode = DeclareLaunchArgument(
      name="scan_mode",
      default_value="Standard",
      description="Define lidar scan mode"
   )
   
   declare_serial_baudrate = DeclareLaunchArgument(
      name="serial_baudrate",
      default_value="115200",
      description="Specify serial baudrate"
   )
   
   start_rplidar_a1 = Node(
      package="rplidar_ros",
      executable="rplidar_composition",
      parameters=[{
         "serial_port": serial_port_path,
         "frame_id": frame_id,
         "angle_compensate": angle_compensate,
         "scan_mode": scan_mode,
         "serial_baudrate": serial_baudrate
      }],
      output="screen"
   )
   
   return LaunchDescription([
      declare_serial_port_path,
      declare_frame_id,
      declare_angle_compensate,
      declare_scan_mode,
      declare_serial_baudrate,
      
      start_rplidar_a1
   ])