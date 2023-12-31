# Launch file to start the RPLidar A1 sensor

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        name='serial_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Lidar serial port'
    )
    frame_id_arg = DeclareLaunchArgument(
        name='frame_id',
        default_value='lidar_link',
        description='Lidar transform frame'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        name='angle_compensate',
        default_value='True',
        description=''
    )
    scan_mode_arg = DeclareLaunchArgument(
        name='scan_mode',
        default_value='Standard',
        description='Lidar scan mode'
    )
    baud_rate_arg = DeclareLaunchArgument(
        name='serial_baudrate',
        default_value='115200',
        description='Lidar UART baudrate'
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        angle_compensate_arg,
        scan_mode_arg,
        baud_rate_arg,

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'angle_compensate': LaunchConfiguration('angle_compensate'),
                'scan_mode': LaunchConfiguration('scan_mode'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate')
            }]
        )
    ])