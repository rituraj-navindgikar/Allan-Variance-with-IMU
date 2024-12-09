from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.execute_local import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    usb_port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'port',
            default_value=usb_port,
            description='Connected USB port'),

        Node(
            package='imu_driver', 
            executable='imu_node', 
            name='imu_node', 
            output='screen',
            emulate_tty=True,
            arguments=['port', usb_port],
        ),
    ])