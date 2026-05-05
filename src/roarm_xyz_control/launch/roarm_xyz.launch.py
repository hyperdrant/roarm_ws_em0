from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roarm_xyz_control',
            executable='roarm_json_driver_node',
            name='roarm_json_driver_node',
            output='screen',
            parameters=[
                {
                    'port': '/dev/ttyUSB0',
                    'baudrate': 115200,
                    'cmd_type': 1041,

                    'min_x': -300.0,
                    'max_x': 300.0,
                    'min_y': -300.0,
                    'max_y': 300.0,
                    'min_z': -300.0,
                    'max_z': 300.0,
                    'min_t': -6.28,
                    'max_t': 6.28,
                }
            ]
        ),
        Node(
            package='roarm_xyz_control',
            executable='keyboard_target_node',
            name='keyboard_target_node',
            output='screen',
        ),
    ])