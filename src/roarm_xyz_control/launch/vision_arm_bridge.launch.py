from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roarm_xyz_control',
            executable='uv_to_arm_node',
            name='uv_to_arm_node',
            output='screen',
            parameters=[
                {
                    'fixed_z': 0.0,
                    'fixed_t': 3.14,
                    'min_x': 180.0,
                    'max_x': 500.0,
                    'min_y': -180.0,
                    'max_y': 260.0,
                    'publish_only_when_detected': True,
                }
            ]
        ),
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
                    'min_x': -999.0,
                    'max_x': 999.0,
                    'min_y': -999.0,
                    'max_y': 999.0,
                    'min_z': -999.0,
                    'max_z': 999.0,
                    'min_t': -10.0,
                    'max_t': 10.0,
                }
            ]
        ),
    ])