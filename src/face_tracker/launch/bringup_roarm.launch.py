from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    roarm_driver = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen',
    )

    tracker = Node(
        package='face_tracker',
        executable='tracker_node',
        name='face_tracker_node',
        output='screen',
        parameters=[{
            'camera_index': 2,
            'show_view': True,
            'dead_zone': 0.08,
        }],
    )

    controller = Node(
        package='face_tracker',
        executable='controller_node',
        name='face_controller_node',
        output='screen',
        parameters=[{
            'lost_mode': 'return',
            'valid_required_frames': 5,
            'dead_zone': 0.08,

            'gain_pan': 2.0,
            'gain_tilt': 2.0,
            'max_rate_pan_deg_s': 35.0,
            'max_rate_tilt_deg_s': 25.0,

            'pan_limit_deg': 120.0,
            'tilt_limit_deg': 120.0,

            'return_rate_pan_deg_s': 10.0,
            'return_rate_tilt_deg_s': 8.0,

            'log_cmd': False,
        }],
    )

    fake_driver = Node(
        package='face_tracker',
        executable='fake_driver_node',
        name='fake_driver_node',
        output='screen',
        parameters=[{
            'pan_joint': 'base_link_to_link1',
            'tilt_joint': 'link1_to_link2',
            'elbow_joint': 'link2_to_link3',
            'wrist_joint': 'link3_to_gripper_link',
            'pan_limit_deg': 120.0,
            'tilt_limit_deg': 120.0,
            'publish_hz': 15.0,
            'invert_pan': True,
            'invert_tilt': True,
            'hold_other_joints': False,
            'pitch_mode': 'elbow',
        }],
    )

    return LaunchDescription([roarm_driver, tracker, controller, fake_driver])
