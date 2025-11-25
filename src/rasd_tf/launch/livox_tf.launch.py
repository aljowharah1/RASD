from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_static_tf',
            arguments=[
                '0',          # x
                '0',          # y
                '0.55',       # z
                '0',          # roll
                '0.17453',    # pitch
                '0',          # yaw
                'base_link',
                'livox_frame'
            ]
        )
    ])

