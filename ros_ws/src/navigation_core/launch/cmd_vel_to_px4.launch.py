from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_core',
            executable='cmd_vel_to_px4',
            name='cmd_vel_to_px4',
            output='screen',
            parameters=[{
                'fixed_altitude': -2.0,
                'setpoint_rate': 20.0
            }]
        )
    ])
