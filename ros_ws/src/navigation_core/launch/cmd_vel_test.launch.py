from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cmd_vel_test_node = Node(
        package='navigation_core',
        executable='cmd_vel_test',
        name='cmd_vel_test',
        output='screen',
        parameters=[{
            'orbit': False,
            'triangle': True,
            'linear_speed': 0.3,
            'angular_speed': 0.4
        }]
    )

    cmd_vel_to_px4_bridge_node = Node(
        package='navigation_core',
        executable='cmd_vel_to_px4',
        name='cmd_vel_to_px4',
        output='screen',
        parameters=[{
            'fixed_altitude': -2.0,
            'setpoint_rate': 20.0
        }]
    )

    return LaunchDescription([
        cmd_vel_test_node,
        cmd_vel_to_px4_bridge_node
    ])
