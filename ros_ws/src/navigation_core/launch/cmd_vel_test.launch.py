from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cmd_vel_test_node = Node(
        package='navigation_core',
        executable='cmd_vel_test',
        name='cmd_vel_test',
        output='screen',
        parameters=[{
            'orbit': True,
            'triangle': False,
            'linear_speed': 0.5,
            'angular_speed': 0.25,
            'twist_rate': 10.0
        }]
    )

    cmd_vel_to_px4_bridge_node = Node(
        package='navigation_core',
        executable='cmd_vel_to_px4',
        name='cmd_vel_to_px4',
        output='screen',
        parameters=[{
            'fixed_altitude': -2.5,
            'setpoint_rate': 10.0
        }]
    )

    return LaunchDescription([
        cmd_vel_test_node,
        cmd_vel_to_px4_bridge_node
    ])
