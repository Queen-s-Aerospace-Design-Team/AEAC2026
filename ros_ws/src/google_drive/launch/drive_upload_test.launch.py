import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('google_drive')
    default_file_path = os.path.join(package_share, 'test_data', 'test_image.jpg')

    file_path_arg = DeclareLaunchArgument(
        'file_path',
        default_value=default_file_path,
        description='Absolute path to the file to upload to Google Drive.'
    )

    uploader_node = Node(
        package='google_drive',
        executable='drive_uploader',
        name='drive_upload_service',
        output='screen'
    )

    test_client_node = Node(
        package='google_drive',
        executable='drive_upload_test_client',
        name='drive_upload_test_client',
        output='screen',
        parameters=[{
            'file_path': LaunchConfiguration('file_path'),
        }]
    )

    return LaunchDescription([
        file_path_arg,
        uploader_node,
        test_client_node,
    ])
