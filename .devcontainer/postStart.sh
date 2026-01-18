#!/bin/bash
set -e

echo "Installing extra software..."

sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros-gzharmonic \
    ros-humble-topic-tools

echo "Starting Micro XRCE Agent..."
MicroXRCEAgent udp4 -p 8888

# Lidar 2D:
# ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
# Frame: x500_lidar_2d_0/link/lidar_2d_v2 # We can always map it to 'map'

# Depth:
# ros2 run ros_gz_bridge parameter_bridge /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
# Frame: x500_depth_0/camera_link/StereoOV7251

# Make a tmux script that launches these automatically