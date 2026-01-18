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