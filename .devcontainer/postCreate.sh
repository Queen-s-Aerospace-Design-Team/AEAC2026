#!/bin/bash
set -e

echo "Installing extra software..."

sudo apt update >/dev/null
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gzharmonic >/dev/null

echo "Starting Micro XRCE Agent..."
MicroXRCEAgent udp4 -p 8888
