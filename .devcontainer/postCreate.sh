#!/bin/bash

sudo apt update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

MicroXRCEAgent udp4 -p 8888 
