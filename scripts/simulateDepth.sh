#!/bin/bash

# Depth Camera x500 Simulation:
# - Must launch while in devcontainer (duh)
# - ros2 bridge with Gazebo
# - Expose point cloud messages and visualize them in rviz2
# - Automatically launch px4_sitl with param: gz_x500_depth_walls ('_depth' for depth camera, and '_walls' for walls map)
# - All of these commands are running in a tmux instance.

# gz msgs -> ros2 msgs:
# - /depth_camera/points @ gz.msgs.PointCloudPacked -> /depth_camera/points @ sensor_msgs/msg/PointCloud2
# - /depth_camera @ gz.msgs.Image -> /depth_camera @ sensor_msgs/msg/Image
# - /camera_info @ gz.msgs.CameraInfo -> /camera_info @ sensor_msgs/msg/CameraInfo
# - /world/walls/model/x500_depth_0/link/lidar_sensor_link/sensor/lidar/scan @ gz.msgs.LaserScan -> /depth_camera/laser_scan @ sensor_msgs/msg/LaserScan
# Frame: 'x500_depth_0/camera_link/StereoOV7251'. StereoOV7251 with a captial 'o'.

set -euo pipefail

SESSION="${TMUX_SESSION:-x500_depth_cam}"
GZ_MODEL_NAME="x500_depth_0"
GZ_WORLD_NAME="walls"
PX4_TARGET="gz_x500_depth_$GZ_WORLD_NAME"
PX4_DIR="$HOME/PX4-Autopilot"
FIXED_FRAME="x500_depth_0/camera_link/StereoOV7251"
RVIZ_CFG="/tmp/x500_depth.rviz"

die() 
{ 
    echo "Error: $*" >&2; exit 1;
}

command -v tmux >/dev/null 2>&1 || die "tmux not found"

# If this session already exists, just attach to it.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux attach -t "$SESSION"
    exit 0
fi

# Setup rviz2 configuration
cat > "$RVIZ_CFG" <<EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: ${FIXED_FRAME}
    Frame Rate: 60
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/PointCloud2
      Name: DepthCloud
      Enabled: true
      Topic: /depth_camera/points
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/PublishPoint
Window Geometry:
  Height: 900
  Width: 1400
  X: 50
  Y: 50
EOF

PX4_CMD="cd ${PX4_DIR} && make px4_sitl ${PX4_TARGET}"
BRIDGE_CMD="sleep 3 && ros2 run ros_gz_bridge parameter_bridge \
    /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
    /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
RVIZ_CMD="sleep 3 && rviz2 -d $RVIZ_CFG --ros-args -p use_sim_time:=true"
RQT_CMD="sleep 3 && ros2 run rqt_image_view rqt_image_view /depth_camera"

tmux new-session -d -s "$SESSION" -n sim zsh

# Create a 2x2 grid of panes
tmux split-window -t "$SESSION:sim" -h -c "#{pane_current_path}" zsh
tmux split-window -t "$SESSION:sim.0" -v -c "#{pane_current_path}" zsh
tmux split-window -t "$SESSION:sim.1" -v -c "#{pane_current_path}" zsh
tmux split-window -t "$SESSION:sim.2" -v -c "#{pane_current_path}" zsh

tmux send-keys -t "$SESSION:sim.0" "$PX4_CMD" C-m
tmux send-keys -t "$SESSION:sim.1" "$BRIDGE_CMD" C-m
tmux send-keys -t "$SESSION:sim.2" "$RVIZ_CMD" C-m
tmux send-keys -t "$SESSION:sim.3" "$RQT_CMD" C-m

tmux select-pane -t "$SESSION:sim.0"
tmux attach -t "$SESSION"

