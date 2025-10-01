#!/bin/bash

# ---- Config ----

CONTAINER_NAME="qadt-dev"
IMAGE_NAME="qadt-dev:latest"
GIT_DIR="$HOME/git"

# Optional XRCE Agent and MAVLink ports
XRCE_UDP_PORT="2018"
MAVLINK_UDP_PORT="18570"

# ---- Display using X11 Default ----

XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"
if [[ "${DISPLAY:-}" =~ ^:[0-9]+$ ]]; then
  # simple local X11
  xhost +local:docker >/dev/null 2>&1 || true
  if [[ ! -f $XAUTH ]]; then
    touch $XAUTH
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - || true
    chmod a+r "$XAUTH"
  fi
  DISPLAY_FLAGS=(
    -e DISPLAY="$DISPLAY"
    -v "$XSOCK:$XSOCK:rw"
    -v "$XAUTH:$XAUTH:ro"
    -e XAUTHORITY="$XAUTH"
    -e QT_X11_NO_MITSHM=1
  )
elif [[ -n "${WAYLAND_DISPLAY:-}" && -d "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" ]]; then
  # Wayland host
  RUNTIME="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  DISPLAY_FLAGS=(
    -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY"
    -e XDG_RUNTIME_DIR="$RUNTIME"
    -v "$RUNTIME/$WAYLAND_DISPLAY:$RUNTIME/$WAYLAND_DISPLAY"
  )
else
  echo "No GUI display detected. Running headless."
  DISPLAY_FLAGS=()
fi

# ---- Clean existing container ----

if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
     echo "Container '$CONTAINER_NAME' already exists. Do you want to remove it?" 
     read -p "WARNING: THIS WILL DELETE ANY LOCAL CHANGES YOU MADE IN THE CONTAINER! [y/N] " response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        if docker rm -f $CONTAINER_NAME; then
            echo "Successfully removed '$CONTAINER_NAME'"
        else
            echo "Error: Failed to remove container '$CONTAINER_NAME'"
            exit 1
        fi
    else
        echo "Operation cancelled."
        exit 1
    fi
fi

# ---- Networking ----

NETWORK_FLAGS=(--network host)
# If you cannot use host networking, comment the above and uncomment below:
# NETWORK_FLAGS=(-p "${XRCE_UDP_PORT}:${XRCE_UDP_PORT}/udp" -p "${MAVLINK_UDP_PORT}:${MAVLINK_UDP_PORT}/udp")

# ---- Env for ROS 2 ----

ROS_FLAGS=(
  -e ROS_DISTRO=humble
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
)

# ---- Volumes ----

VOL_FLAGS=(
  -v "$GIT_DIR:/home/qadt/git"
  -v /var/run/dbus:/var/run/dbus:ro
)

# ---- Run Container ----

docker run -it --name "$CONTAINER_NAME" \
  "${DISPLAY_FLAGS[@]}  " \
  "${NETWORK_FLAGS[@]}" \
  --group-add dialout \
  --ulimit rtprio=99 \
  "${VOL_FLAGS[@]}" \
  "$IMAGE_NAME"
