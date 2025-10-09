#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi
echo "Done."

echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."

echo "Permissions:"
ls -FAlh $XAUTH

echo "Running docker container..."

CONTAINER_NAME="qadt-dev"
IMAGE_NAME="qadt-image"
GIT_DIR="$HOME/git"

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

docker run -it \s3
    -p 18570:18570/udp \
    -e DISPLAY=:0 \
    -e XAUTHORITY=/home/qadt/.Xauthority \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/.Xauthority:/home/qadt/.Xauthority:ro" \
    -v /dev:/dev \
    -v /var/run/dbus/:/var/run/dbus/:z \
    -v "$GIT_DIR:/home/qadt/git" \
    --privileged \
    --name "$CONTAINER_NAME" \
    "$IMAGE_NAME"

# Extra commands for 'docker run':
#   --gpus all      Unable to use for WSL. Use with a native Linux installation. Not via WSL or running a
#                   linux docker container on Mac.
