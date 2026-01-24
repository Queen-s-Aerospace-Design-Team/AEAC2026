#!/bin/bash

# This script is meant to install all software required to run the QADT dev environment on native linux machines. Regardless of hardware restrictions.
# The following software will be installed:
# 	- Docker Engine (not docker desktop)
#	- VSCode (via apt repo)
#	- QGroundControl (installed into ~/Downloads folder)
# This script will also confirm git user settings

# ------- Configure Git User Settings -------

echo "=== Git user configuration ==="

CURRENT_NAME="$(git config --global --get user.name || true)"
CURRENT_EMAIL="$(git config --global --get user.email || true)"

# Function to prompt and set git config
configure_git_user() {
    echo "Your name and email will show up in every commit that you make set them accordingly."
    echo
    read -r -p "Enter your Git user name (this can also be your full name): " GIT_NAME
    read -r -p "Enter your Git user email (personal or school, doesn't matter): " GIT_EMAIL

    git config --global user.name "$GIT_NAME"
    git config --global user.email "$GIT_EMAIL"

    echo "Git global user.name set to: $GIT_NAME"
    echo "Git global user.email set to: $GIT_EMAIL"
}

# If both exist, confirm with user
if [ -n "$CURRENT_NAME" ] && [ -n "$CURRENT_EMAIL" ]; then
    echo "Current Git global configuration:"
    echo "  Name : $CURRENT_NAME"
    echo "  Email: $CURRENT_EMAIL"
    echo

    read -r -p "Do you want to keep these settings? [Y/n]: " REPLY

    case "$REPLY" in
        [nN]|[nN][oO])
            configure_git_user
            ;;
        *)
            echo "Keeping existing Git configuration."
            ;;
    esac
else
    echo "Git global user.name and/or user.email is not set."
    configure_git_user
fi

echo
echo "=== Git configuration complete ==="
echo
# ------- Docker -------

# Instructions at: https://docs.docker.com/engine/install/ubuntu/

echo "=== Docker setup ==="
echo

if ! grep -qi ubuntu /etc/os-release; then
    echo "This script currently supports Ubuntu only."
    exit 1
fi

# check if docker command exists
if command -v docker &>/dev/null; then
    echo "Docker is already installed."

    # Ensure docker daemon is running
    if ! systemctl is-active --quiet docker; then
        echo "Docker service is not running. Starting it..."
        sudo systemctl start docker
    fi

    # Ensure docker starts on boot
    sudo systemctl enable docker

else
    echo "Docker not found. Installing Docker Engine..."

    # Remove older or conflicting packages (safe even if not present)
    sudo apt remove -y \
        docker.io \
        docker-doc \
        docker-compose \
        docker-compose-v2 \
        podman-docker \
        containerd runc || true

    # Set up Docker's apt repository
    sudo apt update
    sudo apt install -y ca-certificates curl

    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

    sudo apt update

    # Install the Docker Packages
    sudo apt install -y \
        docker-ce \
        docker-ce-cli \
        containerd.io \
        docker-buildx-plugin \
        docker-compose-plugin

    # Enable and start Docker
    sudo systemctl enable docker
    sudo systemctl start docker
fi

# Instructions at: https://docs.docker.com/engine/install/linux-postinstall/

# Ensure user is in the docker group
if groups "$USER" | grep -q '\bdocker\b'; then
    echo "User '$USER' is already in the docker group."
else
    echo "Adding user '$USER' to the docker group..."
    sudo groupadd docker
    sudo usermod -aG docker $USER
    echo "You will need to log out and log back in for docker group changes to take effect."
fi

if docker info &>/dev/null; then
    echo "Docker is ready to use."
else
    echo "Docker installed, but may require logout/login before use."
fi

echo
echo "=== Docker setup complete ==="
echo

# ------- VSCode -------

# Instructions at: https://code.visualstudio.com/docs/setup/linux

echo "=== VS Code setup ==="
echo

# Check if VS Code is already installed
if command -v code >/dev/null 2>&1; then
    echo "VS Code is already installed."
else
    echo "VS Code not found. Installing..."

    # Install prerequisites
    sudo apt-get update
    sudo apt-get install -y wget gpg apt-transport-https

    # Install Microsoft signing key
    if [ ! -f /usr/share/keyrings/microsoft.gpg ]; then
        echo "Installing Microsoft GPG key..."
        wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
            | gpg --dearmor > microsoft.gpg
        sudo install -D -o root -g root -m 644 microsoft.gpg \
            /usr/share/keyrings/microsoft.gpg
        rm -f microsoft.gpg
    else
        echo "Microsoft GPG key already installed."
    fi

    # Add VS Code apt repository
    if [ ! -f /etc/apt/sources.list.d/vscode.sources ]; then
        echo "Adding VS Code APT repository..."
        sudo tee /etc/apt/sources.list.d/vscode.sources >/dev/null <<EOF
Types: deb
URIs: https://packages.microsoft.com/repos/code
Suites: stable
Components: main
Architectures: amd64,arm64,armhf
Signed-By: /usr/share/keyrings/microsoft.gpg
EOF
    else
        echo "VS Code APT repository already exists."
    fi

    # Install VS Code
    sudo apt update
    sudo apt install -y code
fi

# Install base VS Code extensions
echo "Installing VS Code extensions..."

if command -v code >/dev/null 2>&1; then
    code --install-extension ms-vscode-remote.remote-containers --force
    code --install-extension ms-azuretools.vscode-docker --force
else
    echo "WARNING: 'code' command not found; skipping extension install."
fi

echo
echo "=== VS Code setup complete ==="
echo

# ------- QGroundControl -------

# Instructions at: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

echo "=== QGroundControl setup ==="
echo

QGC_DIR="$HOME/Downloads"
QGC_APPIMAGE="$QGC_DIR/QGroundControl.AppImage"
QGC_URL="https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"

mkdir -p "$QGC_DIR" # Make sure downloads exist (prob does)

sudo apt install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev

echo "Downloading QGroundControl AppImage..."
curl -L "$QGC_URL" -o "$QGC_APPIMAGE"
chmod +x "$QGC_APPIMAGE"

echo "QGroundControl installed at: $QGC_APPIMAGE"
echo "You can launch it with:"
echo "  $QGC_APPIMAGE"

echo
echo "=== QGroundControl setup complete ==="
echo

# ------- Finish -------

echo "=== Setup complete ==="
echo "Some changes (Docker group membership, drivers, services) may require a reboot."
echo

read -r -p "Would you like to reboot now? [y/N]: " REBOOT_REPLY

case "$REBOOT_REPLY" in
    [yY]|[yY][eE][sS])
        echo "Rebooting..."
        sudo reboot
        ;;
    *)
        echo "Please remember to log out or reboot later for all changes to take effect."
        ;;
esac
