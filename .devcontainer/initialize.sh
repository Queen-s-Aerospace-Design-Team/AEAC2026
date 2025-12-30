#!/usr/bin/env bash
set -e

general() {
    # --- Detect platform ---
    OS=$(uname -s)
    PROFILE=linux

    if [ "$OS" = "Darwin" ]; then
        PROFILE=macos
    elif grep -qi microsoft /proc/version 2>/dev/null; then
        PROFILE=wsl
    fi
}

linux() {
    # --- GPU setup ---
    echo "Checking for NVIDIA GPU drivers..."

    if command -v nvidia-smi &>/dev/null; then
        echo "NVIDIA driver found."
        PROFILE+="-NVIDIA" # Profile is now NVIDIA

        # Check if nvidia-container-toolkit is already installed
        if ! dpkg -s nvidia-container-toolkit &>/dev/null; then
            echo "🔧 Installing NVIDIA Container Toolkit..."

            # Ensure prerequisites are available
            sudo apt-get update
            sudo apt-get install --no-install-recommends curl gnupg2 ca-certificates -y

            # Add NVIDIA repository and GPG key
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
                | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
            curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
                | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' \
                | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

            sudo apt-get update
            sudo apt-get install nvidia-container-toolkit -y

            echo "Configuring Docker to use NVIDIA runtime..."
            sudo nvidia-ctk runtime configure --runtime=docker
            sudo systemctl restart docker || true
            echo "NVIDIA Container Toolkit is ready for use."
        else
            echo "NVIDIA Container Toolkit already installed."
        fi

    else
        echo "NVIDIA driver not found. Skipping GPU setup."
        PROFILE+="-nonNVIDIA" # Profile is now nonNVIDIA

        if [ -e /dev/dri/renderD128 ]; then
            echo "DRM GPU device found (Intel/AMD). Hardware acceleration available."
        else
            echo "No DRM GPU device found. Hardware acceleration unavailable."
        fi
    fi
}

macos() {
    echo "Configuring macOS for container GUIs..."

    # 1) Ensure Homebrew
    if ! command -v brew >/dev/null 2>&1; then
        echo "Homebrew is required. Install from https://brew.sh and re-run."
        exit 1
    fi

    # 2) Install XQuartz if missing
    if ! [ -d "/Applications/Utilities/XQuartz.app" ]; then
        echo "Installing XQuartz..."
        brew install --cask xquartz
    fi

    # 3) Enable indirect GLX + allow network clients
    defaults write org.xquartz.X11 enable_iglx -bool true
    defaults write org.xquartz.X11 nolisten_tcp -bool false

    # 4) Start XQuartz if it isn't already running
    if ! pgrep -x XQuartz >/dev/null; then
        open -a XQuartz
        sleep 2 # brief wait so DISPLAY :0 comes up
    fi

    # 5) Allow localhost TCP connections
    xhost +localhost >/dev/null

    echo "XQuartz is ready."
}

# --- Main execution flow ---
general

if [ "$PROFILE" = "linux" ]; then
    linux
elif [ "$PROFILE" = "macos" ]; then
    macos
fi

# --- Activate Docker Compose profile ---
cp .devcontainer/compose.${PROFILE}.yml .devcontainer/compose.active.yml
echo "Active Docker Compose profile set to: ${PROFILE}"
