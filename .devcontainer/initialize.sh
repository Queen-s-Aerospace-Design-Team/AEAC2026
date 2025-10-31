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

        # Check if nvidia-container-toolkit is already installed
        if ! dpkg -s nvidia-container-toolkit &>/dev/null; then
            echo "🔧 Installing NVIDIA Container Toolkit..."

            # Ensure prerequisites are available
            sudo apt-get update -y
            sudo apt-get install -y --no-install-recommends curl gnupg2 ca-certificates

            # Add NVIDIA repository and GPG key
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
                | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
            curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
                | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' \
                | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

            sudo apt-get update -y
            sudo apt-get install -y nvidia-container-toolkit

            echo "Configuring Docker to use NVIDIA runtime..."
            sudo nvidia-ctk runtime configure --runtime=docker
            sudo systemctl restart docker || true
            echo "NVIDIA Container Toolkit is ready for use."
        else
            echo "NVIDIA Container Toolkit already installed."
        fi

    else
        echo "NVIDIA driver not found. Skipping GPU setup."
    fi
}

# --- Main execution flow ---
general

if [ "$PROFILE" = "linux" ]; then
    linux
fi

# --- Activate Docker Compose profile ---
cp .devcontainer/compose.${PROFILE}.yml .devcontainer/compose.active.yml
echo "Active Docker Compose profile set to: ${PROFILE}"
