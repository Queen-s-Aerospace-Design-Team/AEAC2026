# inside initializeCommand
OS=$(uname -s)
PROFILE=linux

if [ "$OS" = "Darwin" ]; then PROFILE=macos
elif grep -qi microsoft /proc/version 2>/dev/null; then PROFILE=wsl
fi

cp .devcontainer/compose.${PROFILE}.yml .devcontainer/compose.active.yml
