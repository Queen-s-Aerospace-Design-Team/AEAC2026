#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVCONTAINER_DIR="$SCRIPT_DIR/../.devcontainer"
CONTAINER_NAME="qadt-devcontainer"
COMPOSE_FILES=(
  -f "$DEVCONTAINER_DIR/compose.base.yml"
  -f "$DEVCONTAINER_DIR/compose.active.yml"
)

# Run initialize script
"$DEVCONTAINER_DIR/initialize.sh"

# Remove conflicting container if it exists
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Removing existing container '$CONTAINER_NAME'..."
    docker rm -f "$CONTAINER_NAME"
fi

# Start container
docker compose "${COMPOSE_FILES[@]}" up -d
docker exec -it "$CONTAINER_NAME" bash
docker compose "${COMPOSE_FILES[@]}" down
