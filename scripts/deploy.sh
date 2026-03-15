#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/deployment/compose.deployment.yml"
SERVICE_NAME="deploy"
STARTUP_LOG_LINES="100"

die() {
    echo "Error: $*" >&2
    exit 1
}

usage() {
    cat <<'USAGE'
Usage: $(basename "$0") [command]

Commands:
up       Start deployment in background (default)
down     Stop and remove deployment containers
restart  Restart deployment containers
logs     Follow deployment logs
status   Show deployment container status
attach   Open an interactive shell in the running deployment container

Notes:
- Uses compose file: <repo>/deployment/compose.deployment.yml
- Image pulling behavior is controlled in compose via pull_policy (currently 'missing').
- After `up`, prints the last STARTUP_LOG_LINES lines (default: 80) for quick startup visibility.
USAGE
}

command -v docker >/dev/null 2>&1 || die "docker is not installed or not in PATH"
[ -f "$COMPOSE_FILE" ] || die "compose file not found: $COMPOSE_FILE"

docker compose version >/dev/null 2>&1 || die "docker compose plugin is unavailable"

run_compose() {
    docker compose -f $COMPOSE_FILE "$@"
}

ARG="${1:-up}" # default to up

case "$ARG" in
    up)
        run_compose up -d --remove-orphans
        echo "Recent startup logs (${STARTUP_LOG_LINES} lines):"
        run_compose logs --tail=$STARTUP_LOG_LINES -f
        ;;
    down)
        run_compose down --remove-orphans
        ;;
    restart)
        run_compose restart
        run_compose logs --tail=$STARTUP_LOG_LINES -f
        ;;
    logs)
        run_compose logs --tail=$STARTUP_LOG_LINES -f
        ;;
    status)
        run_compose ps
        ;;
    attach)
        CONTAINER_ID="$(run_compose ps -q $SERVICE_NAME)"
        [ -n "$CONTAINER_ID" ] || die "service '$SERVICE_NAME' is not running"

        if docker exec -it "$CONTAINER_ID" bash 2>/dev/null; then
            :
        else
            docker exec -it "$CONTAINER_ID" sh
        fi
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        die "unknown command: $ARG"
        ;;
esac
