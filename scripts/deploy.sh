#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/deployment/compose.deployment.yml"

die() {
    echo "Error: $*" >&2
    exit 1
}

usage() {
    cat <<USAGE
Usage: $(basename "$0") [command]

Commands:
up       Start deployment in background (default)
down     Stop and remove deployment containers
restart  Restart deployment containers
logs     Follow deployment logs
status   Show deployment container status

Notes:
- Uses compose file: $COMPOSE_FILE
- Image pulling behavior is controlled in compose via pull_policy (currently 'missing').
USAGE
}

command -v docker >/dev/null 2>&1 || die "docker is not installed or not in PATH"
[ -f "$COMPOSE_FILE" ] || die "compose file not found: $COMPOSE_FILE"

docker compose version >/dev/null 2>&1 || die "docker compose plugin is unavailable"

run_compose() {
  docker compose -f "$COMPOSE_FILE" "$@"
}

ARG="${1:-up}" # default to up

case "$ARG" in
    up)
        run_compose up -d --remove-orphans
        ;;
    down)
        run_compose down --remove-orphans
        ;;
    restart)
        run_compose restart
        ;;
    logs)
        run_compose logs -f --tail=200
        ;;
    status)
        run_compose ps
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        die "unknown command: $ARG
    "
        ;;
esac
