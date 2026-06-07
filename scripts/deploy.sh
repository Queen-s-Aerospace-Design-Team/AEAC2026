#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DEPLOYMENT_DIR="$SCRIPT_DIR/../deployment"

COMPOSE_BASE="$REPO_DEPLOYMENT_DIR/compose.deployment.yml"
COMPOSE_MISSION="$REPO_DEPLOYMENT_DIR/compose.mission.yml"
COMPOSE_PERCEPTION="$REPO_DEPLOYMENT_DIR/compose.perception.yml"
COMPOSE_DRIVE="$REPO_DEPLOYMENT_DIR/compose.drive.yml"
COMPOSE_UXRCE="$REPO_DEPLOYMENT_DIR/compose.uxrce.yml"
COMPOSE_HARDWARE="$REPO_DEPLOYMENT_DIR/compose.hardware.yml"

COMPOSE_FILES=(
    -f "$COMPOSE_BASE"
    -f "$COMPOSE_MISSION"
    -f "$COMPOSE_PERCEPTION"
    -f "$COMPOSE_DRIVE"
    -f "$COMPOSE_UXRCE"
    -f "$COMPOSE_HARDWARE"
)

AVAILABLE_SERVICES=(
    "mission"
    "perception"
    "drive"
    "uxrce"
    "hardware-controller"
)

STARTUP_LOG_LINES="100"
CMD="${1:-up}" # CMD defaults to 'up'
COMPOSE_TARGET="${2:-all}" # Defaults to composing all files
RESTART_TARGET="${2:-all}"
LOGS_TARGET="${2:-all}"
ATTACH_TARGET="${2:-mission}" # attach default to mission service
PULL_POLICY="${PULL_POLICY:-missing}" # defaults to 'docker compose ... --pull=missing'

die() {
    echo "Error: $*" >&2
    exit 1
}

usage() {
    cat <<'USAGE'
Usage: ./deploy [command] [target]

Commands:
- up       Start deployment (default)
- down     Stop and remove deployment containers
- restart  Restart deployment containers
- logs     Follow deployment logs
- status   Show deployment container status
- attach   Open an interactive shell in the running deployment container

Targets:
- all
- mission
- perception
- drive
- uxrce
- hardware

Notes:
- target 'all' specifies all targets
- attach defaults to mission
- Set PULL_POLICY=always to force `docker compose up` to pull updated images
    - Possible PULL_POLICY values: always|missing|never
    - PULL_POLICY defaults to 'missing'

Examples:
- ./deploy up
- export PULL_POLICY=always && ./deploy up
- ./deploy up perception
- ./deploy logs perception
- ./deploy restart 
- ./deploy attach mission
USAGE
}

run_compose() {
    # -p supplues the compose project name. Causes docker to group resources under this
    # specific project name. Makes compose down easier to execute.
    if [ "$1" = "up" ] && [ -n "$PULL_POLICY" ]; then
        echo "docker compose up with PULL_POLICY=$PULL_POLICY..."
        docker compose "${COMPOSE_FILES[@]}" -p qadt-deployment "$1" --pull "$PULL_POLICY" "${@:2}"
    else
        docker compose "${COMPOSE_FILES[@]}" -p qadt-deployment "$@"
    fi
}

get_services_for_target() {
    case "$1" in
        all)
            echo "${AVAILABLE_SERVICES[@]}"
            ;;
        mission)
            echo "mission"
            ;;
        perception)
            echo "perception"
            ;;
        drive)
            echo "drive"
            ;;
        uxrce)
            echo "uxrce"
            ;;
        hardware)
            echo "hardware-controller"
            ;;
        *)
            die "invalid target '$1'"
            ;;
    esac
}

UP_SERVICES="$(get_services_for_target "$COMPOSE_TARGET")"
RESTART_SERVICES="$(get_services_for_target "$RESTART_TARGET")"
LOG_SERVICES="$(get_services_for_target "$LOGS_TARGET")"
ATTACH_SERVICE="$(get_services_for_target "$ATTACH_TARGET")"

case "$CMD" in
    up)

        run_compose up -d $UP_SERVICES
        run_compose logs --tail=$STARTUP_LOG_LINES -f $UP_SERVICES
        ;;
    down)
        run_compose down --remove-orphans
        ;;
    restart)
       
        run_compose restart $RESTART_SERVICES
        run_compose logs --tail=$STARTUP_LOG_LINES -f $RESTART_SERVICES
        ;;
    logs)

        run_compose logs --tail=$STARTUP_LOG_LINES -f $LOG_SERVICES
        ;;
    status)
        run_compose ps
        ;;
    attach)
        CONTAINER_ID="$(run_compose ps -q "$ATTACH_SERVICE")"
        [ -n "$CONTAINER_ID" ] || die "service '$ATTACH_TARGET' is not running"

        docker exec -it "$CONTAINER_ID" bash 2>/dev/null
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        die "unknown command: $CMD"
        ;;
esac
