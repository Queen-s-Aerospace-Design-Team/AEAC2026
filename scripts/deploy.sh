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

STARTUP_LOG_LINES="100"
CMD="${1:-up}" # CMD defaults to 'up'
COMPOSE_TARGET="${2:-all}" # Defaults to composing all files
RESTART_TARGET="${2:-all}"
LOGS_TARGET="${2:-all}"
ATTACH_TARGET="${2:-mission}" # attach default to mission service

die() {
    echo "Error: $*" >&2
    exit 1
}

usage() {
    cat <<'USAGE'
Usage: ./deploy [command] [target]

Commands:
up       Start deployment in background (default)
down     Stop and remove deployment containers
restart  Restart deployment containers
logs     Follow deployment logs
status   Show deployment container status
attach   Open an interactive shell in the running deployment container

Notes:
- target: all|mission|perception|drive|uxrce|hardware
- attach defaults to mission
- logs/restart use target as the service when target != all
- Image pulling behavior is controlled in compose via pull_policy (currently 'missing').
- After `up`, prints a number of log lines for specified service

Examples:
- ./deploy
- ./deploy up perception
- ./deploy logs perception
- ./deploy restart all
- ./deploy attach mission
USAGE
}

command -v docker >/dev/null 2>&1 || die "docker is not installed or not in PATH"
[ -f "$COMPOSE_BASE" ] || die "compose file not found: $COMPOSE_BASE"

docker compose version >/dev/null 2>&1 || die "docker compose plugin is unavailable"

# Added a helper 
set_compose_files() {
    case "$1" in
        mission)
            COMPOSE_FILES="-f $COMPOSE_MISSION"
            ;;
        perception)
            COMPOSE_FILES="-f $COMPOSE_PERCEPTION"
            ;;
        drive)
            COMPOSE_FILES="-f $COMPOSE_DRIVE"
            ;;
        uxrce)
            COMPOSE_FILES="-f $COMPOSE_UXRCE"
            ;;
        hardware)
            COMPOSE_FILES="-f $COMPOSE_HARDWARE"
            ;;
        all|*)
            COMPOSE_FILES="-f $COMPOSE_MISSION \
                           -f $COMPOSE_PERCEPTION \
                           -f $COMPOSE_DRIVE \
                           -f $COMPOSE_UXRCE \
                           -f $COMPOSE_HARDWARE"
            ;;
    esac
}

run_compose() {
    # -p supplues the compose project name. Causes docker to group resources under this
    # specific project name. Makes compose down easier to execute.
    docker compose -p qadt-deployment $COMPOSE_FILES "$@"
}

set_compose_files $COMPOSE_TARGET

case "$CMD" in
    up)
        run_compose up -d
        echo "Recent startup logs (${STARTUP_LOG_LINES} lines):"
        run_compose logs --tail=$STARTUP_LOG_LINES -f
        ;;
    down)
        run_compose down --remove-orphans
        ;;
    restart)
        if [ "$RESTART_TARGET" = "all" ]; then
            run_compose restart
            run_compose logs --tail=$STARTUP_LOG_LINES -f
        else
            run_compose restart "$RESTART_TARGET"
            run_compose logs --tail=$STARTUP_LOG_LINES -f "$RESTART_TARGET"
        fi
        ;;
    logs)
        if [ "$LOGS_TARGET" = "all" ]; then
            run_compose logs --tail=$STARTUP_LOG_LINES -f
        else
            run_compose logs --tail=$STARTUP_LOG_LINES -f "$LOGS_TARGET"
        fi
        ;;
    status)
        run_compose ps
        ;;
    attach)
        CONTAINER_ID="$(run_compose ps -q "$ATTACH_TARGET")"
        [ -n "$CONTAINER_ID" ] || die "service '$ATTACH_TARGET' is not running"

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
        die "unknown command: $CMD"
        ;;
esac
