#!/usr/bin/env bash
# One entry point for the simulator.
#   ./sim.sh up                      start SITL (builds the image the first time)
#   ./sim.sh land [scenario] [opts]  fresh SITL + one scored landing (calm|moderate|rough or a .json path)
#   ./sim.sh matrix                  all scenarios, with/without vision and platform
#   ./sim.sh test                    offline tests
#   ./sim.sh console                 MAVProxy console (detach: Ctrl-P Ctrl-Q)
#   ./sim.sh down                    stop SITL
set -euo pipefail
cd "$(dirname "$0")"
. scripts/env.sh

running() { docker ps --format '{{.Names}}' | grep -qx sitl; }

up() {
    if running; then
        echo "SITL already running"
        return
    fi
    if ! docker image inspect drone-capstone/ardupilot-sitl:latest >/dev/null 2>&1; then
        echo "Building SITL image (first time only, ~15 min)..."
        $COMPOSE build
    fi
    local since
    since="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    $COMPOSE up -d sitl
    echo "Waiting for SITL..."
    sitl_wait_ready "$since"
}

restart() {
    if running; then
        echo "Resetting SITL..."
        sitl_restart
    else
        up
    fi
}

cmd="${1:-help}"
shift || true
case "$cmd" in
    up)      up ;;
    down)    $COMPOSE down ;;
    console) docker attach sitl ;;
    test)    $PY -m pytest "$@" ;;
    matrix)  up; scripts/run_matrix.sh ;;
    land)
        scen="${1:-calm}"
        [ $# -gt 0 ] && shift
        [ -f "$scen" ] || scen="sim/scenarios/$scen.json"
        restart
        $PY -m scripts.land_mission --scenario "$scen" "$@"
        ;;
    *) sed -n '2,8p' "$0" | sed 's/^# //' ;;
esac
