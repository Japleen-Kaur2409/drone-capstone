#!/usr/bin/env bash
# One entry point for the project.
#   ./sim.sh up                      start SITL (builds the image the first time)
#   ./sim.sh land [scenario] [opts]  fresh SITL + one scored landing (calm|moderate|rough or a .json path)
#   ./sim.sh matrix                  all scenarios, with/without vision and platform
#   ./sim.sh console                 MAVProxy console (detach: Ctrl-P Ctrl-Q)
#   ./sim.sh down                    stop SITL
#   ./sim.sh test                    offline tests
#   ./sim.sh markers                 printable markers + pad images in markers/
#   ./sim.sh camera [opts]           desk test: live ArUco detection on this computer's camera (Esc quits)
set -euo pipefail
cd "$(dirname "$0")"
ENV_SH_LAZY=1 . scripts/env.sh

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
    # no Docker needed
    test)    $PY -m pytest "$@" ;;
    markers) $PY -m scripts.make_markers "$@" ;;
    camera)  $PY -m vision.aruco_node --no-mav --show "$@" ;;
    # simulator
    up)      docker_env; up ;;
    down)    docker_env; $COMPOSE down ;;
    console) docker_env; docker attach sitl ;;
    matrix)  docker_env; up; scripts/run_matrix.sh ;;
    land)
        docker_env
        scen="${1:-calm}"
        [ $# -gt 0 ] && shift
        [ -f "$scen" ] || scen="sim/scenarios/$scen.json"
        restart
        $PY -m scripts.land_mission --scenario "$scen" "$@"
        ;;
    *) sed -n '2,10p' "$0" | sed 's/^# //' ;;
esac
