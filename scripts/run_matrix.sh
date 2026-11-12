#!/usr/bin/env bash
# Run a batch of scored landings (fresh SITL each run) and print the summary.
#   scripts/run_matrix.sh            # default matrix
#   SEEDS="0 1 2" scripts/run_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
export DOCKER_CONFIG=${DOCKER_CONFIG:-$PWD/.docker}
export DOCKER_HOST=${DOCKER_HOST:-unix://$HOME/.colima/default/docker.sock}
PY=.venv/bin/python
LOG=${LOG:-logs/matrix_$(date +%Y%m%d_%H%M%S).csv}
SEEDS=${SEEDS:-0}

run() {
    docker-compose restart sitl >/dev/null 2>&1
    sleep 12
    echo "== $*"
    $PY -m scripts.land_mission --log "$LOG" "$@" | grep -E "^  (result|error_m|vz_touch|deck_tilt)"
}

for seed in $SEEDS; do
    run --scenario sim/scenarios/calm.yaml --seed "$seed" --no-vision
    run --scenario sim/scenarios/calm.yaml --seed "$seed"
    for s in moderate rough; do
        run --scenario "sim/scenarios/$s.yaml" --seed "$seed"
        run --scenario "sim/scenarios/$s.yaml" --seed "$seed" --stabilised
    done
done

echo
column -s, -t < "$LOG" | cut -c1-160
