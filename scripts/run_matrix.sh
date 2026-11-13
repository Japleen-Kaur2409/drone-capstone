#!/usr/bin/env bash
# Run a batch of scored landings (fresh SITL each run) and print the summary.
#   scripts/run_matrix.sh            # default matrix
#   SEEDS="0 1 2" scripts/run_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
. scripts/env.sh
LOG=${LOG:-logs/matrix_$(date +%Y%m%d_%H%M%S).csv}
SEEDS=${SEEDS:-0}

run() {
    sitl_restart || return
    echo "== $*"
    $PY -m scripts.land_mission --log "$LOG" "$@" | grep -E "^  (result|error_m|on_tag|on_pad|tilt_ok|deck_tilt)"
}

for seed in $SEEDS; do
    run --scenario sim/scenarios/calm.json --seed "$seed" --no-vision
    run --scenario sim/scenarios/calm.json --seed "$seed"
    for s in moderate rough; do
        run --scenario "sim/scenarios/$s.json" --seed "$seed"
        run --scenario "sim/scenarios/$s.json" --seed "$seed" --stabilised
    done
done

echo
column -s, -t < "$LOG" | cut -c1-160
