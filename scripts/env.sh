# Docker / Python environment for this repo. Source it: `. scripts/env.sh`
# Works with Colima (starts it if needed) or Docker Desktop. Docker is only touched
# when a simulator command calls docker_env.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"

docker_env() {
    if command -v colima >/dev/null 2>&1; then
        colima status >/dev/null 2>&1 || colima start --cpu 4 --memory 8 --disk 40
        export DOCKER_HOST="unix://$HOME/.colima/default/docker.sock"
        # project-local Docker config avoids a stale Docker Desktop credential helper
        mkdir -p "$REPO/.docker"
        [ -f "$REPO/.docker/config.json" ] || echo '{}' > "$REPO/.docker/config.json"
        export DOCKER_CONFIG="$REPO/.docker"
    fi
}

# Sourced by hand (`. scripts/env.sh`): set Docker up straight away
[ "${ENV_SH_LAZY:-0}" = 1 ] || docker_env

if command -v docker-compose >/dev/null 2>&1; then
    COMPOSE="docker-compose"
else
    COMPOSE="docker compose"
fi

if [ -n "${VIRTUAL_ENV:-}" ] || [ ! -x "$REPO/.venv/bin/python" ]; then
    PY="python"
else
    PY="$REPO/.venv/bin/python"
fi

# Wait until SITL logs EKF alignment after the given time (default: now)
sitl_wait_ready() {
    local since="${1:-$(date -u +%Y-%m-%dT%H:%M:%SZ)}"
    for _ in $(seq 1 60); do
        docker logs --since "$since" sitl 2>&1 | grep -q "EKF3 IMU0 tilt alignment complete" && return 0
        sleep 1
    done
    echo "SITL did not report ready; check: docker logs sitl" >&2
    return 1
}

# Fresh SITL (vehicle on the ground at home)
sitl_restart() {
    local since
    since="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    $COMPOSE restart sitl >/dev/null 2>&1
    sitl_wait_ready "$since"
}
