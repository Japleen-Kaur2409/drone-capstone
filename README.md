# Drone Capstone — Marine Autonomous Landing (software / SITL)

Holybro X500 v2 + Pixhawk 6C, **ArduPilot Copter**. ArduCopter SITL + MAVProxy run in Docker
(Colima on Apple Silicon); vision / guidance scripts run on the Mac.

## One-time setup (macOS, Apple Silicon)

```bash
brew install colima docker docker-compose
colima start --cpu 4 --memory 8 --disk 40
# ~/.docker/config.json still points at Docker Desktop's credential helper, so use a
# project-local config + Colima's socket (run in every new shell, or add to ~/.zshrc):
export DOCKER_CONFIG=$PWD/.docker DOCKER_HOST=unix://$HOME/.colima/default/docker.sock
mkdir -p .docker && [ -f .docker/config.json ] || echo '{}' > .docker/config.json
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
docker-compose build          # ~10-20 min, compiles ArduCopter for arm64
```

## Run

```bash
docker-compose up -d sitl     # start SITL + MAVProxy
docker attach sitl            # MAVProxy console (detach: Ctrl-P Ctrl-Q)
.venv/bin/python scripts/smoke_test.py   # arm, takeoff 10 m, land
docker-compose down
```

| Port | Use |
|------|-----|
| TCP 127.0.0.1:14550 | GCS (QGroundControl / Mission Planner, TCP link) |
| TCP 127.0.0.1:14551 | Python scripts (`tcp:127.0.0.1:14551`) |

Options: `HOME_LOC="lat,lon,alt,hdg" SPEEDUP=2 docker-compose up -d sitl`.
Change ArduCopter version: `docker-compose build --build-arg COPTER_TAG=Copter-4.x.y`.

After a Mac reboot: `colima start`.
