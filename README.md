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
ArduCopter version defaults to the current stable (`Copter-4.7.2`), matching what goes on the
Pixhawk 6C. Change it with `docker-compose build --build-arg COPTER_TAG=Copter-4.x.y`.

## Precision landing (ArUco, closed loop in SITL)

```bash
.venv/bin/python -m pytest                          # offline: render -> detect -> LANDING_TARGET -> ArduPilot decode
docker-compose up -d sitl
.venv/bin/python -m scripts.land_mission --scenario sim/scenarios/calm.json --show
.venv/bin/python -m scripts.land_mission --scenario sim/scenarios/rough.json --stabilised
```

Each run appends a scored row (touchdown error vs pad centre, descent speed, deck tilt, detection
rate) to `logs/landings.csv`. Restart SITL between runs (`docker-compose restart sitl`) so each one
starts on the ground at home.

| Path | What |
|------|------|
| `config/plnd.parm` | ArduPilot precision-landing params, loaded at SITL boot (`PARAMS=` env) |
| `config/plnd_sitl_beacon.parm` | Sanity check with ArduPilot's built-in sim beacon instead of vision |
| `config/pad.json` | Pad layout: 0.50 m marker for acquisition (to ~10 m), 0.15 m marker at the landing point |
| `config/cam.json` | Camera intrinsics (sim values now; replace with real calibration) |
| `sim/camera.py` | Simulated downward camera (ray-cast onto the moving deck) |
| `sim/deck.py`, `sim/scenarios/` | Sea-state deck motion + 2-DOF platform model |
| `vision/detector.py` | ArUco multi-marker PnP; same code for sim and the real drone |
| `vision/landing_target.py` | LANDING_TARGET encoding matched to ArduPilot's decoder |
| `vision/aruco_node.py` | Real-camera node for the companion computer |
| `scripts/land_mission.py` | Scored closed-loop landing |

SITL limitation: the simulated copter always touches down on flat ground, so deck heave/tilt affect
what the camera sees and the score, not contact physics. Gazebo (ardupilot_gazebo) is the next step
for that.

After a Mac reboot: `colima start`.
