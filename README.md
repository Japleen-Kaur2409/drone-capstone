# Drone Capstone — Marine Autonomous Landing (software / SITL)

Holybro X500 v2 + Pixhawk 6C, **ArduPilot Copter**. ArduCopter SITL + MAVProxy run in Docker
(Colima on Apple Silicon); vision / guidance scripts run on the Mac.

## Quick start

```bash
./sim.sh land calm --show       # starts everything, flies one scored landing
```

`sim.sh` starts Colima if needed, sets up Docker, builds the SITL image the first time (~15 min),
and resets the simulator before every landing. Run it from any shell; venv optional.

| Command | Does |
|---------|------|
| `./sim.sh land [calm\|moderate\|rough] [--stabilised] [--show]` | Fresh SITL + one scored landing |
| `./sim.sh matrix` | All scenarios, with/without vision and platform |
| `./sim.sh test` | Offline tests |
| `./sim.sh up` / `./sim.sh down` | Start / stop SITL |
| `./sim.sh console` | MAVProxy console (detach: Ctrl-P Ctrl-Q) |

## One-time setup (new machine)

```bash
brew install colima docker docker-compose       # macOS; or install Docker Desktop
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

Using plain `docker`/`docker-compose` by hand on a Colima Mac: first run `. scripts/env.sh`.

| Port | Use |
|------|-----|
| TCP 127.0.0.1:14550 | GCS (QGroundControl / Mission Planner, TCP link) |
| TCP 127.0.0.1:14551 | Python scripts (`tcp:127.0.0.1:14551`) |

SITL options: `HOME_LOC="lat,lon,alt,hdg" SPEEDUP=2 docker-compose up -d sitl`.
ArduCopter version defaults to the current stable (`Copter-4.7.2`), matching what goes on the
Pixhawk 6C. Change it with `docker-compose build --build-arg COPTER_TAG=Copter-4.x.y`.

## Precision landing (ArUco, closed loop in SITL)

Each `./sim.sh land` run appends a scored row (touchdown error vs pad centre, descent speed, deck
tilt, detection rate) to `logs/landings.csv`. Add `--trace logs/trace.csv` for a per-frame log.

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
