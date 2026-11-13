# Drone Capstone — Marine Autonomous Landing (software)

Autonomous precision landing of a **Holybro X500 v2 + Pixhawk 6C (ArduPilot Copter 4.7.2)** onto a
**2-DOF stabilised landing platform** at sea, guided by **ArUco markers** and a downward camera.

The drone hardware isn't here yet, so everything runs in simulation. The vision and landing code is
written to move onto the real drone unchanged. Only the camera input and the connection change.

- **Why this exists, how the team can use it, and how it carries over to the real drone:**
  [docs/PROJECT_AND_HARDWARE.md](docs/PROJECT_AND_HARDWARE.md)
- **This file:** how to run it, what every file does, and what to change.

---

## 1. Quick start

```bash
./sim.sh land calm --show
```

That one command starts Docker (Colima) if needed, builds the simulator the first time (~15 min),
resets the simulated drone, and flies one scored landing:

1. Arms and climbs to 8 m.
2. Flies toward the deck, deliberately aiming 1.5 m off (like GPS error).
3. The simulated camera sees the pad. Our detector sends `LANDING_TARGET` to ArduPilot and the
   console shows `PrecLand: Target Found`.
4. Switches to LAND. ArduPilot steers onto the pad using our vision.
5. Prints the score (`error_m` = distance from pad centre) and adds a row to `logs/landings.csv`.

`--show` opens a window with what the drone's camera sees.

### All commands

| Command | What it does |
|---|---|
| `./sim.sh land calm` | Fresh simulator + one scored landing. Scenarios: `calm`, `moderate`, `rough`, or any `.json` path |
| `./sim.sh land rough --stabilised` | Same, with the 2-DOF platform switched on |
| `./sim.sh matrix` | Batch: every scenario, with/without vision, with/without platform. Prints a summary table |
| `./sim.sh test` | Offline tests (no simulator needed, ~2 s) |
| `./sim.sh up` | Start the simulator and leave it running |
| `./sim.sh console` | MAVProxy console for typing commands to the drone (leave with **Ctrl-P then Ctrl-Q**, not Ctrl-C) |
| `./sim.sh down` | Stop the simulator |
| `./sim.sh` | Show this list |

### Options for `./sim.sh land`

| Flag | Default | Meaning |
|---|---|---|
| `--stabilised` | off | Platform on: cancels most of the deck's roll/pitch |
| `--stab-gain 0.85` | 0.85 | Fraction of deck roll/pitch the platform cancels (0 = none, 1 = perfect) |
| `--alt 8` | 8 m | Approach altitude. The big marker is detectable to about 10 m |
| `--seed 0` | 0 | Picks the direction of the approach error. Use the same seed to compare changes fairly |
| `--no-vision` | off | GPS-only baseline: vision runs but nothing is sent to ArduPilot |
| `--show` | off | Live camera window |
| `--trace logs/trace.csv` | off | Per-frame log: drone vs pad position, detections, vision error |
| `--log logs/x.csv` | `logs/landings.csv` | Where the result row goes |
| `--rate 15` | 15 Hz | Camera frame rate |
| `--timeout 240` | 240 s | Give up after this long |

---

## 2. First-time setup on a new machine

```bash
git clone git@github.com:Japleen-Kaur2409/drone-capstone.git
cd drone-capstone
brew install colima docker docker-compose        # macOS. Or install Docker Desktop instead
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./sim.sh land calm                                # first run builds the simulator (~15 min)
```

If your own venv is already active, `sim.sh` uses it. Otherwise it uses `.venv`.

To run `docker` or `docker-compose` by hand on a Colima Mac, first run `. scripts/env.sh`
in that terminal.

---

## 3. How it fits together

```
┌──────────────── Docker container "sitl" ─────────────────┐
│  ArduCopter 4.7.2 SITL  ──tcp:5760──>  MAVProxy          │
│  (simulated Pixhawk +    params from     ├─ TCP 14550 ──┼──> QGroundControl
│   physics)               config/*.parm   └─ TCP 14551 ──┼──> our Python scripts
└──────────────────────────────────────────────────────────┘
                                                   │
         scripts/land_mission.py (on your Mac) <───┘
           ├─ reads drone position + attitude
           ├─ sim/deck.py        -> where the pad is and how it's tilted (sea state)
           ├─ sim/camera.py      -> renders the downward camera image
           ├─ vision/detector.py -> finds the ArUco markers, computes the landing point
           └─ vision/landing_target.py -> sends LANDING_TARGET to ArduPilot
                                          (ArduPilot's precision landing does the steering)
```

On the real drone, the `sim/` part goes away. A real camera feeds `vision/aruco_node.py`, which
uses the same detector and message code and talks to the Pixhawk over a serial cable. See
[docs/PROJECT_AND_HARDWARE.md](docs/PROJECT_AND_HARDWARE.md).

---

## 4. Every file, what it does, and what to change

### Top level

| File | What it does | When to change it |
|---|---|---|
| `sim.sh` | The one command for everything (section 1) | Adding a new shortcut command |
| `docker-compose.yml` | Defines the `sitl` container: ports 14550/14551, mounts `config/`, passes `HOME_LOC`, `SPEEDUP`, `PARAMS` | Change ports, start location, which param files load |
| `requirements.txt` | Python packages: pymavlink, MAVProxy, OpenCV (with ArUco), numpy, pytest | Adding a library |
| `pytest.ini` | Lets `pytest` find the code | Rarely |
| `.gitignore` | Keeps `.venv/`, `.docker/`, `logs/` and flight logs out of git | Adding new generated files |

### `docker/sitl/` — the simulated drone

| File | What it does | What to change |
|---|---|---|
| `Dockerfile` | Downloads ArduPilot at `COPTER_TAG` (Copter-4.7.2) and compiles SITL plus MAVProxy | `COPTER_TAG` to match the firmware on the real Pixhawk. **Needs a rebuild:** `. scripts/env.sh && docker-compose build` |
| `entrypoint.sh` | Starts the simulated copter with `copter.parm` + our `config/` param files, then MAVProxy with the two TCP outputs | Vehicle model (`MODEL`), extra MAVProxy options |

Environment variables (set before `./sim.sh up`, e.g. `SPEEDUP=2 ./sim.sh up`):

| Variable | Default | Meaning |
|---|---|---|
| `HOME_LOC` | `-35.363261,149.165230,584,353` | Start location: lat, lon, altitude (m), heading |
| `SPEEDUP` | 1 | Run the sim faster than real time |
| `PARAMS` | `plnd.parm` | Comma-separated param files from `config/` to load at boot |

### `config/` — settings shared by the simulator and, later, the real drone

| File | What it does | What to change |
|---|---|---|
| `plnd.parm` | ArduPilot precision-landing params: `PLND_ENABLED`, `PLND_TYPE 1` (MAVLink), Kalman filter, retry behaviour, descent speeds | **The main tuning file.** E.g. `PLND_OPTIONS 1` (moving target), `LAND_SPD_MS` (final descent speed), `PLND_XY_DIST_MAX` (how close before descending), `PLND_LAG` (camera latency). Loaded on every reset |
| `plnd_sitl_beacon.parm` | ArduPilot's built-in fake beacon instead of our vision. Sanity check that precision landing itself works | Use with `PARAMS=plnd.parm,plnd_sitl_beacon.parm ./sim.sh up`, then fly by hand from the console |
| `pad.json` | Landing pad layout: 0.50 m marker (seen from up to ~10 m) and 0.15 m marker at the landing point (for the last metre). ArUco dictionary `DICT_4X4_50` | Marker sizes, positions, add markers. **Must match the printed pad exactly** |
| `cam.json` | Camera intrinsics: 640×480, ~70° field of view, no distortion (sim values) | Replace with the real camera's calibration. Change resolution/FOV to test other cameras |

ArduPilot 4.7 renamed some params (e.g. `LAND_SPEED` in cm/s became `LAND_SPD_MS` in m/s).
Unknown names are **silently ignored**. After changing `plnd.parm`, check it took effect:
`./sim.sh console`, then `param show LAND_*`.

### `sim/` — simulation only (not used on the real drone)

| File | What it does | What to change |
|---|---|---|
| `camera.py` | Simulated downward camera. Traces every pixel onto the tilted, moving deck and draws the pad from `pad.json`, with noise | `noise_std`, `blur_px` for harder conditions; `SEA`/`PAD` brightness. Glare, rain and lighting effects would go here |
| `deck.py` | Ship deck motion from a scenario (sums of sine waves) plus the platform model (cancels `stab_gain` of roll/pitch) | A smarter platform model (lag, servo limits), random waves |
| `scenarios/calm.json` | Small motion, 1.5 m approach error | |
| `scenarios/moderate.json` | ±5° roll, ±0.15 m heave, ±0.4 m sway | |
| `scenarios/rough.json` | ±10° roll, ±0.35 m heave, ±0.8 m sway, 2.5 m approach error | |

Scenario format (copy one to make your own, then run `./sim.sh land myfile` or pass a path):

```json
{
  "name": "rough",
  "deck":   {"north": 6.0, "east": 4.0, "heading_deg": 60},
  "motion": {
    "heave": [[0.35, 6.5, 0]],          // each term: [amplitude, period_s, phase_deg]
    "roll":  [[10.0, 7.5, 0]],          // roll/pitch in degrees, others in metres
    "pitch": [[6.0, 9.0, 15]],
    "surge": [[0.4, 11.0, 0]],          // forward/back along the ship
    "sway":  [[0.8, 9.0, 90]],          // side to side
    "drift_ne": [0.0, 0.0]              // ship moving, m/s north/east
  },
  "approach_error_m": 2.5               // how far off the drone aims when arriving
}
```
(JSON doesn't allow comments. They're only shown here for explanation.)

**Simulator limitation:** the simulated copter always touches down on flat ground. Deck heave and
tilt change what the camera sees and the score, but not the contact physics. A Gazebo setup with a
real moving deck is the next step for that.

### `vision/` — the part that goes on the real drone

| File | What it does | What to change |
|---|---|---|
| `pad.py` | Loads `pad.json` and `cam.json`; gives each marker's corner positions on the pad | New pad formats |
| `detector.py` | Finds ArUco markers and solves one combined pose (PnP) from all visible markers, giving the landing point in camera coordinates. Ignores markers smaller than 12 px | Detection settings, filtering of bad detections, minimum marker size |
| `landing_target.py` | Converts the landing point to MAVLink `LANDING_TARGET` (angles + distance, `MAV_FRAME_BODY_FRD`) **exactly the way ArduPilot 4.7.2 decodes it** (checked against `AC_PrecLand_MAVLink.cpp`) | Only if ArduPilot changes its decoding, or to switch to the x/y/z (`position_valid=1`) form |
| `aruco_node.py` | **The real-drone program.** Camera → detector → `LANDING_TARGET` at 15 Hz. Works with a webcam on the bench too | Camera source, rate, mounting rotation |

### `scripts/`

| File | What it does |
|---|---|
| `land_mission.py` | The scored closed-loop landing (what `./sim.sh land` runs). See flags in section 1 |
| `run_matrix.sh` | Batch of landings (what `./sim.sh matrix` runs). `SEEDS="0 1 2"` for more runs per case |
| `smoke_test.py` | Basic check without vision: arm, take off 10 m, land. `python scripts/smoke_test.py` while the sim is up |
| `env.sh` | Shared setup: starts Colima, sets Docker variables, picks Python, reset/wait helpers |

### `tests/`

| File | What it checks |
|---|---|
| `test_vision.py` | Renders known poses → detects → encodes → **decodes the way ArduPilot does** → checks the drone would steer to the right spot (within a few cm). Also: small marker takes over low down, big marker works high up, message packing, and that the platform reduces tilt. Run `./sim.sh test` after every vision change |

### `logs/` (not in git)

| File | Contents |
|---|---|
| `landings.csv` | One row per landing: scenario, platform on/off, vision on/off, `error_m`, `vz_touch` (touchdown speed), `deck_tilt_deg`, first detection altitude, detection rate, duration, precision-landing messages |
| `matrix_*.csv` | Results of each `./sim.sh matrix` |
| `trace*.csv` | Per-frame traces from `--trace` |

---

## 5. Connecting QGroundControl

1. `./sim.sh up`
2. QGroundControl → **Q** icon → Application Settings → Comm Links → **Add**
3. Type **TCP**, Server `127.0.0.1`, Port `14550` → OK → **Connect**

The drone appears on the map. `./sim.sh land` resets the simulator, which drops QGC's connection.
It usually reconnects by itself; otherwise click Connect again.

## 6. Controlling the drone from the terminal

`./sim.sh console`, press Enter, then for example:

```
mode guided
arm throttle
takeoff 10
guided -35.36320 149.16530 10
mode land
param show PLND_*
param set LAND_SPD_MS 0.3
```

Leave with **Ctrl-P then Ctrl-Q**. Ctrl-C stops the simulator. Changes made here are lost on the
next reset; put permanent ones in `config/plnd.parm`.

---

## 7. Making changes: the workflow

1. Change one thing: a param in `config/plnd.parm`, a scenario, the pad, or code.
2. `./sim.sh test` if you touched code.
3. Run the **same** scenario and seed before and after: `./sim.sh land rough --seed 0`.
4. Compare `error_m` in `logs/landings.csv`, or run `./sim.sh matrix` for everything.
5. If a landing looks wrong, add `--trace logs/trace.csv` and look at it frame by frame.
6. Commit and push when it helps.

## 8. Current results (Copter 4.7.2, seed 0)

| Scenario | Landing error |
|---|---|
| Calm, GPS only (no vision) | 1.44 m |
| Calm, vision | **0.06 m** |
| Moderate, platform off / on | 0.28 / 0.31 m |
| Rough, platform off / on | **2.04 / 2.32 m**: not solved yet |

The platform cuts touchdown tilt (rough: 8.7° → 1.4°) but doesn't reduce position error, because
in this model it only cancels tilt.

## 9. What you can do with this

- **Tune precision landing** before risking hardware: descent speed, retry behaviour, moving-target mode.
- **Design the pad**: marker sizes and layout vs. how high the drone first sees it and how
  accurately it lands.
- **Choose a camera**: change `cam.json` resolution/FOV and see the effect on detection range.
- **Quantify the platform's value** across sea states, for the report.
- **Stress test**: more noise or blur, a drifting ship, bigger waves, larger approach error.
- **Ground-station practice**: plan and fly missions in QGroundControl against the simulator.
- **Bench-test the real vision code** with a webcam and a printed marker (`python -m vision.aruco_node --no-mav --show`).
- **Regression check**: run `./sim.sh matrix` after any change to make sure nothing got worse.

Open work: fix rough-sea landings, a camera calibration script, a Gazebo setup with real deck
physics, and the platform controller interface. See
[docs/PROJECT_AND_HARDWARE.md](docs/PROJECT_AND_HARDWARE.md).
