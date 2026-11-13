# Project purpose, how to use this, and the path to the real drone

How to run things and what each file does: see the [README](../README.md).

---

## 1. The problem

Landing a drone on a boat is hard for three reasons:

1. **GPS isn't precise enough.** Normal GPS is off by 1–3 m. A landing pad is under a metre across.
2. **The target moves.** The deck rolls, pitches, heaves (moves up and down) and sways, and the
   ship may be moving.
3. **Touchdown on a tilted deck is dangerous.** A leg hitting a deck tilted 10° can tip the drone
   over, into the water.

Our system has two parts that address these:

- **Vision guidance (this repo):** a downward camera sees ArUco markers on the pad. The companion
  computer works out exactly where the pad is and sends that to the flight controller, which steers
  onto it. This fixes problems 1 and 2.
- **2-DOF stabilised platform (mechanical team):** the pad sits on a platform that cancels the
  deck's roll and pitch, so the pad stays level. This fixes problem 3.

## 2. Why simulate first

We don't have the drone yet. Even when we do, every failed real landing risks the hardware, and
over water it may lose it. Simulation lets us:

- **Build and debug the complete software chain** (camera → detection → MAVLink → ArduPilot →
  steering) with nothing at risk.
- **Catch the subtle bugs that crash real drones.** A flipped sign in the angle convention sends
  the drone the wrong way. Our tests checked this: a deliberately wrong sign gives a 94 cm error and
  the tests fail. We also caught that ArduPilot 4.7 silently ignores the old `LAND_SPEED` param name.
- **Try design options in minutes:** marker sizes, camera choice, landing speed, platform strength.
- **Produce measured evidence for the report and demo.**

## 3. What the project can use this for

| Use | How |
|---|---|
| **Show the vision system works** | Calm sea: 0.06 m landing error with vision vs 1.44 m with GPS alone (`./sim.sh matrix`) |
| **Show the platform's value** | Compare `--stabilised` vs not across sea states: touchdown tilt drops from 8.7° to 1.4° in rough sea |
| **Find the limits** | Which sea state breaks it, and why (`--trace`). Currently rough sea fails, which gives a concrete engineering problem to solve and document |
| **Make design decisions with data** | Pad size and layout (`config/pad.json`), camera resolution/FOV (`config/cam.json`), approach altitude, descent speed |
| **Tune ArduPilot before flying** | All precision-landing params in `config/plnd.parm` carry straight over to the real Pixhawk |
| **Team development** | Anyone can pull, run `./sim.sh land rough`, change something, and measure the effect |
| **Demo** | `./sim.sh land calm --show` with QGroundControl connected shows the camera view and the map at once |
| **Practice operations** | Fly missions and change params in QGroundControl against the simulator before touching hardware |

## 4. Simulation vs the real drone

| Piece | In simulation | On the real drone |
|---|---|---|
| Flight controller | ArduCopter 4.7.2 SITL in Docker | Pixhawk 6C running ArduCopter 4.7.2 |
| Precision-landing params | `config/plnd.parm`, loaded at boot | **Same file**, loaded once onto the Pixhawk |
| Camera | `sim/camera.py` renders images from the simulated pose | Real downward camera on the companion computer |
| Camera settings | `config/cam.json` (ideal values) | `config/cam.json` from **real calibration** |
| Pad | `config/pad.json` drawn by the simulator | **Same file**; the printed pad must match it |
| Detection + message | `vision/detector.py`, `vision/landing_target.py` | **Same code** |
| Program running vision | `scripts/land_mission.py` | `vision/aruco_node.py` |
| Link to flight controller | TCP 14551 through MAVProxy | Serial cable (Pixhawk TELEM2 ↔ companion UART) |
| Deck + platform | `sim/deck.py` with sine-wave motion and a gain | Real boat and the real 2-DOF platform |
| Mission (takeoff, approach) | Automated by `land_mission.py` | Pilot or a QGroundControl mission flies to the boat, then switches to LAND |
| Ground station | QGroundControl on TCP 14550 | QGroundControl over a telemetry radio |

**Moves over unchanged:** `vision/`, `config/plnd.parm`, `config/pad.json`.
**Gets replaced:** `config/cam.json` (calibration) and the image source.
**Simulation only:** `sim/`, `docker/`, `sim.sh`, `land_mission.py`.

## 5. Implementing it on the real drone

### 5.1 Hardware to add to the X500 v2

| Part | Purpose | Notes |
|---|---|---|
| Companion computer | Runs `aruco_node.py` | Raspberry Pi 5 or Jetson Orin Nano. A Pi 5 is enough for ArUco at 640×480, 15–30 Hz |
| Downward camera | Sees the pad | Global-shutter is strongly preferred: rolling shutter smears markers on a vibrating drone. Wide-ish lens (~70° FOV, like the sim) |
| Downward rangefinder | Height above the deck | **Needed**: `plnd.parm` uses it (`PLND_ALT_MIN/MAX`), and at sea the barometer measures height above the *sea*, not the moving deck. E.g. a small lidar (TFmini-S class) |
| Landing pad | The target | Printed to the exact sizes in `pad.json`, matte (no glare), rigid, flat |
| Telemetry radio | QGroundControl link | Standard SiK/Holybro radio |

### 5.2 Flight controller setup (Pixhawk 6C)

1. Flash **ArduCopter 4.7.2** with QGroundControl (Vehicle Setup → Firmware). This matches the
   simulator. If a different version is used, change `COPTER_TAG` in `docker/sitl/Dockerfile` so the
   sim matches.
2. Normal X500 setup: frame type, accelerometer/compass calibration, radio, flight modes, failsafes.
3. Enable the companion link on TELEM2:
   `SERIAL2_PROTOCOL 2` (MAVLink 2), `SERIAL2_BAUD 921` (921600 baud).
4. Load the precision-landing params from this repo (USB connected):
   ```bash
   mavproxy.py --master /dev/tty.usbmodem*     # then at the prompt:
   param load config/plnd.parm
   ```
5. Set the camera position relative to the drone's centre: `PLND_CAM_POS_X/Y/Z` (metres).
6. Configure the rangefinder (`RNGFND1_*` params, e.g. a TFmini-S on a serial port) and check its
   reading in QGroundControl. `plnd.parm` relies on it for `PLND_ALT_MIN`/`ALT_MAX`. If you fly
   without one, set both to 0, or ArduPilot will never retry a landing.

### 5.3 Companion computer setup

1. Wire Pixhawk **TELEM2** to the companion's UART: TX→RX, RX→TX, GND→GND. Don't power the
   Pixhawk from the companion.
2. Install the code:
   ```bash
   git clone git@github.com:Japleen-Kaur2409/drone-capstone.git && cd drone-capstone
   python3 -m venv .venv && .venv/bin/pip install opencv-contrib-python numpy pymavlink
   ```
3. Mount the camera pointing straight down with the **top of the image towards the drone's nose**.
   The code assumes this. Any other mounting needs a rotation added in `aruco_node.py`.
4. **Calibrate the camera** with a printed ChArUco board, and write the result to
   `config/cam.json` (same format). The calibration script is still to be written.
5. Run the vision node:
   ```bash
   .venv/bin/python -m vision.aruco_node --conn /dev/serial0 --baud 921600 --camera 0
   ```
   It prints each detection: marker IDs, distance, angles. Make it start at boot (systemd service)
   once it works.

### 5.4 Test progression (each step must pass before the next)

| Step | Setup | Pass when |
|---|---|---|
| 1. Desk test | Laptop camera, pad printed from `./sim.sh markers` at the labelled size: `./sim.sh camera` | Markers detected steadily; after calibration, distance matches a tape measure within a few cm; angles change sign correctly when you move the pad left/right/forward/back |
| 2. Bench, props off | Companion + camera + Pixhawk on the drone, connected to QGroundControl | `aruco_node` prints detections at ~15 Hz while connected to the Pixhawk; moving the pad under the drone changes the angles the right way. |
| 3. Hover over a static pad, on land | Low altitude, pilot ready on the RC in Loiter | QGC shows `PrecLand: Target Found`. Switching to LAND tracks onto the pad |
| 4. Precision landing on land | Approach from 1–2 m offset at 5–8 m | Repeated landings within ~10 cm, like the sim |
| 5. Moving platform on land | 2-DOF platform on a test rig that tilts/moves | Lands on the moving, stabilised pad |
| 6. On water | Calm conditions first, recovery boat ready | Lands on the boat |

Always keep a pilot with an RC transmitter able to take over (switch to Loiter/Stabilize), and a
geofence set.

### 5.5 Marine-specific things to plan for

- **Moving target:** ArduPilot's moving-target mode (`PLND_OPTIONS 1`) made rough-sea landings
  *worse* in simulation (its filter lags a fast-swaying deck). We use raw measurements instead. A
  deck-motion-aware controller on the companion computer is the next step beyond that (roadmap).
- **Height above deck:** use a rangefinder; the barometer can't see deck heave.
- **Getting to the boat:** GPS waypoint to the boat, or ArduPilot's Follow mode with a GPS on the
  boat, then switch to LAND once the camera sees the pad.
- **Glare and reflections:** matte pad, camera exposure settings. Test in sun.
- **Water and salt:** protect the electronics and connectors. Consider floats on the landing gear.
- **Failsafes over water:** battery and link-loss failsafes should return to a safe point, not
  land in the sea.
- **Regulations:** check drone rules for the flying location, especially over water and near boats.

## 6. Where the 2-DOF platform fits

The platform has its own controller (IMU + servos/actuators) and works independently of the drone.
In the simulator it's modelled simply: it cancels a fraction (`--stab-gain`, default 85%) of the
deck's roll and pitch. It can't remove heave or sideways motion.

Ways to connect it to this software later:

- **Better model:** put the real platform's measured response (lag, max angle, servo speed) into
  `sim/deck.py`, so the simulated results reflect the actual design.
- **Platform → drone data:** the platform could send its tilt and the deck motion over MAVLink, so
  the drone times its touchdown for a calm moment.
- **Gazebo:** a physics simulator with a real moving deck and platform model, so touchdown
  contact is simulated too (SITL lands on flat ground). ArduPilot's `ardupilot_gazebo` plugin is the
  usual route.

## 7. Roadmap

| # | Task | Status |
|---|---|---|
| 1 | SITL + MAVProxy in Docker, one-command runner | Done |
| 2 | Vision pipeline + ArduPilot precision landing, verified against ArduPilot's source | Done |
| 3 | Sea-state scenarios, platform model, scored landings | Done |
| 4 | Rough-sea landings: 3×3 marker pad, rangefinder, retry/estimator tuning (2–3 m → ~0.6 m) | Done |
| 4b | Acceptance test: `./sim.sh verify` (lands on the 15 cm tag) | Done |
| 4c | Deck-motion-aware guidance on the companion (predict sway, time touchdown) to get rough sea below ~0.3 m | Future |
| 5 | Camera calibration script (ChArUco → `cam.json`) | To do |
| 6 | Choose companion computer, camera, rangefinder | To do (team decision) |
| 7 | Desk and bench tests with real camera (5.4 steps 1–2) | When hardware arrives |
| 8 | Real platform model in `sim/deck.py`; Gazebo deck physics | Later |
| 9 | Flight tests (5.4 steps 3–6) | When drone is built |
