# How the simulation works, and how to test things with it

This is the detailed guide to our SITL (Software In The Loop) setup: what runs where, how a
simulated landing actually happens, how ArduPilot's precision landing behaves, and step-by-step
recipes for testing changes.

Testing: [TESTING.md](TESTING.md). Results: [TEST_RESULTS.md](TEST_RESULTS.md). Quick commands: [OPERATIONS.md](OPERATIONS.md). Every file: [README](../README.md).
Real drone plan: [PROJECT_AND_HARDWARE.md](PROJECT_AND_HARDWARE.md).

---

## 1. What SITL is

**SITL = the real ArduCopter flight code, compiled to run on a computer instead of the Pixhawk.**

On the Pixhawk 6C, ArduCopter reads real sensors (IMUs, barometer, compass, GPS, rangefinder) and
drives real motors. In SITL the *same* flight code runs, but:

- a **physics model** (`--model quad`) simulates the airframe: motor thrust, drag, gravity, wind;
- **simulated sensors** are generated from that physics state, with realistic noise and delays:
  IMU, barometer, compass, GPS (u-blox protocol), and our downward rangefinder;
- **serial ports** become TCP sockets.

So arming checks, flight modes, the EKF position estimate, failsafes, precision landing and every
parameter behave as on the real Pixhawk. What SITL **can't** tell you: real vibration, real camera
images, how a real X500 is tuned, wind gusts over a real deck, radio dropouts, and touchdown
contact with a moving surface.

Two things in our sim are *not* ArduPilot's: the **camera** and the **deck**. Those are our own
Python models (sections 4–5), because SITL has no camera and only flat ground.

---

## 2. What runs where

```
 your Mac                              Docker container "sitl" (Linux, arm64)
┌──────────────────────────────┐      ┌────────────────────────────────────────────────┐
│ ./sim.sh                     │      │ /entrypoint.sh                                  │
│  └─ scripts/land_mission.py ─┼─TCP──┼─► :14551 ┐                                      │
│      sim/deck.py             │14551 │          │  MAVProxy (PID 1)                    │
│      sim/camera.py           │      │          ├─ master tcp:127.0.0.1:5760 ───┐     │
│      vision/detector.py      │      │ :14550 ◄─┘                               │     │
│      vision/landing_target.py│      │    ▲                                     ▼     │
│                              │      │    │      arducopter (SITL)  SERIAL0 = tcp:5760│
│ QGroundControl ──────────────┼─TCP──┼────┘       • physics model "quad"            │
│                              │14550 │            • simulated IMU/baro/GPS/compass   │
└──────────────────────────────┘      │            • simulated rangefinder            │
                                      │            • params: copter.parm + /config/*  │
                                      │            • logs: /sitl/logs/*.BIN           │
                                      └────────────────────────────────────────────────┘
                                          ./config  is mounted read-only at /config
```

### Startup sequence (`docker/sitl/entrypoint.sh`)
1. `arducopter --model quad --home <HOME_LOC> --speedup <SPEEDUP> --defaults <param files> -w -I0`
   - `--defaults`: ArduPilot's standard `copter.parm`, then each file named in `PARAMS`
     (default `plnd.parm,sitl_rangefinder.parm`) from `/config`. Later files override earlier ones.
   - `-w` **wipes the stored parameters** on every start, so each start/reset is a clean drone with
     exactly the param files' values. This is why changes made in QGC or the console disappear on
     reset, and why `config/*.parm` is the place for permanent changes.
2. After 3 s, MAVProxy connects to SITL's SERIAL0 (TCP 5760) and opens two listening ports:
   **14550** (for QGroundControl) and **14551** (for our scripts). Each serves one client at a time.

`./sim.sh land` restarts the container before each flight, which repeats this sequence (drone back
on the ground at home, full battery, params from files), then waits for the log line
`EKF3 IMU0 tilt alignment complete`. The drone is armable once it also has GPS (~20–30 s).

### Parameters: where values come from
| Source | When applied | Survives a reset? |
|---|---|---|
| `copter.parm` (ArduPilot's SITL defaults, inside the image) | every start | yes |
| `config/plnd.parm`, `config/sitl_rangefinder.parm` | every start | yes (edit these for permanent changes) |
| `PARAMS=...` env var | when the container is created (`./sim.sh down` then `up`) | yes |
| QGC Parameters page / `param set` in the console | immediately | **no** |

ArduPilot silently ignores unknown parameter names in these files. 4.7 renamed several params
(e.g. `LAND_SPEED` cm/s → `LAND_SPD_MS` m/s), so always check with `param show NAME` after editing.

---

## 3. MAVLink: how the programs talk

Everything (QGC, MAVProxy, our scripts, ArduPilot) speaks **MAVLink**, a stream of small binary
messages. The ones that matter here:

| Message | Direction | Used for |
|---|---|---|
| `HEARTBEAT` | both | "I'm alive", flight mode, armed state |
| `ATTITUDE` | AP → us | roll/pitch/yaw, at 50 Hz (we request it with `SET_MESSAGE_INTERVAL`) |
| `LOCAL_POSITION_NED` | AP → us | position north/east/down from home, at 50 Hz |
| `EXTENDED_SYS_STATE` | AP → us | landed / in air (touchdown detection) |
| `STATUSTEXT` | AP → us | text messages, e.g. `PrecLand: Target Found` |
| `COMMAND_LONG` | us → AP | arm, takeoff, set message rates, battery reset |
| `SET_POSITION_TARGET_LOCAL_NED` | us → AP | fly to a point (GUIDED mode) |
| **`LANDING_TARGET`** | us → AP | **where the pad is, as seen by our camera** |

### LANDING_TARGET in detail
We send it ~15 times a second while the pad is visible:

| Field | Our value |
|---|---|
| `frame` | `MAV_FRAME_BODY_FRD`: angles are relative to the drone's body |
| `angle_x` | `atan2(x_cam, z_cam)`: how far right of the image centre the landing point is (rad) |
| `angle_y` | `atan2(y_cam, z_cam)`: how far down in the image (rad) |
| `distance` | straight-line distance camera → landing point (m) |
| `position_valid` | 0 (we use the angle form) |

ArduPilot rebuilds the direction as `normalize(-tan(angle_y), tan(angle_x), 1)` in body axes
(we read this in `AC_PrecLand_MAVLink.cpp`, 4.7.2), rotates it by the drone's attitude to get a
north/east/down vector, and multiplies by `distance`. That gives the pad position relative to the
drone, which precision landing steers to. The camera is assumed to point straight down with
**image-up = drone nose**. `tests/test_vision.py` replays this exact decoding to make sure a pad
we see ends up where it really is.

---

## 4. One simulated landing, step by step (`scripts/land_mission.py`)

The script runs **one loop** (~500 Hz) that reads all incoming MAVLink, then does two jobs:

**Job A: the camera, at 15 frames per *simulated* second.** Time comes from ArduPilot's
`time_boot_ms`, so everything stays in sync even with `SPEEDUP`.
1. `sim/deck.py` gives the pad's position and tilt at this instant (sea-state scenario + platform).
2. `sim/camera.py` renders the image the downward camera would see from the drone's current
   position and attitude.
3. `vision/detector.py` finds ArUco markers and solves the landing point (all visible markers in
   one PnP solve).
4. If found, `vision/landing_target.py` sends `LANDING_TARGET`.
5. With `--trace`, a row is written: drone vs pad position, which markers were seen, and how far
   off the estimate is from the truth (as ArduPilot would decode it).

**Job B: the mission state machine.**

| Phase | What happens | Ends when |
|---|---|---|
| takeoff | GUIDED, arm (retries until pre-arm checks pass), takeoff to `--alt` (8 m) | above 95 % of target altitude |
| approach | fly to the pad's nominal position **plus a deliberate error** (`approach_error_m` in a random direction from `--seed`), like arriving on GPS | within 0.4 m of that point and slower than 0.3 m/s |
| land | switch to **LAND**. From here ArduPilot's precision landing does all the steering, using our `LANDING_TARGET` messages | `EXTENDED_SYS_STATE` says on ground, or disarmed |

**Scoring at touchdown** (written to `logs/landings.csv`):

| Column | Meaning |
|---|---|
| `error_m` | horizontal distance drone centre → landing point, at touchdown |
| `on_tag` | 1 if the drone centre is inside the 15 cm landing tag (marker id 1) |
| `on_pad` | 1 if the drone centre is at least 0.25 m inside the pad edge (X500 gear ≈ 0.5 m across) |
| `tilt_ok` | 1 if the pad was tilted ≤ 5° at touchdown |
| `vz_touch` | descent speed just before touchdown (m/s) |
| `deck_tilt_deg` | pad tilt at touchdown |
| `first_det_alt_m` | altitude when the pad was first detected |
| `det_rate` | fraction of camera frames with a detection (whole flight, incl. takeoff) |
| `precland_msgs` | ArduPilot's precision-landing messages: Found / Lost / Retrying / Failsafe |

---

## 5. The camera and deck models

### Camera (`sim/camera.py`)
- A pinhole camera from `config/cam.json`: 640×480, focal length 457 px (~70° horizontal FOV).
- For **every pixel**, a ray is cast from the camera, intersected with the (tilted, moving) deck
  plane, and the pad texture is sampled there. So perspective, tilt, heave and partial views are
  exact, not approximated.
- The pad texture is drawn from `config/pad.json`, with a mip-map so it doesn't alias when far away.
- Gaussian noise (σ = 3 grey levels) is added. Options for harder conditions: `noise_std`, `blur_px`.
- Not modelled (yet): lens distortion, motion blur, sun glare, exposure changes, rolling shutter.

### Pad (`config/pad.json`)
A 1.5 m pad with a 3×3 grid of markers: a 0.50 m marker for acquisition from altitude (detectable to
~10 m), a 0.15 m **landing tag** at the landing point, and seven 0.22 m markers covering the rest.
The detector combines whatever is visible into one pose, so the landing point is known from **any**
marker. That keeps tracking alive in the last metre, where the camera sees only a small patch.

### Deck and platform (`sim/deck.py`, `sim/scenarios/*.json`)
- Motion = sums of sine waves per axis: heave (up/down), roll, pitch, surge (fore/aft), sway
  (side to side), plus optional steady drift. Heading rotates surge/sway into north/east.
- The **2-DOF platform** is modelled as cancelling a fraction `--stab-gain` (default 0.85) of the
  deck's roll and pitch. It doesn't remove heave or translation, as a real 2-DOF platform can't.
- **Limitation:** the simulated drone always touches down on flat ground at the home altitude.
  Heave and tilt only change what the camera sees and the score. Contact physics on a moving deck
  needs a Gazebo setup.

---

## 6. ArduPilot precision landing: how it behaves, and what we learned

When in **LAND** with `PLND_ENABLED 1`, ArduPilot:
1. Takes each `LANDING_TARGET`, turns it into a pad position relative to the drone (section 3).
2. Runs it through an estimator: `PLND_EST_TYPE 0` = raw (use the latest measurement), `1` =
   Kalman filter (smooths and estimates velocity).
3. Steers horizontally towards the pad while descending at `LAND_SPD_MS`.
4. Pauses the descent while further than `PLND_XY_DIST_MAX` from the pad.
5. If the target is lost: keeps descending for `PLND_TIMEOUT` seconds, then (with `PLND_STRICT 1`)
   **retries**: climbs back up and re-approaches, up to `PLND_RET_MAX` times, then lands anyway
   ("Failsafe Measures").
6. With a rangefinder: below `PLND_ALT_MIN` it commits to landing even if the target is lost; above
   `PLND_ALT_MAX` losing the target doesn't count.

### What we found tuning it for rough sea
All on the rough scenario, seeds 0/1/2, unless noted. Every step was diagnosed with `--trace`.

| Step | Change | Rough-sea error (seeds 0/1/2) | What the trace showed |
|---|---|---|---|
| Start | Kalman filter, `TIMEOUT 4`, `ALT_MIN 0.3`/`ALT_MAX 20`, 2-marker pad | 2.0–2.3 m (seed 0) | Vision accurate (≤ 4 cm); drone lags the swaying pad ±1.5–1.9 m; pad lost below ~1.2 m |
| Moving target | `PLND_OPTIONS 1`, `TIMEOUT 1` | 1.70 / 3.21 / 2.61 m | Still lagging; no retries happen |
| Raw estimator | `PLND_EST_TYPE 0` | 0.82 m (seed 1, vs 3.36 m) | Less lag. A faster Kalman (`ACC_P_NSE 10`) gave 2.10 m |
| **Bug found** | `ALT_MIN`/`ALT_MAX` set to 0 (no rangefinder) | 0.47 / 1.14 / 0.46 m | With them non-zero and no rangefinder, ArduPilot **never retries**. Now 4 retries per landing |
| 3×3 marker pad | `config/pad.json` grid | 0.97 / 0.26 / 0.12 m | Low-altitude visibility 41 → 142 of 243 poses; retries now wasted in the last 0.5 m |
| **Rangefinder** | `sitl_rangefinder.parm`, `ALT_MIN 0.5`, `ALT_MAX 10` | **0.62 / 0.63 / 0.62 m** | No retries, 41 s per landing. Tracking holds to ~0.7 m; the rest is lag behind the sway |
| Kalman again | `EST_TYPE 1` + `OPTIONS 1` with all fixes | 2.07 / 1.87 / 1.80 m | Overshoots the oscillating deck: raw stays |
| Gating | `XY_DIST_MAX 0.3` | 0.89 / 0.21 / 0.90 m | Not better on average, 4× slower: reverted |
| Faster final descent | `LAND_SPD_MS 0.7` | 0.86 / 0.89 / 1.49 m | Worse: reverted |

Still deck (`./sim.sh verify`): every landing on the 15 cm tag, 2.7–2.9 cm from centre.

**Take-aways for the real drone:** fit a rangefinder (or set `PLND_ALT_MIN`/`ALT_MAX` to 0), use a
multi-marker pad, prefer the raw estimator on a fast-moving deck, and keep `PLND_TIMEOUT` short.
The remaining rough-sea error is control lag behind the deck. Fixing it needs deck-motion-aware
guidance, beyond stock ArduPilot. Full details per experiment: [TEST_RESULTS.md](TEST_RESULTS.md).

---

## 7. How to test things

All testing recipes live in [TESTING.md](TESTING.md): offline tests, the desk camera test, smoke test,
scored landings, the land-on-tag acceptance test (`./sim.sh verify`), the matrix, comparing a change
fairly, reading traces and ArduPilot logs, fault injection, and new scenarios, pads and cameras.
What we've run so far and the results: [TEST_RESULTS.md](TEST_RESULTS.md).

---

## 8. What a good result looks like

| Check | Good |
|---|---|
| `./sim.sh test` | all pass |
| `./sim.sh verify` | VERIFY PASSED |
| calm, vision | `error_m` under ~0.1 m, `on_tag` mostly 1 |
| calm, no vision | ~1–2 m (shows the vision matters) |
| `est_err_m` in traces | a few cm |
| `precland_msgs` | `Target Found`, ideally no `Failsafe Measures` |
| stabilised vs not | `tilt_ok` = 1 with the platform in moderate/rough |

---

## 9. Limits of this simulation

- The deck motion is synthetic (sine waves), not measured ship data.
- Touchdown is always on flat ground; the deck's heave/tilt don't physically push the drone.
- The camera is ideal apart from noise: no distortion, glare, blur, or rolling shutter.
- The `quad` physics model is not tuned to a real X500 v2, so tracking performance on the real
  drone will differ. Re-tune `PLND_*` on the real aircraft, starting from these values.
- No real-world latency from a companion computer (we set `PLND_LAG 0.05`; measure the real one).
