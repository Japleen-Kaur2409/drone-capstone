# Operations guide: commands for the simulator, QGroundControl and the code

A practical cheat sheet. Everything runs from the repo folder.
For what each file does, see the [README](../README.md). For the project and hardware plan, see
[PROJECT_AND_HARDWARE.md](PROJECT_AND_HARDWARE.md).

All console commands below were tested against our ArduCopter 4.7.2 simulator.

---

## 1. Start, stop, reset

| Command | What it does |
|---|---|
| `./sim.sh up` | Start the simulated drone (first time builds it, ~15 min). Returns when it's ready |
| `./sim.sh down` | Stop it |
| `./sim.sh land calm` | **Reset** the drone (fresh, on the ground, full battery, default params) and fly a scored landing |
| `./sim.sh battery` | Refill the battery to 100 % without resetting (see section 5) |
| `./sim.sh console` | Open the MAVProxy console to type commands (section 3) |
| `./sim.sh` | List all commands |

To reset without flying a landing:
```bash
. scripts/env.sh && docker-compose restart sitl
```

After a start or reset, give it **20–30 s** before arming: it needs a GPS fix and the position
estimate (EKF) to settle. QGroundControl shows "Ready To Fly" when it's armable.

---

## 2. QGroundControl

### Connect (once)
1. `./sim.sh up`
2. QGC → **Q** icon (top left) → **Application Settings** → **Comm Links** → **Add**
3. Name `SITL`, Type **TCP**, Server Address `127.0.0.1`, Port `14550` → **OK**
4. Select `SITL` → **Connect**

Next time, just open QGC, go to Comm Links and press Connect (or tick "Automatically Connect on
Start" on the link). After a reset (`./sim.sh land`), QGC usually reconnects by itself; if not,
press Connect again.

### Fly view
| To do | How |
|---|---|
| Take off | **Takeoff** button on the left toolbar → set altitude → slide to confirm |
| Fly to a point | Click on the map → **Go to location** → slide to confirm (drone must be flying in Guided) |
| Change altitude | **Action** → **Change Altitude** |
| Change flight mode | Click the mode name at the top (e.g. "Guided") → choose a mode |
| Land here | **Land** button |
| Come home | **RTL** button (Return To Launch) |
| Pause | **Pause** (holds position) |
| See warnings | Messages appear at the top; the bell/message icon shows the history |

### Plan view (missions)
1. Click **Plan** (top left).
2. Click the map to add waypoints. The first item should be **Takeoff**; add **Land** or **Return**
   at the end via the item list.
3. **Upload** (top right).
4. Back in Fly view: **Start Mission** → slide to confirm.

### Parameters
**Q** icon → **Vehicle Setup/Configuration** → **Parameters** → search, e.g. `PLND`, `LAND_SPD`,
`RTL_ALT`. Click a value to change it. **These are lost on the next reset.** To keep a change,
put it in `config/plnd.parm` (or another file in `config/`, see section 6).

### Useful tools
- **Analyze Tools → MAVLink Inspector**: live view of every message (`BATTERY_STATUS`,
  `GLOBAL_POSITION_INT`, ...).
- **Analyze Tools → Log Download**: flight logs from the drone (works on the real Pixhawk too).

---

## 3. MAVProxy console (terminal control)

Open with `./sim.sh console`, press **Enter** to get the prompt. QGC shows everything you do here.

**Leave with Ctrl-P then Ctrl-Q.** Pressing Ctrl-C kills MAVProxy and stops the simulator
(then `./sim.sh up` to restart).

### Fly
```
mode guided                      # Guided: the drone follows commands
arm throttle                     # arm the motors
takeoff 10                       # climb to 10 m (needs Guided + armed)
guided -35.36320 149.16530 10    # fly to lat, lon at 10 m
setspeed 3                       # horizontal speed 3 m/s
mode land                        # land where it is
mode rtl                         # return to launch and land
disarm                           # stop motors (on the ground)
```

### LOITER / ALT_HOLD in the simulator: centre the throttle first
These modes follow the RC sticks, and the simulator's virtual throttle stick sits at **minimum**,
so the drone sinks to the ground. Centre it first:
```
rc 3 1500                        # throttle stick to centre (channel 3)
mode loiter                      # now holds position and altitude
```
(Tested: without `rc 3 1500` the drone dropped from 5 m; with it, it held 5.0 m.)

### Status and checks
```
status                           # every latest message (long)
status GLOBAL_POSITION_INT       # one message, e.g. position/altitude
arm list                         # arming checks
```
If arming fails, the reason appears as `PreArm: ...`, usually "waiting for GPS/EKF" right after
a reset. Wait 20–30 s.

### Parameters
```
param show PLND_*                # wildcard search
param set LAND_SPD_MS 0.3        # change (lost on reset)
param load /config/plnd.parm     # reload our param file (the container sees config/ as /config)
```

### Missions
```
wp list                          # download the current mission
wp clear                         # delete it
mode auto                        # run the uploaded mission (plan it in QGC, upload, then this)
```

### Battery
```
long MAV_CMD_BATTERY_RESET 255 100    # refill to 100 % (same as ./sim.sh battery)
```

### Reboot the autopilot
```
reboot                           # on the ground, disarmed
```

---

## 4. Simulator-only experiments (fault injection)

Set these from the console or in QGC's Parameters. They are **all cleared by a reset**.

| Experiment | Commands | What to expect |
|---|---|---|
| Wind | `param set SIM_WIND_SPD 6`<br>`param set SIM_WIND_DIR 270`<br>`param set SIM_WIND_TURB 2` | 6 m/s wind from the west, gusty. Good test for precision landing |
| GPS failure | `param set SIM_GPS1_ENABLE 0` | GPS lost; watch the EKF/GPS failsafe messages and what the drone does (it will usually land). Back on: `param set SIM_GPS1_ENABLE 1` |
| Fewer satellites | `param set SIM_GPS1_NUMSATS 5` | Weaker GPS; may block arming |
| RC transmitter lost | `param set SIM_RC_FAIL 1` | RC failsafe → RTL (`FS_THR_ENABLE 1`). Recover: `param set SIM_RC_FAIL 0` |
| Weak motor | `param set SIM_ENGINE_MUL 0.5`<br>`param set SIM_ENGINE_FAIL 1` | Motor 1 at 50 % thrust (`SIM_ENGINE_FAIL` is a bitmask: 1 = motor 1, 2 = motor 2, 4 = motor 3...) |
| Realistic battery drain | `param set SIM_BATT_CAP_AH 3.3` | Voltage now drops as it flies (see section 5) |
| Battery failsafe | `param set BATT_FS_LOW_ACT 2`<br>`param set BATT_LOW_VOLT 11.0` | Low battery → RTL (1 = Land, 2 = RTL) |
| Geofence | `param set FENCE_ENABLE 1` | Flying beyond 150 m radius or 100 m altitude triggers RTL |
| Faster simulation | `SPEEDUP=3 ./sim.sh up` (from stopped) | Runs 3× faster than real time |
| Different start location | `HOME_LOC="43.6629,-79.3957,90,0" ./sim.sh up` (from stopped) | lat, lon, altitude, heading. E.g. near the water at your test site |

`./sim.sh down` first if the simulator is already running, so `SPEEDUP`/`HOME_LOC` take effect.

---

## 5. "Battery is dead" / battery warnings

### What's going on
In our simulator the battery **voltage never drops** (`SIM_BATT_CAP_AH 0` = constant 12.6 V). But
ArduPilot still **counts the charge used** against a 3300 mAh pack (`BATT_CAPACITY`). After a
few flights without a reset, the **percentage** reaches 0 and QGroundControl warns "battery
low/critical" or shows it as dead, even though nothing has really failed. By default our sim
has no battery failsafe (`BATT_FS_LOW_ACT 0`), so the drone keeps flying.

### Fixes (pick one)
| Fix | Command |
|---|---|
| **Refill without resetting** | `./sim.sh battery`, or in the console: `long MAV_CMD_BATTERY_RESET 255 100` |
| Set a specific level (to test low-battery behaviour) | `./sim.sh battery 20` |
| Full reset (also restores all params) | `./sim.sh land calm`, or `. scripts/env.sh && docker-compose restart sitl` |
| Make it last longer this session | Console: `param set BATT_CAPACITY 20000` (lost on reset) |

### If it won't arm because of the battery
`PreArm: Battery ...` means a minimum-to-arm limit is set. Refill (above) or remove the limit:
```
param set BATT_ARM_VOLT 0
param set BATT_ARM_MAH 0
```

### If a battery failsafe already switched it to RTL or Land
Refill (`./sim.sh battery`), then switch back to the mode you want (`mode guided`). If you
switched on realistic drain (`SIM_BATT_CAP_AH`), refill the simulated cells too:
`param set SIM_BATT_VOLTAGE 12.6`, or set `SIM_BATT_CAP_AH 0` for constant voltage.

### On the real drone
"Battery dead" there is real: land and swap or charge the pack. Set the real values
(`BATT_CAPACITY` = your pack's mAh, `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT 2` for RTL) during Pixhawk
setup. Over water, the low-battery action must bring it back before it's too late.

---

## 6. Running the code

### Landings and scoring
| Command | What it does |
|---|---|
| `./sim.sh land calm` | Scored landing (scenarios: `calm`, `moderate`, `rough`) |
| `./sim.sh land rough --stabilised` | With the 2-DOF platform |
| `./sim.sh land calm --show` | With a live camera window |
| `./sim.sh land calm --no-vision` | GPS-only baseline (no precision landing) |
| `./sim.sh land rough --trace logs/trace.csv` | Per-frame debug log |
| `./sim.sh land rough --seed 3` | Different approach direction (same seed = fair comparison) |
| `./sim.sh land calm --alt 10 --stab-gain 0.95` | Approach altitude, platform strength |
| `./sim.sh land sim/scenarios/mine.json` | Your own scenario file |
| `./sim.sh matrix` | All scenarios, with/without vision and platform |
| `SEEDS="0 1 2" ./sim.sh matrix` | Same, three runs each |

Results:
```bash
column -s, -t < logs/landings.csv
```

### Tests and checks
| Command | What it does |
|---|---|
| `./sim.sh test` | Offline vision tests (~2 s, no simulator) |
| `python scripts/smoke_test.py` | Arm, take off 10 m, land, with the sim running (no vision) |

### Camera and markers (no drone needed)
| Command | What it does |
|---|---|
| `./sim.sh markers` | Printable markers and pad images in `markers/`, labelled with print size |
| `./sim.sh camera` | Live ArUco detection on your laptop camera. Esc quits |
| `./sim.sh camera --camera 1` | Use a different camera |

### Param files and ArduPilot version
| Command | What it does |
|---|---|
| Edit `config/plnd.parm`, then any `./sim.sh land` | Permanent param changes, loaded on every reset |
| `PARAMS=plnd.parm,mine.parm ./sim.sh up` | Load extra param files from `config/` (from stopped) |
| `PARAMS=plnd.parm,plnd_sitl_beacon.parm ./sim.sh up` | ArduPilot's built-in fake beacon instead of our vision |
| Change `COPTER_TAG` in `docker/sitl/Dockerfile`, then `. scripts/env.sh && docker-compose build` | Different ArduPilot version (~15 min rebuild) |

### On the real drone (companion computer)
```bash
python -m vision.aruco_node --conn /dev/serial0 --baud 921600 --camera 0
```

### Getting the latest code
```bash
git pull
pip install -r requirements.txt
```

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| `PreArm: ...` / won't arm | Wait 20–30 s after a reset. Read the PreArm message; for battery see section 5 |
| Drone sinks in LOITER / ALT_HOLD | `rc 3 1500` first (section 3) |
| QGC won't connect | Is the sim running (`./sim.sh up`)? Link must be **TCP** 127.0.0.1:14550, not UDP |
| QGC lost connection | A reset drops it; press Connect again |
| Pressed Ctrl-C in the console | Simulator stopped: `./sim.sh up` |
| `./sim.sh land` or `./sim.sh battery` gets no data / hangs | Port 14551 serves **one program at a time**. Close the other script (QGC uses 14550, so it doesn't clash) |
| `Cannot connect to the Docker daemon` | `colima start` (sim.sh does this automatically) |
| `docker-credential-desktop not found` | Run docker by hand only after `. scripts/env.sh` |
| `No such file sim/scenarios/...json` | `git pull` (scenarios used to be YAML) |
| Param change didn't do anything | Check the name exists in 4.7: `param show NAME`. Unknown names are silently ignored (e.g. `LAND_SPEED` became `LAND_SPD_MS`) |
| Rough-sea landings miss by ~2 m | Known open issue; see README results |
