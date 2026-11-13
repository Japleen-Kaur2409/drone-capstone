# How to test

Every kind of test in this repo: what it checks, how to run it, and how to read the result.
What we've already run and found: [TEST_RESULTS.md](TEST_RESULTS.md).
How the simulator works underneath: [SITL_GUIDE.md](SITL_GUIDE.md).

| Test | Command | Needs simulator | Time |
|---|---|---|---|
| Offline vision tests | `./sim.sh test` | no | ~2 s |
| Desk camera test | `./sim.sh camera` | no (laptop camera) | live |
| Smoke test | `python scripts/smoke_test.py` | yes | ~1 min |
| One scored landing | `./sim.sh land <scenario>` | yes | ~1 min |
| **Land-on-tag acceptance** | `./sim.sh verify` | yes | ~3 min |
| Batch matrix | `./sim.sh matrix` | yes | ~7 min per seed |
| Fault injection | param files + `./sim.sh land` | yes | ~1 min each |

**Before committing a change:** `./sim.sh test && ./sim.sh verify && ./sim.sh matrix`

---

## 1. Offline vision tests (`./sim.sh test`)

`tests/test_vision.py`, run with pytest. No simulator, no Docker. Extra pytest options pass through,
e.g. `./sim.sh test -k sign -v`.

| Test | Checks |
|---|---|
| `test_round_trip_matches_ardupilot_decoding` (4 poses) | Render the pad from a known pose → detect → encode `LANDING_TARGET` → **decode exactly as ArduPilot 4.7.2 does** → the drone would steer to the true pad position within a few cm. Includes tilted drone, tilted deck, 8 m altitude, 1 m altitude |
| `test_small_marker_takes_over_low` | At 0.4 m straight above, only the 15 cm landing tag is visible and the distance is right |
| `test_pad_stays_visible_low_and_off_centre` | At 0.7–0.9 m altitude and up to ~0.7 m off-centre, some marker is still visible and the landing point is still within 3 cm. (Fails with the old 2-marker pad: this guards the pad design) |
| `test_big_marker_acquires_high` | The 0.5 m marker is detected from 9 m |
| `test_sign_convention_target_ahead_and_right` | Pad ahead/right of the drone decodes as ahead/right. The classic bug that makes drones fly away from the target |
| `test_landing_target_packs_and_decodes` | The MAVLink message bytes carry our angles, distance and frame correctly |
| `test_platform_reduces_tilt` (3 scenarios) | With the platform on, max pad tilt is < 20 % of the deck's |

Run these after **any** change to `vision/`, `sim/camera.py`, `config/pad.json` or `config/cam.json`.
If you change the camera, some thresholds (altitudes, error limits) may need adjusting.

---

## 2. Desk camera test (`./sim.sh camera`)

Tests the real vision code on a real camera, without a drone.
1. `./sim.sh markers` writes printable images to `markers/` (labelled with print size).
2. Show `markers/marker1_15cm.png` on a phone, or print it at the labelled size.
3. `./sim.sh camera` (`--camera 1` for another camera). Esc quits.

Each detection prints `ids`, `dist` (m), `ax`/`ay` (rad) and `err` (reprojection error, px).

| Check | Pass |
|---|---|
| Detection | Green outline, steady `ids` while the marker is in view |
| Signs | Move the marker right → `ax` increases; move it down in the image → `ay` increases |
| Distance | Only once the print matches `pad.json` sizes **and** the camera is calibrated: matches a tape measure within a few cm |
| `err` | Under ~1 px |

---

## 3. Smoke test (`scripts/smoke_test.py`)

Checks the simulator and MAVLink link work at all, no vision involved.
```bash
./sim.sh up
python scripts/smoke_test.py          # arm in GUIDED, take off to 10 m, hover 3 s, land
```
Pass: ends with `Landed and disarmed. SITL OK.` Use it after rebuilding the image or changing
`docker/`.

---

## 4. One scored landing (`./sim.sh land`)

```bash
./sim.sh land calm                     # scenarios: calm, moderate, rough, ontag, or a .json path
./sim.sh land rough --stabilised --seed 2 --show
```
Resets the simulator, takes off to 8 m, flies to the pad with a deliberate GPS-like error, switches
to LAND, and ArduPilot's precision landing steers using our camera. Prints the result and appends it
to `logs/landings.csv` (or `--log FILE`).

| Result column | Meaning | Good |
|---|---|---|
| `error_m` | drone centre → landing point at touchdown | calm < 0.1 m |
| `on_tag` | inside the 15 cm landing tag | 1 in calm/still |
| `on_pad` | ≥ 0.25 m inside the pad edge (X500 gear ≈ 0.5 m) | 1 |
| `tilt_ok` | pad tilt ≤ 5° at touchdown | 1 with the platform |
| `vz_touch` | descent speed at touchdown (m/s) | ≈ `LAND_SPD_MS` |
| `first_det_alt_m` | where the pad was first seen | ≈ approach altitude |
| `precland_msgs` | ArduPilot messages | `Target Found`, no `Failsafe Measures` |

Useful flags: `--no-vision` (GPS-only baseline), `--stabilised`, `--stab-gain`, `--alt`, `--seed`,
`--trace FILE`, `--show`. Full list: README section 1.

---

## 5. Land-on-tag acceptance test (`./sim.sh verify`)

The test that says "the system works":
```bash
./sim.sh verify              # seeds 0 1 2
./sim.sh verify 0 1 2 3 4    # more seeds
```
- Scenario `sim/scenarios/ontag.json`: still deck (harbour or ground test), drone arrives 1.5 m off
  in a different direction for each seed.
- **Pass:** the drone's centre touches down inside the 15 cm landing tag (marker id 1) in **every**
  run. The tag is checked in the pad's own frame, so it's a real square, not a radius.
- Output: one `PASS`/`FAIL` line per seed, then `VERIFY PASSED` or `VERIFY FAILED` (exit code 1, so
  it can gate a script). Rows go to `logs/verify.csv`.
- It really fails when it should: with vision off it reports
  `FAIL: touchdown 1.431 m from the landing point`.

To check a single landing against the tag: `./sim.sh land <scenario> --require-on-tag`.

---

## 6. Batch matrix (`./sim.sh matrix`)

```bash
./sim.sh matrix                            # one seed
SEEDS="0 1 2" ./sim.sh matrix              # three seeds (recommended)
LOG=logs/my_matrix.csv SEEDS="0 1 2" ./sim.sh matrix
```
Per seed: calm without vision (baseline), calm with vision, moderate ±platform, rough ±platform (6
landings, fresh simulator each). Prints `result`, `error_m`, `on_tag`, `on_pad`, `tilt_ok` per run,
then the whole CSV as a table. Default log: `logs/matrix_<date>.csv`.

---

## 7. Comparing a change fairly

Rough-sea results vary a lot run to run, so a single landing proves nothing.

1. **Fix everything except your change**: same scenario, same `--seed` (the seed fixes the approach
   direction; deck motion is deterministic), same pad, camera and params.
2. Run **before** and **after** with at least 3 seeds, into separate logs:
   ```bash
   for s in 0 1 2; do ./sim.sh land rough --seed $s --log logs/before.csv; done
   # make the change
   for s in 0 1 2; do ./sim.sh land rough --seed $s --log logs/after.csv; done
   column -s, -t < logs/before.csv; column -s, -t < logs/after.csv
   ```
3. **Check the config you think you're testing is live**: `./sim.sh console`, then
   `param show PLND_*`. Unknown names in `.parm` files are silently ignored, and `PARAMS` changes
   need `./sim.sh down` first.
4. Compare the **mean and the worst** `error_m`, plus `on_pad`/`on_tag` counts.
5. Write it up in [TEST_RESULTS.md](TEST_RESULTS.md): config, seeds, numbers, conclusion.

---

## 8. Debugging with `--trace`

```bash
./sim.sh land rough --seed 1 --trace logs/trace.csv
column -s, -t < logs/trace.csv | less
```
One row per camera frame (15 Hz):

| Column | Meaning |
|---|---|
| `t` | seconds since takeoff |
| `phase` | takeoff / approach / land |
| `alt` | altitude (m) |
| `rel_n`, `rel_e` | drone minus pad position, north/east (m). Should shrink towards 0 during `land` |
| `det` | 1 if the pad was detected |
| `ids` | which markers were seen |
| `est_err_m` | error of our pad estimate after ArduPilot-style decoding. Should be a few cm |

**How to read it:**
- `est_err_m` large → **vision** problem (detector, pad, camera model).
- `est_err_m` small but `rel_n`/`rel_e` large or oscillating → **control** problem (ArduPilot
  precision-landing settings, lag).
- `det` goes to 0 near the ground → the pad **left the camera view** (pad design, camera FOV).

Low-altitude part only:
```bash
awk -F, 'NR==1 || ($2=="land" && $3<3)' logs/trace.csv | column -s, -t
```
Plot (after `pip install matplotlib`):
```bash
python - <<'EOF'
import csv, matplotlib.pyplot as plt
r = [x for x in csv.DictReader(open("logs/trace.csv")) if x["phase"] == "land"]
t = [float(x["t"]) for x in r]
for k in ("alt", "rel_n", "rel_e"):
    plt.plot(t, [float(x[k]) for x in r], label=k)
plt.legend(); plt.xlabel("s"); plt.show()
EOF
```

### ArduPilot's own log
```bash
. scripts/env.sh
docker exec sitl ls /sitl/logs                     # newest = highest number
docker cp sitl:/sitl/logs/00000010.BIN logs/
mavlogdump.py --types PL logs/00000010.BIN | less  # precision landing
```
`PL`: `TAcq` target acquired, `pX`/`pY` target relative position (m), `vX`/`vY` target velocity,
`mX`/`mY`/`mZ` raw measurement. Also useful: `CTUN`, `POS`, `MODE`, `MSG`. Logs are lost on
`./sim.sh down`. The same tools read logs from the real Pixhawk.

---

## 9. Fault injection

Quick, interactive (lost on reset): see [OPERATIONS.md](OPERATIONS.md) section 4 (wind, GPS loss,
RC loss, weak motor, battery).

For **scored** landings under a fault, put the `SIM_*` params in a file, e.g. `config/wind.parm`:
```
SIM_WIND_SPD 6
SIM_WIND_DIR 270
SIM_WIND_TURB 2
```
```bash
./sim.sh down && PARAMS=plnd.parm,sitl_rangefinder.parm,wind.parm ./sim.sh up
for s in 0 1 2; do ./sim.sh land calm --seed $s --log logs/wind.csv; done
./sim.sh down     # back to defaults on the next start
```
`PARAMS` applies only when the container is created, hence `down` first. Keep
`sitl_rangefinder.parm` in the list.

Other ideas: `SIM_GPS1_NUMSATS 6` (weak GPS), `SIM_ENGINE_MUL 0.8` + `SIM_ENGINE_FAIL 1` (weak motor),
camera noise (`noise_std` in `sim/camera.py`), a new sea state (copy a scenario file).

---

## 10. Other test variations

| Want to test | How |
|---|---|
| A new sea state | Copy `sim/scenarios/moderate.json` → `mine.json`, edit, `./sim.sh land mine` |
| Pad layout | Edit `config/pad.json` → `./sim.sh test` → `./sim.sh markers` (look at it) → `./sim.sh verify` |
| A different camera | Edit `config/cam.json`: `f = (width/2) / tan(HFOV/2)`. Wider = sees the pad lower down, but from less high up |
| Faster runs | `./sim.sh down && SPEEDUP=3 ./sim.sh up`. Watch `det_rate` doesn't drop |
| ArduPilot version | `COPTER_TAG` in `docker/sitl/Dockerfile`, rebuild, then smoke test + verify + matrix |
| Precision landing alone (no vision) | `./sim.sh down && PARAMS=plnd.parm,plnd_sitl_beacon.parm ./sim.sh up`, then fly GUIDED → LAND from the console near the beacon |
