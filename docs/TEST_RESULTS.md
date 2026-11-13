# Test results log

What we tested, how, and what we found, in the order it happened. How to run these yourself:
[TESTING.md](TESTING.md).

**Common setup unless noted:** ArduCopter 4.7.2 SITL (`--model quad`), camera 640×480 ~70° HFOV
(`config/cam.json`), approach at 8 m, `LAND_SPD_MS 0.4`. `error_m` = horizontal distance from the
drone's centre to the landing point at touchdown. Seeds change the approach direction; deck motion
is deterministic. Dates: 12–13 Nov 2026.

## Summary

| | Calm (vision) | Calm (GPS only) | Moderate | Rough | Still deck, on tag? |
|---|---|---|---|---|---|
| Start (T1) | 0.06 m | 1.44 m | ~0.3 m | **2.0–3.4 m** | not measured |
| Now (T13, 3 seeds) | 0.07 m, 3/3 on tag | 1.45 m | 0.36 m, 3/3 on pad | **0.61 m**, 2/3 on pad | **3/3 on tag, 2.7–2.9 cm** |

Rough sea improved about 3–4× through four fixes found by tracing (T6–T8). What's left is lag: stock
ArduPilot precision landing can't keep up with a pad swaying ±1 m every few seconds. The
next step is deck-motion-aware guidance (see the conclusion at the end).

---

## T1. Baseline matrix

**Config:** Kalman estimator (`PLND_EST_TYPE 1`), `PLND_TIMEOUT 4`, `PLND_ALT_MIN 0.3` /
`PLND_ALT_MAX 20`, no rangefinder, 2-marker pad (0.50 m + 0.15 m). Seed 0. `./sim.sh matrix`.

| Scenario | error_m |
|---|---|
| Calm, GPS only (`--no-vision`) | 1.435 |
| Calm, vision | 0.056 (0.045 on an earlier single run) |
| Moderate / moderate + platform | 0.283 / 0.310 |
| Rough / rough + platform | **2.043 / 2.316** |

**Conclusion:** vision works on a calm deck (25× better than GPS). Rough sea fails. The platform cut
touchdown tilt 8.7° → 1.4° but didn't change position error, which is expected because it only
cancels tilt.

## T2. Rough sea, traced (diagnosis)

**Config:** as T1, `./sim.sh land rough --trace`. Seed 0: error 2.21 m.

**Findings:**
- Vision is **accurate**: `est_err_m` ≤ 4 cm whenever the pad is detected.
- The drone-to-pad offset swings ±1.5–1.9 m during the whole descent: the drone **lags the swaying
  deck**.
- Below ~1.2 m the pad **leaves the camera view** and isn't found again; the drone lands blind.

## T3. Moving-target mode + short timeout

**Config:** T1 + `PLND_OPTIONS 1` (estimate/follow target velocity), `PLND_TIMEOUT 1`. Rough, seeds 0/1/2.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | 1.695 | 3.213 | 2.610 |

**Conclusion:** no better. Trace of seed 1 (error 3.36 m): mean offset above 1.5 m altitude 1.18 m,
max 2.16 m. Still no retries happening.

## T4. Raw estimator vs faster Kalman filter (seed 1)

| Config | error_m | Mean / max offset above 1.5 m |
|---|---|---|
| T3 (Kalman + moving target) | 3.361 | 1.18 / 2.16 m |
| **A: raw** (`PLND_EST_TYPE 0`, `PLND_OPTIONS 0`) | **0.819** | **0.65 / 1.77 m** |
| B: Kalman + moving target + `PLND_ACC_P_NSE 10` | 2.103 | 0.83 / 1.97 m |

**Conclusion:** the Kalman filter lags and overshoots a fast-moving deck; raw measurements track
better. Trace A: still loses the pad at ~1.15 m when 0.6–0.8 m off-centre.

## T5. Why no retries? A config bug

Reading `AC_PrecLand.cpp` (4.7.2): if `PLND_ALT_MIN` or `PLND_ALT_MAX` is non-zero, ArduPilot uses
the **rangefinder** to decide whether a lost target "should" be visible. With no rangefinder it
always decides "out of range", which **disables retries completely**. Our `0.3`/`20` had been
silently turning retries off since the start.

## T6. Retries enabled (bug fixed)

**Config:** raw estimator, `PLND_TIMEOUT 1`, `PLND_ALT_MIN 0` / `PLND_ALT_MAX 0`. Rough, seeds 0/1/2.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | **0.465** | **1.137** | **0.460** |
| Retries | 4 | 4 | 4 |

**Conclusion:** big improvement. But every attempt still loses the pad below ~1.3 m; after 4
retries ArduPilot gives up ("Failsafe Measures") and lands.

## T7. 3×3 marker pad

**Change:** `config/pad.json` from 2 markers to a 3×3 grid (0.50 m acquisition marker, 0.15 m
landing tag, seven 0.22 m markers), all used in one PnP solve.

**Offline coverage** (camera at 0.6/0.9/1.2 m, drone up to 1 m off-centre, tilted deck; 243 poses):

| Pad | Poses where the pad is detected | Estimate error |
|---|---|---|
| Old (2 markers) | 41 / 243 | |
| **New (3×3 grid)** | **142 / 243** | mean 0.1 cm, max 0.4 cm |

Big-marker acquisition still works at 8 and 10 m.

**Flight** (T6 config + new pad), rough seeds 0/1/2: **0.968 / 0.258 / 0.118 m**, 4 retries each,
~125 s per landing. **Conclusion:** better on two seeds, but the retries now happen *in the last
half-metre*, where the camera sees only ~40 × 30 cm. That wastes the retries and makes landings slow.

## T8. Simulated rangefinder + `PLND_ALT_MIN 0.5`

**Change:** `config/sitl_rangefinder.parm` (`RNGFND1_TYPE 100`, 0.2–12 m), loaded by default;
`PLND_ALT_MIN 0.5` (commit to landing below 0.5 m), `PLND_ALT_MAX 10`.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | **0.623** | **0.630** | **0.617** |
| Retries | 0 | 0 | 0 |
| Duration | 41.5 s | 41.2 s | 41.5 s |

**Conclusion:** consistent, no retries, 3× faster than T7. The trace shows tracking holds down to
~0.7 m (markers 1–8 in view), estimates within 3 mm. The remaining ~0.6 m is the **lag behind the
deck's sway** during the final descent. The near-identical errors come from the same deck phase at
touchdown (same timing on every seed).

## T9. Kalman + moving target, re-tested with all fixes

**Config:** T8 but `PLND_EST_TYPE 1`, `PLND_OPTIONS 1`.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | 2.074 | 1.872 | 1.797 |

Offset 0.6–6 m altitude: mean 1.16 m / max 3.13 m (vs raw: 0.60 / 1.08 m).
**Conclusion:** worse even with continuous tracking. Raw stays.

## T10. Descent gating `PLND_XY_DIST_MAX 0.3`

**Config:** T8 + `PLND_XY_DIST_MAX 0.3` (pause descent while > 0.3 m off).

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | 0.885 | 0.208 | 0.897 |
| Duration | 186 s | 173 s | 186 s |

**Conclusion:** not more accurate on average (0.66 vs 0.62 m), and 4× slower. Reverted to 2.5.

## T11. Faster final descent `LAND_SPD_MS 0.7`

**Idea:** less blind time in the last 0.5 m. **Config:** T8 + `LAND_SPD_MS 0.7`.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | 0.856 | 0.889 | 1.494 |

**Conclusion:** worse. Reverted to 0.4.

## T12. Land-on-tag acceptance test (`./sim.sh verify`)

**Config:** final (T8). Scenario `ontag`: still deck, 1.5 m approach error. Pass = drone centre
inside the 15 cm landing tag.

| Seed | 0 | 1 | 2 |
|---|---|---|---|
| error_m | 0.029 | 0.027 | 0.027 |
| On tag | PASS | PASS | PASS |

**Negative control:** same scenario with `--no-vision` → `FAIL: touchdown 1.431 m from the landing
point`. **Conclusion:** the test can fail, and the final config lands well inside the tag (7.5 cm
half-width) every time, better than T1's calm 4.5–5.6 cm.

## T13. Final matrix, 3 seeds

**Config:** final: raw estimator, `PLND_TIMEOUT 1`, `PLND_ALT_MIN 0.5`/`ALT_MAX 10` + SITL
rangefinder, 3×3 pad, `LAND_SPD_MS 0.4`. `SEEDS="0 1 2" ./sim.sh matrix`.

| Scenario | Mean / worst error | On tag | On pad | Tilt OK | Mean touchdown tilt |
|---|---|---|---|---|---|
| Calm, GPS only | 1.45 / 1.57 m | 0/3 | 0/3 | 3/3 | 0.8° |
| **Calm, vision** | **0.07 / 0.07 m** | **3/3** | 3/3 | 3/3 | 1.0° |
| Moderate | 0.36 / 0.48 m | 0/3 | 3/3 | 2/3 | 4.4° |
| Moderate + platform | 0.36 / 0.39 m | 0/3 | 3/3 | **3/3** | 0.7° |
| Rough | 0.62 / 0.64 m | 0/3 | 2/3 | **0/3** | 7.5° |
| Rough + platform | 0.61 / 0.63 m | 0/3 | 2/3 | **3/3** | 1.1° |

Raw rows: `logs/matrix_final.csv` (not in git; rerun with `SEEDS="0 1 2" ./sim.sh matrix`).

**Conclusion:** calm sea lands on the tag every time; vision is ~20× better than GPS alone. Moderate
and rough land on the pad (rough on 2/3, because the pad-edge margin is tight at 0.6 m). Rough went
from 2.0–3.4 m (T1–T3) to 0.6 m. **The platform does its job:** touchdown tilt is safe (≤ 5°) in
3/3 runs with it vs 0/3 without in rough sea, and 2/3 → 3/3 in moderate. Position error is the same
with and without it, as expected, since it doesn't cancel translation.

---

## Conclusions so far

1. **Vision is not the bottleneck.** Pad estimates are within millimetres to a few cm in every trace.
2. **Config matters more than it looks.** `PLND_ALT_MIN`/`ALT_MAX` without a rangefinder silently
   disabled retries (T5). Fit a rangefinder on the real drone, or set both to 0.
3. **Pad design matters.** A marker grid keeps tracking alive in the last metre (T7, 3.5× coverage).
4. **For a fast-swaying deck, raw measurements beat ArduPilot's Kalman/moving-target mode** (T4, T9).
5. **The limit is now control lag.** Stock precision landing can't follow ±1 m sway with 3–9 s
   periods closely enough. Options: (a) deck-motion-aware guidance on the companion computer
   (predict the deck and send velocity/position targets in GUIDED, timing touchdown for a calm
   moment); (b) a larger pad or platform; (c) re-check whether the rough scenario's sway is
   realistic for the actual boat; (d) Gazebo with real deck contact.
6. **The platform** removes tilt (rough: 7.5° → 1.1° at touchdown; safe-tilt landings 0/3 → 3/3),
   which is its job; position accuracy must come from guidance.

## Real hardware

Nothing flown yet. Re-tune `PLND_*` on the real X500, starting from `config/plnd.parm`, and add
rows here.
