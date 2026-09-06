# Calibration

Every number in this file is either derived from a measurement of this machine or
flagged as an uncalibrated placeholder. Nothing here is inherited from a generic
PX4 tutorial.

---

## 1. Camera intrinsics

Read from `sim/models/x500_delivery/model.sdf`:

| Property | Value |
| --- | --- |
| Resolution | 320 × 240 |
| Horizontal FOV | 1.047 rad (60.0°) |
| Update rate | 10 Hz |
| Mount pose (relative to `base_link`) | `0 0 -0.05 0 1.5708 0` |

```
fx = (w/2) / tan(hfov/2) = 160 / tan(0.5235) = 277.2 px
```

`config.CAMERA_FX_PX` computes this rather than storing it, so it cannot drift
away from the resolution and FOV it is derived from.

`scripts/preflight.py` compares the **live** image dimensions against config and
refuses to declare the system flight-ready on a mismatch. It has to: the whole
landing control law converts pixels to metres through `fx`, so a model swap that
changes the resolution silently rescales every command.

---

## 2. The sign convention (finding F6)

**This is the most dangerous thing in the landing loop to guess at.** Invert
either sign and the controller becomes positive feedback that accelerates away
from the pad — and the symptom (the drone drifts off, the landing times out)
looks identical to a marker that was never properly acquired. Someone under
pressure will flip a sign to see if it helps. Hence the derivation, and hence the
script that confirms it.

### Derivation

The mount is `pose 0 0 -0.05 0 1.5708 0` — a **+90° pitch about Y**. Gazebo
cameras look down **+X**, with image right = **−Y** and image down = **−Z**.

Applying R_y(90°) to the camera's axes, expressed in body FRD (Forward-Right-Down):

```
X_cam → ( 0,  0, −1)   camera boresight → body DOWN     ✓ it looks down
Y_cam → ( 0,  1,  0)   → body LEFT
Z_cam → ( 1,  0,  0)   → body FORWARD
```

Therefore:

```
image u (rightwards) = −Y_cam  →  body RIGHT
image v (downwards)  = −Z_cam  →  body AFT
```

So a marker **below** the image centre (positive `err_y`) is **behind** the
drone, and closing on it means flying backwards:

```
forward_m = −err_y · h / fx
right_m   = +err_x · h / fx
```

These are the signs the original code used. They are now derived rather than
guessed, and pinned by `tests/test_landing_control.py::
test_body_frame_signs_match_the_derived_camera_mount`.

**Corollary, useful when reading logs:** flying forward makes the pad move DOWN
the image, so `err_y` *increases*. Flying right makes the pad move LEFT, so
`err_x` *decreases*.

### Empirical confirmation

```bash
# With PX4 SITL up, the drone hovering over a pad, and the vision bridge running:
python3 scripts/calibrate_camera_signs.py --drone-id drone-0 --axis forward
python3 scripts/calibrate_camera_signs.py --drone-id drone-0 --axis right
```

The script commands a known body-frame velocity, observes which way the marker's
pixels moved, and reports **AGREES** or **CONTRADICTS**. It also back-computes
`fx` from the pixel displacement over the known ground displacement.

Expect roughly 10–20% disagreement on that `fx` estimate: the commanded velocity
is not the achieved velocity during the acceleration phase, so the true ground
travel is less than `speed × duration`. A discrepancy much larger than that means
the intrinsics are genuinely wrong.

**Record each run here.** Table below; the derivation above is the prediction, a
run is the evidence.

| Date | Axis | Speed | Δerr (px) | Verdict | fx measured | By |
| --- | --- | --- | --- | --- | --- | --- |
| _pending_ | forward | 0.4 m/s | — | — | — | — |
| _pending_ | right | 0.4 m/s | — | — | — | — |

The remediation shipped with these rows unfilled because the confirmation needs a
live simulator. Filling them in is the last item on the P4 gate.

---

## 3. The marker/altitude budget (finding F3)

A marker of side `s` at altitude `h` spans `fx·s/h` pixels. A 4×4 ArUco tag needs
roughly **25–30 px** to decode reliably (`config.MARKER_MIN_DECODE_PX = 25`).

| Altitude | 0.5 m pad | 2.0 m pad |
| --- | --- | --- |
| 10 m (`TARGET_ALT_M`) | **13.9 px** | 55.4 px |
| 6 m (`SEARCH_ALT_M`) | 23.1 px | **92.4 px** |
| 5 m | 27.7 px | 110.9 px |
| 3 m | 46.2 px | 184.8 px |

Reproduce this table with:

```bash
python3 -c "
import config
for h in (10, 6, 5, 3):
    print(h, round(config.marker_px_at_altitude(h, 0.5), 1),
             round(config.marker_px_at_altitude(h), 1))"
```

**Two levers, both applied.** 2 m pads alone would work at cruise altitude
(55 px), and a 6 m search altitude alone would be marginal with 0.5 m pads
(23 px). Together they give about 4× headroom, which is what absorbs motion
blur, oblique viewing angles and lighting variation. Either alone is fragile.

`config.PAD_SIZE_M` and `sim/models/pad_N/model.sdf` must agree; asserted by
`tests/test_landing_control.py::test_config_pad_size_matches_the_shipped_model`.

### The quiet zone costs 20% of the budget — measured

The table above is the width of the **plane**. The ArUco decoder only ever sees
a marker, and since the pads became a nested set there is no single marker width
to quote — that is the point of them.

> **Superseded.** This section described the original pad: one marker filling
> 79.6% of a 2.0 m plane (199 px of a 250 px texture). That geometry is why the
> pad became **undetectable below 2.52 m** — a marker that large outgrows a
> 320×240 frame long before touchdown — and it is the root cause of the landing
> failure this project spent a rewrite on. It is recorded here because the
> measurement behind it, that the decoder sees the marker and not the plane it
> is painted on, is still true and still load-bearing.

The shipped pads carry **five markers at two scales**, and the authoritative
budget lives in `world/pad_layout.py`:

| id | side | position | in frame ≥ | decodes ≤ |
| --- | --- | --- | --- | --- |
| `p` | 0.40 m | (0.00, 0.00) | 0.69 m | 4.44 m |
| `10p+10` / `10p+11` | 0.58 m | (∓0.65, 0.00) | 1.88 m | 6.43 m |
| `10p+12` / `10p+13` | 0.58 m | (0.00, ∓0.65) | 2.51 m | 6.43 m |

Print the live table, derived from the shipped geometry, with:

```
$ python3 -c "
import math, config, world.pad_layout as L
tan_h = math.tan(config.CAMERA_HFOV_RAD / 2)
tan_v = (config.CAMERA_HEIGHT_PX / 2) / config.CAMERA_FX_PX
print(L.describe(config.CAMERA_FX_PX, tan_h, tan_v))"
```

`scripts/preflight.py` re-derives the same numbers from the committed textures
and fails if `SEARCH_ALT_M`, `COMMIT_ALT_M` or `NO_CLIMB_ALT_M` fall outside the
band the pad actually supports.

### Calibrating the offset, not the pixels

The landing loop no longer converts pixels to metres using an altitude. Each
marker's real size is known, so `perception/pad_estimator.py` fits a homography
from the marker's four pad-frame corners to its four image corners; the pad
centre is that homography applied to (0, 0), and the Jacobian there is the local
pixels-per-metre. Accuracy against rendered ground truth is **3–8 mm below 3 m**
and 21 mm at 8 m, with range good to better than 1% below 6 m
(`tests/test_pad_estimator.py`).

**This correction strengthens F3 rather than weakening it.** On plane widths the
original 0.5 m pad looked merely "marginal at 5 m" (27.7 px). Marker-corrected it
is **22.1 px at 5 m — below the threshold at every altitude the mission would
have searched from.** The shipped 2 m pad clears it even at full cruise altitude.

### Confirmed in flight, 23 Aug 2026

Taken during the remediation, over `pad_0` in the `delivery` world:

| Altitude | Predicted (plane) | Predicted (marker) | **Measured** |
| --- | --- | --- | --- |
| 5.51 m | 100.6 px | 80.1 px | **88.6 px** |

The measurement sits between the two and closer to the marker-corrected figure,
which is what the correction predicts. First acquisition happened at 2.0 m during
the climb at 219 px. Only marker ID 0 was ever reported — the correct pad.
Pinned by `test_flight_measured_marker_size_matches_the_corrected_budget`.

### ArUco decodability, measured

Verified with OpenCV 4.13.0 against all 27 predefined dictionaries:

```
arucotag     354×354   NOT DECODABLE (all 27 dictionaries)
arucotag_0   250×250   DICT_4X4_50 → [0]
arucotag_1   250×250   DICT_4X4_50 → [1]
arucotag_2   250×250   DICT_4X4_50 → [2]
arucotag + 40 px white border → DICT_4X4_50 → [0]
```

`model://arucotag` — which the old code spawned for **all three** pads — has no
white quiet zone, and ArUco's contour stage cannot segment a marker without one.
That single asset is why precision landing never worked, independently of every
other bug. This is finding F1.

Note also that `arucotag_0` and `arucotag_2` additionally decode under
`DICT_ARUCO_MIP_36h12` as IDs 102 and 116. Harmless here, and the reason the
detector is configured with **exactly one** dictionary: a multi-dictionary
detector would report phantom markers.

`scripts/preflight.py` re-runs this decode check on every preflight, so a pad
asset regression fails on the bench rather than in the air.

---

## 4. LiDAR geometry

From `sim/models/x500_delivery/model.sdf`:

| Property | Value |
| --- | --- |
| Samples | 360 over −π…+π (1°/sample) |
| Range | 0.15 – 12.0 m |
| Update rate | 10 Hz |
| Mount | `0 0 0.1 0 0 0` relative to `base_link` |

### The self-hit mask

The propellers sit approximately **0.29 m** from the airframe centre, which is
**above** the sensor's own 0.15 m floor. A prop tip is therefore a physically
valid return — and without masking it reads as a permanent 0.29 m wall directly
ahead. The avoider locks a dodge it can never clear, escalates after 12 s, and
the mission stalls forever.

`config.MIN_VALID_RANGE_M = 0.5` clears the props with margin while staying far
below `SAFE_DIST = 6.5 m`.

**Verify on the bench.** Hover in an empty world for 60 s and confirm the logged
`eff_front_m` stays at `INF_REPLACE_M` (15.0):

```bash
tail -f /tmp/droneprogram/avoider_0.log
```

Any sustained reading near 0.3 m means the mask is not doing its job — check
whether the prop radius on this airframe is larger than assumed.

`config.INF_REPLACE_M` (15.0) must exceed `CLEAR_DIST` (9.0), or an empty world
would never read as clear. `validate_scan_geometry()` asserts this at startup.

---

## 5. Battery energy model — UNCALIBRATED

```python
config.BATTERY_ENERGY_PER_M_PCT = 0.01  # % of pack per metre
```

**This is a placeholder**, a conservative guess from typical multirotor figures
(≈3.5 m/s cruise, ≈200 W hover), not a measurement of this airframe. It is
deliberately in `config.py` with that caveat attached rather than buried in
`battery.py`.

It also ignores altitude changes and wind, both of which make it optimistic.
`config.BATTERY_RESERVE_MARGIN_PCT = 15` sits on top of it for that reason.

### Procedure to replace it with real data

1. Fully charge. Record `battery_pct` from `/fleet/status`.
2. Fly a straight leg of known length (≥200 m) at `CRUISE_SPEED_M_S`, level.
3. Record `battery_pct` on arrival.
4. `BATTERY_ENERGY_PER_M_PCT = (start_pct − end_pct) / distance_m`
5. Repeat three times; take the **highest** value, not the mean. An
   underestimate strands a drone; an overestimate only refuses a leg.
6. Record the result in the table below and update `config.py`.

| Date | Distance | Start % | End % | %/m | Airframe |
| --- | --- | --- | --- | --- | --- |
| _pending_ | — | — | — | 0.01 (placeholder) | SITL x500_delivery |

**Units.** MAVSDK 2.x reports `Battery.remaining_percent` on **0–100**; in 1.x it
was **0–1**. `pyproject.toml` pins `mavsdk>=2.0,<3` for exactly this reason —
the old `>=1.4.0` spanned the break, so the same comparison was either 100×
too strict or 100× too lax depending on what pip resolved.

---

## 6. Geodesy accuracy

`drone_agent/geo.py` uses flat-earth (equirectangular) approximations: one degree
of latitude is a constant 111 320 m, longitude scaled by `cos(lat)`.

| Range | Approximate error |
| --- | --- |
| 500 m (`GEOFENCE_RADIUS_M`) | < 0.1 m |
| 1 km | ≈ 0.2 m |
| 10 km | ≈ 2 m |

Acceptable because the geofence is 500 m and PX4 enforces it, so the error is
always far below GPS noise. **If the geofence is ever widened past a few
kilometres, replace these with haversine/Vincenty rather than adjusting a
constant** — the approximation's failure mode is a slow drift, not an obvious
break, so it will not announce itself.

---

## 7. PX4 parameters

Applied from `config.PX4_PARAMS` at mission start by
`drone_agent/px4_params.py`, and **read back** to confirm.

| Parameter | Value | Why this value |
| --- | --- | --- |
| `MPC_ACC_HOR` | 2.0 | Equal to `config.MAX_ACCEL_M_S2`, so our slew limit and the autopilot's agree instead of one silently dominating |
| `MPC_ACC_HOR_MAX` | 5.0 | Headroom for avoidance manoeuvres |
| `MPC_JERK_MAX` | 8.0 | Lower is smoother; reduces EKF stress |
| `MPC_XY_VEL_MAX` | 4.0 | Above `CRUISE_SPEED_M_S = 3.5`, so cruise is not clipped |
| `MPC_XY_P` | 0.95 | Tighter position hold; higher risks oscillation |
| `MPC_Z_VEL_MAX_DN` | 1.0 | Caps descent above the landing schedule's 0.8 m/s |
| `MPC_Z_VEL_MAX_UP` | 3.0 | Brisk takeoff |

**Read-back matters as much as writing.** PX4 silently ignores a set for an
unknown parameter name, and clamps values outside a parameter's declared range. A
write-only "apply" would report success for a value the autopilot rejected, so
every flight would be of unknown configuration — and comparing two flights would
be meaningless.

These were previously a comment block in `config.py` asking the operator to type
seven values into a PX4 shell. A manual step does not survive a reboot, cannot be
reviewed in a diff, and leaves nothing in the logs to distinguish "tuned" from
"never typed".
