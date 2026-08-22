# DroneProgram architecture

The reference for every producer/consumer boundary in the system.

**The rule this document exists to enforce: one schema change means one edit to
the table below, in the same commit.** The reason is specific. The vision bridge
published `err_x,err_y,id` as CSV while `landing.py` parsed JSON, and that
mismatch survived for months because no single file revealed it — the producer
and the consumer were edited in different weeks and each looked correct on its
own. A boundary with no owner is a boundary that drifts.

---

## 1. One contract per hop

| Producer | Transport | Payload | Consumer |
| --- | --- | --- | --- |
| Gazebo `downward_camera` | gz topic, **model-scoped** (§5) | `gz.msgs.Image`, 320×240 RGB8, 10 Hz | `perception/vision_bridge.py` |
| `vision_bridge.py` | UDP `5005 + 10·i` | `{t, seq, w, h, fx, drone_id, detections[]}` (§2) | `DroneMission.vision_data` |
| Gazebo `rplidar_a1` | gz topic → `ros_gz_bridge` → `/drone_i/scan` | `sensor_msgs/LaserScan`, 360 pts, 10 Hz | `avoider_node.py` |
| `avoider_node.py` | UDP `5006 + 10·i` | `{t, seq, action, eff_front_m, …}` (§3) | `DroneMission.lidar_data` |
| PX4 SITL | MAVLink UDP `14540 + i` | MAVSDK telemetry / offboard | `DroneMission` + `SetpointPublisher` |
| `DroneMission` | HTTP `:5050` | obstacle report / bbox query | `obstacle_memory_service/` (aiosqlite) |
| `DroneMission` | in-process registry | `get_state_snapshot(drone_id)` | `fleet_dispatch/app.py` → SQLite → WebSocket → Leaflet |

Ports are derived, never typed: `config.mavsdk_port()`, `config.vision_port()`,
`config.lidar_port()`, all keyed off `config.drone_index(drone_id)`.

---

## 2. Vision schema

One datagram **per frame**, including frames with no detections:

```json
{
  "t": 12345.678,          // seconds, time.monotonic() in the PRODUCER
  "seq": 4711,             // monotonically increasing; gaps mean dropped datagrams
  "w": 320, "h": 240,      // live image size, not a config constant
  "fx": 277.2,             // derived from the LIVE width: (w/2)/tan(hfov/2)
  "drone_id": "drone-0",
  "detections": [
    {
      "id": 0,                       // ArUco DICT_4X4_50 id
      "err_x": -12.5, "err_y": 5.0,  // px from image centre: +x right, +y down
      "area_px": 3021.0,
      "size_px": 55.4,               // mean side length; compare with MARKER_MIN_DECODE_PX
      "corners": [[x, y], [x, y], [x, y], [x, y]]
    }
  ]
}
```

Three properties are load-bearing:

- **`detections` is a list, and every marker in frame is in it.** The old bridge
  sent `corners[0][0]` — the first marker only — so `landing.select_target()`
  could never see the pad it was asked for when several were visible.
- **An empty list is a positive statement.** "The camera is alive and sees
  nothing" must be distinguishable from "the bridge is dead". Sending nothing at
  all conflates them.
- **`t` is monotonic in the producer's process.** The consumer does *not* use it
  for staleness; `udp_receiver` stamps its own arrival time, because what matters
  is "how long since I heard anything" and only the receiver can answer that.

---

## 3. Avoidance schema

One datagram **per scan**, including clear ones:

```json
{
  "t": 12345.678, "seq": 4711,
  "action": "CLEAR",            // CLEAR | DODGE_LEFT | DODGE_RIGHT | ESCALATE
  "eff_front_m": 15.0,          // min(median, min) of the front cone
  "med_left_m": 15.0, "med_right_m": 15.0,
  "dodge_age_s": 0.0,           // how long the current dodge has been latched
  "drone_id": "drone-0"
}
```

`ESCALATE` is **level-triggered, not edge-triggered**: it is re-sent on every
scan while the condition holds. UDP is lossy, and a once-only edge signal that
happened to be dropped would never fire again — the drone would dodge into the
wall forever. The consumer latches it per dodge episode and bounds re-planning
with `config.DETOUR_MAX_ATTEMPTS_PER_LEG`.

---

## 4. Mission state machine

```
                          ┌──────┐
                          │ IDLE │
                          └──┬───┘
                       start │           battery_low ──► ABORT
                          ┌──▼──────┐
              ┌──────────►│ TAKEOFF │──── fail_x2 ─────► ABORT
              │           └──┬──────┘
              │ more_legs    │ altitude_reached
              │           ┌──▼──────┐
              │           │ ENROUTE │──── battery_low ─► ABORT
              │           └──┬──────┘└─── blocked ─────► HOVER_AND_ALERT
              │              │ arrived
              │        ┌─────▼─────┐
              │   ┌───►│ SEARCHING │─── search_exhausted ─► HOVER_AND_ALERT
              │   │    └─────┬─────┘
              │   │          │ marker_locked
              │   │ lock_lost│
              │   │    ┌─────▼────┐
              │   ├────┤ APPROACH │
              │   │    └─────┬────┘
              │   │          │ centered_stable
              │   │    ┌─────▼───┐
              │   └────┤ DESCEND │◄── marker_drift
              │        └─────┬───┘
              │              │ touchdown
              │      ┌───────▼──────┐
              │      │  PAYLOAD_OP  │── op_failed_x2 ──► HOVER_AND_ALERT
              │      └───────┬──────┘
              │              │ op_confirmed
              │        ┌─────▼────┐
              └────────┤ NEXT_LEG │─── mission_complete ─► DONE
                       └──────────┘
```

`SafetySupervisor` can `force(ABORT)` from **any** state. A safety layer must not
need the transition table's permission to record that a drone is coming down.

Terminal states (`DONE`, `ABORT`) accept no events, so a late event from a task
being torn down cannot resurrect a finished mission.

---

## 5. The offboard-stream invariant

**PX4 leaves OFFBOARD mode if the setpoint stream gaps for more than roughly
half a second.**

One task per mission (`SetpointPublisher`) publishes `self._setpoint` at 20 Hz,
unconditionally, for the mission's whole duration. Mission code *only mutates
that object* — it never calls the offboard plugin.

That is a structural invariant, not a convention, and it replaces four separate
ways the old code broke it:

1. **Mode thrashing.** `navigate_with_avoidance` called `offboard.start()` then
   `stop()` per leg, and `execute_precision_landing` called `start()` again — six
   start/stop pairs per 3-leg mission. `drone_logic.py`'s own docstring warned
   that "PX4 cannot handle rapid mode thrashing" while the code did exactly that.
2. **Frame mixing.** `landing.py` alternated `set_velocity_body()` for the
   descent with `set_velocity_ned()` for the search, inside one session, at
   10 Hz. PX4 tracks which setpoint type a session is streaming.
3. **Blocking calls on the critical path.** Every setpoint was sent from the
   mission's own tick, so a synchronous `subprocess.run("gz service …")` at
   10 Hz (in `payload.py`) froze the loop and starved the stream. This is
   finding F5.
4. **No visibility.** Nothing measured stream continuity. `SetpointPublisher`
   tracks `max_gap_s` and logs an error above 0.4 s, while there is still time
   to react.

**Only NED is streamed.** World-frame velocity is unaffected by attitude. A
body-frame command is relative to the nose, and when PX4 brakes the nose pitches
up — a flat-mounted 2D LiDAR then stares at the sky, returns infinity, the
avoider declares the path clear, and the drone lunges into the wall it was
braking for. Landing thinks in body terms because the camera is bolted to the
airframe, so `command_body_horizontal()` rotates into NED **at the call site**
using the held heading, keeping the publisher single-frame.

---

## 6. The marker/altitude budget

The camera is 320×240 with `hfov = 1.047 rad`, so

```
fx = (w/2) / tan(hfov/2) = 160 / tan(0.5235) = 277.2 px
```

A marker of side `s` at altitude `h` spans `fx·s/h` pixels, and a 4×4 ArUco tag
needs roughly **25–30 px** to decode reliably:

| Altitude | 0.5 m pad (original) | 2.0 m pad (shipped) |
| --- | --- | --- |
| 10 m (cruise) | **13.9 px** — undecodable | 55.4 px |
| 6 m (`SEARCH_ALT_M`) | 23.1 px — marginal | **92.4 px** |
| 3 m | 46.2 px | 184.8 px |

The documented flow — arrive at `TARGET_ALT`, then search — therefore could not
lock even with a decodable texture. **Both** levers are applied, because either
alone is fragile: 2 m pads *and* a descent to `SEARCH_ALT_M` before searching.

**The figures above are plane widths, and the decoder sees less.** The pad
textures carry a white quiet zone — the reason they decode at all — so the black
marker is 79.6% of the plane (measured: 199 px of a 250 px texture).
`config.decodable_px_at_altitude()` applies that factor, and it is the function
to compare against `MARKER_MIN_DECODE_PX`. Marker-corrected, the shipped pad is
44 px at cruise altitude, and the original 0.5 m pad is **below the threshold at
every altitude the mission would have searched from** — not merely marginal.
Confirmed in flight at 5.51 m: 88.6 px measured against an 80.1 px
marker-corrected prediction. See [CALIBRATION.md](CALIBRATION.md) § 3.

`config.PAD_SIZE_M` must match `sim/models/pad_N/model.sdf`, and
`config.CAMERA_*` must match `sim/models/x500_delivery/model.sdf`. Both pairs
are asserted by `tests/test_landing_control.py`, and `scripts/preflight.py`
checks the live sensor against config before anything flies.

### Why the loop runs in metres

Pixel error for a fixed ground offset grows as `1/h`, so a pixel-gain controller
has effective gain `K_p·fx/h`:

| Altitude | Effective gain (old, `K_p = 0.015`) |
| --- | --- |
| 10 m | 0.42 s⁻¹ — sluggish but stable |
| 1 m | **4.16 s⁻¹** — overshoot and oscillation, exactly at touchdown |

Converting to metres *first* (`offset_m = err_px · h / fx`) and then applying a
fixed metric gain makes the loop altitude-invariant. This also delivers most of
what `solvePnP` was deferred for, with no calibration rig. Signs are derived in
[CALIBRATION.md](CALIBRATION.md).

---

## 7. Model-scoped sensor topics

`sim/models/x500_delivery/model.sdf` declares **no `<topic>` element**. Gazebo
then derives the name from the entity path:

```
/world/<world>/model/<model>_<instance>/link/<link>/sensor/<sensor>/<suffix>
```

so drone-0's camera is
`/world/delivery/model/x500_delivery_0/link/camera_link/sensor/downward_camera/image`.

This is deliberate and must not be "fixed" by adding a topic back. The previous
in-submodule edit hardcoded absolute `/camera/image` and `/lidar/scan`, so with
`FLEET_SIZE = 3` all three drones published onto the same two topics: every
vision bridge saw a blend of three cameras and every avoider reacted to a blend
of three LiDARs. Per-drone UDP ports solved the downstream half of that and none
of the upstream half. This is finding F7.

`sim_topics.py` owns the derivation. Nothing else builds these strings —
`scripts/preflight.py`, `run_system.sh` and both consumers all call it, so the
name exists in exactly one place.

Instance naming is PX4's: `px4-rc.gzsim` spawns `${PX4_SIM_MODEL}_${instance}`.
Note that `PX4_GZ_MODEL_NAME` does **not** rename a spawned model — setting it
makes PX4 skip spawning and attach to a pre-existing one, which is a different
launch mode.

---

## 8. Fail-closed sensing

Navigation and landing both hold position rather than proceed when a feed goes
quiet. That was already the intent, but the timestamps the checks read
(`last_lidar_ts`, `last_telemetry_ts`) **were never written by anything**, so the
consequences were wrong in both directions:

- navigation computed `time.time() - 0`, concluded the LiDAR feed was stale on
  its first tick, and held position forever;
- the supervisor computed the same difference and fired an emergency RTL at
  `t = 0`, before telemetry could possibly have arrived.

`drone_agent/udp_receiver.py` is the missing writer. It also distinguishes two
states that must not be conflated:

- `is_stale()` — treats "never heard from" as stale. Correct for navigation:
  never infer a clear path from silence.
- `has_ever_arrived()` — false until the first datagram. What lets the
  supervisor's heartbeat check avoid firing on a cold start.

All durations are `time.monotonic()`. Wall clock is subject to NTP steps, and a
backwards step in flight makes a timeout fire instantly or never.

---

## 9. Files that no longer exist

Duplication is the mechanism by which a fix gets applied to the copy that is not
running. Each deletion has a named survivor:

| Deleted | Survivor | Why |
| --- | --- | --- |
| `drone_web/app.py` | `fleet_dispatch/app.py` | Two dispatchers; the canonical one already had the error handling |
| `obstacle_memory_service.py` (root) | `obstacle_memory_service/` | In-memory dict with private constants vs. persistent and config-driven |
| `working code reference/` | git history | Two files byte-identical; the third was 438 lines of commented-out dead code |
| `RUN_SYSTEM.sh` | `scripts/run_system.sh` | Started one bridge and one avoider — a single-drone launcher |
| `vision_bridge.py` (root) | `perception/vision_bridge.py` | `perception/` was an empty placeholder |

Plus one consolidation: `get_distance_m` was defined **six times** with
identical bodies (and `get_bearing` twice more). One copy now lives in
`drone_agent/geo.py`. Six copies of a formula means a correction to one leaves
five wrong — and it meant the geofence check and the detour planner could
disagree about distance while both looked right in isolation.

---

## 10. Where the tunables live

`config.py` is the **only** source. Nothing else may re-declare a threshold that
lives there.

This is not a style preference. `drone_web/drone_logic.py` lines 41–50
re-declared ten navigation constants at module scope, and `avoider_node.py`
re-declared five as class constants. Editing `config.py` therefore had no effect
on what actually flew, which made the file that claims to centralise tuning the
most misleading file in the repository. `make check` greps for their return.

Naming: every quantity carries its unit — `_m`, `_m_s`, `_m_s2`, `_pct`, `_px`,
`_deg`, `_rad`, `_s`, `_hz`. Two of the seven findings behind this work were unit
bugs (pixels treated as metres; battery percent treated as a 0–1 fraction) that a
name would have caught.

**One deliberate exception:** `SAFE_DIST`, `CLEAR_DIST`, `CLEAR_CONFIRM_S`,
`MIN_LOCK_S` and `ESCALATION_LOCK_S` keep their unit-less names.
`tests/test_avoider_node.py` reads them by name and passes them positionally into
`decide_action()`, and the acceptance criterion for that phase is that the test
passes *unmodified*. Renaming them to satisfy a convention would have meant
editing the test to fit the implementation, which is backwards.
