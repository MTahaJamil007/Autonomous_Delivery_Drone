# DroneProgram status

**Rule: nothing gets ✅ without a call site and a named verification.** Every row
below carries a "Verified by" column naming either an automated test or a
procedure in [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md).

That rule is the point of this document. The previous version of this file marked
the FSM, the payload bay and the safety supervisor as "Working" while **nothing
in the repository imported them**, and asserted that `pytest tests/` succeeded
when it aborted during collection and ran zero tests. The docs oversold
completion, and that was the real defect to fix — the code problems were
downstream of it.

**Legend**

| | Meaning |
| --- | --- |
| ✅ | Has a call site, and an automated test proves the behaviour |
| 🛫 | Implemented and wired, but the acceptance criterion needs a live PX4 — procedure written, **not yet run** |
| ⚠️ | Known limitation, deliberately accepted |

Nothing is marked ✅ on the strength of the code existing.

---

## Documented behaviours

These are the nine behaviours `STATUS.md` and `RUN_GUIDE.md` commit to, taken
literally.

| Behaviour | State | Verified by |
| --- | --- | --- |
| 3-leg delivery mission ends in FSM `DONE`, job `COMPLETED` | 🛫 | `test_the_full_happy_path_reaches_done` (FSM path); ACCEPTANCE § 4 (flight) |
| Precision landing on ArUco 0/1/2 with disambiguation | 🛫 | `test_select_target_never_returns_the_wrong_marker`, `test_two_markers_appear_in_one_frame_with_distinct_ids`; **live acquisition confirmed 23 Aug** (marker 0 decoded from the real camera at 6 m — the behaviour that had never once worked); the 20-landing gate is ACCEPTANCE § 7 |
| LiDAR reactive dodging with time hysteresis | ✅ | `test_replay_clear_dodge_escalate_clear_without_wall_clock`, `test_hysteresis_band_keeps_dodging`; live at 10 Hz with no self-returns (ACCEPTANCE § 2, 23 Aug) |
| Obstacle escalation after 12 s triggers detour planning | ✅ | `test_replay_...` (emission), `test_a_known_obstacle_changes_the_planned_route` (planning); ACCEPTANCE § 10 (flight) |
| Battery monitoring gates each leg | ✅ | `test_gate_@18%` in `battery.check_leg_battery`; call site in `DroneMission.fly_leg` |
| Safety supervisor: heartbeat + geofence, direct authority | ✅ | `test_dispatch_lifecycle` + the three P3.5 defect checks; instantiated in `DroneMission.prepare` |
| Payload bay kinematically follows the airframe | 🛫 | `PayloadBay` instantiated in `DroneMission.prepare`, attach/release at both PAYLOAD_OP entries; ACCEPTANCE § 5 (flight) |
| Fleet-wide obstacle sharing, persisted between missions | ✅ | `test_obstacles_survive_a_restart`, `test_one_wall_becomes_one_row_with_rising_confidence` |
| Multi-drone fleet (3 drones, queued jobs) | 🛫 | `test_fleet_isolation.py` (16 tests: topics, ports, sysids, entity names); ACCEPTANCE § 11 (flight) |

**Why 🛫 and not ✅.** Each of those four has a call site and passes every test
that can be written without a simulator. What is missing is the *statistical* or
*observational* criterion — 19 of 20 landings within 0.5 m, the cargo box visibly
tracking the airframe, three drones actually flying at once. Marking them ✅ on
the strength of the unit tests would be the exact error this document exists to
prevent.

### What has been confirmed against a running simulator

A live session on 23 Aug 2026 verified the parts of the chain that unit tests
cannot reach. Logged in [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md) § 9:

- **The vendored sensors work (F2).** The drone spawned from `sim/models/` with
  camera and LiDAR present, independent of the PX4 submodule.
- **Topics are model-scoped (F7).** Live names matched `sim_topics` character for
  character. No absolute `/camera/image` anywhere.
- **The pad decodes (F1).** `pad_0` spawned, the drone climbed to 6 m, and the
  live camera reported **marker ID 0 and nothing else** across 46 frames. This is
  the single behaviour that had never worked in this system's history.
- **Preflight passes for real** — all six checks, including live camera and scan
  geometry against `config.py`.
- **No LiDAR self-returns** — 10.0 Hz, `eff_front_m` constant at 15.0 m.
- **`udp://` connects where `udpin://` hung forever** (see the MAVSDK note below).

One measurement corrected the plan's arithmetic: the pad textures' white quiet
zone means the decoder sees only 79.6% of the plane, so the F3 budget was ~20%
optimistic. This *strengthens* F3 — marker-corrected, the original 0.5 m pad was
below the decode threshold at **every** altitude the mission would have searched
from, not merely marginal at 5 m. `config.decodable_px_at_altitude()` applies the
measured factor.

---

## Test suite

```
$ scripts/test.sh
126 passed, 3 deselected in ~37 s
```

The 3 deselected are `@pytest.mark.sitl` scenarios, excluded by `pytest.ini`
because they need a live autopilot. Run them with `scripts/test.sh -m sitl`.

**Before this work:** `pytest tests/` reported nothing at all. Two independent
causes:

1. ROS 2 Humble's `launch_testing` pytest plugins are incompatible with pytest 9
   and raised `PluginValidationError` **before collection**, so any run from a
   ROS-sourced shell — which is the normal development shell here, since `rclpy`
   is needed — failed without executing a test. `scripts/test.sh` sets
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
2. `tests/test_avoider_node.py` imported `decide_action`, which did not exist, so
   one collection error aborted the whole run. And
   `test_marker_disambiguation.py:80` called `input()`, which with no TTY never
   returns — the suite hung forever.

| Test file | Count | Covers |
| --- | --- | --- |
| `test_avoider_node.py` | 1 | The pre-existing state-machine test, **unmodified** |
| `test_detour.py` | 4 | Tangent-bypass planner |
| `test_landing.py` | 3 | Marker disambiguation, EMA filter |
| `test_mission_fsm.py` | 3 | Transition table |
| `test_dispatch_lifecycle.py` | 9 | Atomic claim, honest lifecycle, no stranded drones |
| `test_dispatch_endtoend.py` | 3 | Real HTTP dispatch with no autopilot |
| `test_perception_contracts.py` | 16 | Replay, self-hit masking, both sensor schemas |
| `test_mission_core.py` | 30 | Per-drone state, setpoint stream, leg wiring, PX4 params |
| `test_landing_control.py` | 23 | Altitude-invariant gain, descent cone, marker budget |
| `test_detour_memory.py` | 18 | Dedupe, placement, persistence, confidence merge |
| `test_fleet_isolation.py` | 16 | Every shared resource is per-drone |

---

## Repository gates

`make check` runs all of these. Each encodes a regression this codebase actually
had, so a failure names the specific mistake.

| Gate | What it catches |
| --- | --- |
| `make import-check` | A module that only imports from one working directory. **This is the exact check that would have caught the `get_state_snapshot` blocker at commit time.** |
| `make no-shadowed-config` | A tunable re-declared outside `config.py`. Ten were shadowed in `drone_logic.py`, so editing `config.py` had no effect on flight. |
| `make docs-check` | A reference to a `.md` file that does not exist. Seven pointed at a `HOW_TO_RUN` guide that never existed, two of them inside runtime error strings. |
| `make schema-check` | Simulation assets drifting from `config.py` — camera intrinsics, pad size, world origin vs. map centre, and the marker/altitude budget. |
| `make blocking-check` | `subprocess.run`, `cv2.imshow` or `time.sleep` on the flight path. Finding F5: these gap the setpoint stream and PX4 drops OFFBOARD. |

---

## Architecture

| Concern | Owner |
| --- | --- |
| Tunables | `config.py`, and nowhere else |
| Geodesy | `drone_agent/geo.py` — one definition, replacing six copies |
| Mission state and orchestration | `drone_agent/mission.py` (`DroneMission`) |
| Offboard stream | `drone_agent/setpoint.py` — one task, one frame, no gaps |
| Navigation and detour | `drone_agent/navigation.py` |
| Precision landing | `drone_agent/landing.py` |
| Safety | `drone_agent/safety_supervisor.py` + a PX4-enforced geofence |
| Marker detection | `perception/vision_bridge.py` |
| Reactive avoidance | `avoider_node.py` |
| Dispatch and fleet state | `fleet_dispatch/` |
| Obstacle memory | `obstacle_memory_service/` (persistent) |
| Simulation assets | `sim/` — vendored out of the PX4 submodule |
| Topic derivation | `sim_topics.py` |

`drone_web/drone_logic.py` is 90 lines: the contract, and nothing else. It was
528 lines that duplicated the mission core.

Full contract table, FSM diagram and the offboard invariant:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

---

## Known limitations

| ⚠️ | Detail |
| --- | --- |
| Battery energy model is uncalibrated | `BATTERY_ENERGY_PER_M_PCT = 0.01` is a placeholder, not a measurement of this airframe. Replacement procedure in CALIBRATION § 5. |
| Camera signs not yet confirmed in flight | The derivation is recorded and unit-tested; `scripts/calibrate_camera_signs.py` confirms it empirically but has not been run. CALIBRATION § 2. |
| Flat-earth geodesy | ≈2 m error at 10 km. Irrelevant inside the 500 m geofence; replace with haversine before widening it. CALIBRATION § 6. |
| Fleet capped at 10 | PX4's `px4-rc.mavlink` maps instances above 9 onto port 14549. Enforced by `spawn_fleet.sh`. |
| SITL behaviour may not transfer to hardware | Every threshold is in `config.py`; recalibrate `fx` and pad size per CALIBRATION § 1 and § 3. |
| `solvePnP` still unused | `compute_metric_offset()` exists but is not called. The altitude-based conversion delivers most of its value with no calibration rig; full pose estimation would add marker tilt, which nothing needs yet. |

---

## Environment

Measured on this host, not assumed.

| Component | Version |
| --- | --- |
| PX4 | v1.17.0-alpha1-1225 |
| MAVSDK | **2.12.10** — note: not the 2.8.4 the plan recorded |
| Gazebo | Harmonic |
| ROS 2 | Humble |
| Python | 3.10.12 |
| OpenCV | 4.13.0 |
| pytest | 9.1.1 |

⚠️ **MAVSDK 2.12.10 rejects `udpin://`.** It reports `Unknown protocol` /
`Invalid connection URL`, `mavsdk_server` exits, and `System.connect()` then
blocks forever in `channel_ready_future` — so a dispatch hung with the drone
`BUSY`. `config.MAVSDK_URL_SCHEME` is `udp` (the 2.x name; `udpin` is 3.x), and
the gRPC handshake is now separately bounded. See `config.py` for the full note.

---

## Getting started

```bash
make install          # editable install + dev extras
make check            # every gate
source sim/env.sh     # point Gazebo at the vendored assets
world/spawn_fleet.sh  # Gazebo + PX4
scripts/run_system.sh # services, bridges, avoiders, vision
make preflight        # MUST pass before dispatching
```

Then open <http://localhost:5000>. Full walkthrough: [RUN_GUIDE.md](RUN_GUIDE.md).

---

## Where the remaining work is

Ten flight procedures in [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md), none of them
run yet. In priority order:

1. **§ 4** — the three-leg mission. Everything else depends on this passing.
2. **§ 8** — confirm the camera signs before trusting the landing loop.
3. **§ 2** — the LiDAR self-return check; five minutes, and it validates the
   propeller mask.
4. **§ 7** — the twenty landings. Needs `scripts/sitl_landing_trial.py`, which is
   **not shipped**; writing it is the first task of whoever runs that gate.
5. **§ 10** — the two detour runs. The most interesting result in the system:
   run 2 should record zero dodges.
6. **§ 11** — the three-drone fleet.

Expect the first flight after § 4 to surface two or three defects of the same
kind as finding F5 — bugs that only exist once the code actually runs. Roughly a
third of `drone_agent/` was sound and unit-tested before this work and had simply
never been executed against a running autopilot. That is planned for, not a
setback.
