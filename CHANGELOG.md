# Changelog

One history for this project. This file replaces the R1–R6 "fix round" narrative
that was scattered through docstrings and status files, where it was impossible
to tell which round a given comment belonged to or whether it had landed. Git
carries the rest.

---

## 0.8.0 — Remediation

Closes the gap between what the documentation claimed and what the code did.
Eight verification-gated phases, each anchored to a measured fact about this
machine rather than an assumption about PX4 in general.

### The two blockers this started from

- **`fleet_dispatch/app.py:93` imported `get_state_snapshot`, which did not
  exist** — from inside `run_drone_task` and *above* its `try:`. Every dispatch
  raised `ImportError` before entering the try block, so
  `finally: complete_job(...)` never ran and the drone stayed `BUSY` forever. A
  three-drone fleet emptied after three dispatches, each of which the UI had
  reported as successfully assigned.
- **`execute_delivery` was called with five positional arguments against a
  four-parameter definition.** The `TypeError` was swallowed by a broad
  `except Exception` and the job was recorded as `COMPLETED`.

### Seven findings that reshaped the work

Uncovered by inspecting the simulation assets and doing the sensor arithmetic.
Each would have silently defeated a fix applied only at the Python layer.

| | Finding | Fix |
| --- | --- | --- |
| **F1** | `model://arucotag` has no white quiet zone and decodes in **zero** of OpenCV's 27 predefined dictionaries. It was spawned for all three pads, so `locked` never became true — the root cause of precision landing never working, independent of every other bug. | `sim/models/pad_0/1/2` with the decodable `arucotag_N` textures |
| **F2** | The camera and LiDAR were an **uncommitted edit inside the `PX4-Autopilot/Tools/simulation/gz` submodule**. A `submodule update --force` would have deleted both, and nothing checked. | Vendored into `sim/`; `preflight.py` checks the topics |
| **F3** | A 0.5 m marker spans **13.9 px at 10 m** cruise altitude; a 4×4 tag needs 25–30 px. The documented flow could not lock even with a decodable pad. | 2 m pads **and** `SEARCH_ALT_M = 6` |
| **F4** | The landing controller's effective gain was `K_p·fx/h`, rising from 0.42 s⁻¹ at 10 m to **4.16 s⁻¹ at 1 m** — oscillation exactly at touchdown. | Convert pixels to metres first, then a fixed metric gain |
| **F5** | `payload.py` ran `subprocess.run("gz service …")` synchronously from a 10 Hz async tick, freezing the event loop — including the setpoint publisher. Enabling the documented payload feature would by itself have caused mid-flight OFFBOARD loss. | Dedicated setpoint publisher + async subprocess I/O |
| **F6** | The camera sign convention was *correct* — and easy to "fix" into a crash. | Derived, unit-tested, and `scripts/calibrate_camera_signs.py` |
| **F7** | Sensor topics were absolute, so N drones published onto the same two topics and every consumer saw a blend of all of them. | No `<topic>` element; Gazebo derives a model-scoped name per instance |

### P0 — Freeze the baseline

- `.gitignore` covers runtime state; `fleet.db` and `obstacles.db` untracked.
- Deleted `working code reference/` (verified first: one file byte-identical, one
  differing by a trailing newline, one 438 lines of commented-out dead code),
  `drone_web/app.py`, the root `obstacle_memory_service.py`, and `RUN_SYSTEM.sh`.
- `sim/` vendors the world, the sensor-equipped model, three decodable pads and
  the cargo box. `sim/env.sh` documents why its ordering is load-bearing.
- `scripts/preflight.py` refuses to declare the system flight-ready unless the
  topics, geometry, pads, autopilot and services all check out.
- `pytest.ini` + `scripts/test.sh`. ROS Humble's `launch_testing` plugins are
  incompatible with pytest 9 and aborted collection, so `pytest tests/` from a
  ROS-sourced shell ran **zero tests**. Removed the `input()` that hung the suite
  forever with no TTY.
- `pyproject.toml`; deleted the nine `sys.path.insert` blocks. `mavsdk` pinned to
  `>=2.0,<3` — `remaining_percent` changed from 0–1 to 0–100 between majors and
  the old `>=1.4.0` spanned that break.

### P1 — Dispatch path integrity

- `DeliveryJob` / `MissionResult` dataclasses. Four same-typed floats in a row is
  a signature no reviewer and no tool can check.
- Imports at module scope; the stub `execute_delivery` fallback deleted. It
  logged critically and returned successfully, so a build that could not fly
  reported every mission `COMPLETED`.
- `finalize_job(job_id, status, detail)` replaces `complete_job(job_id)`, which
  hardcoded `COMPLETED`.
- Atomic claim: one conditional `UPDATE` on one WAL connection, checked by
  `rowcount`. The old three-connection sequence let two dispatches claim one
  drone.
- The queue drains itself. Nothing had ever started a `QUEUED` job.
- `recover_orphaned_jobs()` at startup, so a crash mid-mission does not
  permanently consume a drone.

### P2 — Perception contracts

- `perception/vision_bridge.py` emits **all** detections. The old bridge sent
  `corners[0][0]` — the first marker only — so disambiguation was
  unimplementable with three pads in view.
- `decide_action()` extracted as a pure function with exactly the signature
  `tests/test_avoider_node.py` had been written against for months. `ESCALATE` is
  emitted at last, level-triggered so a dropped datagram cannot lose it.
- `config.MIN_VALID_RANGE_M` masks propeller returns at ≈0.29 m, which had read
  as a permanent unclearable wall.
- `drone_agent/udp_receiver.py` — **the missing writer**. `last_lidar_ts` and
  `last_telemetry_ts` were read by three call sites and written by none, so
  navigation held position forever *and* the supervisor fired an emergency RTL at
  `t = 0`.
- All durations `time.monotonic()`.

### P3 — Mission core

- `DroneMission` replaces three module-level state dicts. Module globals are
  shared by every importer, which is what actually made a fleet impossible.
- **One setpoint publisher**, 20 Hz, for the mission's whole duration. Removes
  mode thrashing (six `start`/`stop` pairs per mission), frame mixing
  (`set_velocity_body` alternating with `set_velocity_ned` at 10 Hz inside one
  session), and F5's stalls — in one structural change.
- FSM-driven orchestrator; legs from `marker_models.LEG_ORDER`, so each leg wants
  its own marker instead of three hardcoded zeroes. A failed leg routes to
  `HOVER_AND_ALERT` or `ABORT` — the old code had no `else` and continued from an
  unknown altitude.
- Battery gate, supervisor, payload bay and obstacle prefetch all wired.
- **Supervisor fixes.** The RTL→land fallback was unreachable (the flag latched
  before the command was accepted, so a rejected RTL produced *no action at
  all*); the heartbeat check fired at `t = 0`; the battery check was commented
  out. A real PX4 geofence is now uploaded — it survives a Python crash.
- Obstacle reporting was dead code: `if action != last_action: last_action =
  action` followed by `if action != last_action and …`.
- Acceleration limiting replaces tick blending.
- `sim/models/cargo_box/` — `PayloadBay` had been addressing an entity that
  existed nowhere on this machine.
- PX4 tuning applied from code and read back. Seven values were a comment asking
  the operator to type them into a shell.
- `grep -c "TARGET_ALT\s*=" drone_web/drone_logic.py` → **0**.

### P4 — Precision landing

- Metric control (F4), EMA on the measurement rather than the command.
- Descent gated by a cone narrowing with altitude, with a rate schedule.
- `SEARCH_ALT_M` + 2 m pads (F3).
- `LandedState.ON_GROUND` + disarm for touchdown; deleted the per-tick telemetry
  subscription that opened a fresh MAVLink stream every 100 ms.
- The spiral holds altitude, spaces rings by 0.6 of the camera footprint (21
  waypoints, not 100+), and the timeout is hard-capped.
- `docs/CALIBRATION.md` + `scripts/calibrate_camera_signs.py`.

### P5 — Escalation, detour, memory

- The `TODO` at `navigation.py:293` is implemented. The old handler logged a
  warning and kept flying at the wall.
- Reports deduplicated within 10 m and placed where the obstacle actually is —
  the drone's position projected along its bearing — rather than at the drone.
- The persistent service is the one that runs; `from db import …` fixed to a
  package-relative import, `/stats` added.
- **Re-confirmation now raises confidence.** The old merge averaged, so
  re-reporting a 0.5 obstacle at 0.5 left it at 0.5 forever and the acceptance
  criterion "confidence > 0.5" was unreachable.

### P6 — Fleet

- Model-scoped topics (F7); `sim_topics.py` owns the derivation.
- `spawn_fleet.sh` rewritten: build **once** outside the loop (it used to run
  `make` inside, serialising N builds and racing on one build directory), one
  Gazebo server, PX4 instances standalone.
- `scripts/run_system.sh` starts one bridge, one avoider and one vision bridge
  **per drone**, headless, with logs on disk.
- Per-drone UDP ports, gRPC ports, MAVLink sysids and Gazebo entity names.
- A real fleet view: one row per drone, per-drone markers and trails, and a
  reason column.

### P7 — Docs and guardrails

- `docs/ARCHITECTURE.md`, `docs/CALIBRATION.md`, `docs/ACCEPTANCE.md`.
- `STATUS.md` rewritten from the code, with a "Verified by" column on every row.
- `make check`: lint, import smoke test from a neutral cwd, and four repository
  gates that each encode a regression this codebase had.
- Seven references to a nonexistent `HOW_TO_RUN` guide resolved.

### Corrections to the plan, found while implementing

Recorded because each contradicts something the plan asserted:

- **MAVSDK is 2.12.10, not 2.8.4, and it rejects `udpin://`** — `Unknown
  protocol` / `Invalid connection URL`. `mavsdk_server` then exits and
  `System.connect()` blocks forever in `channel_ready_future`, so a dispatch hung
  with the drone `BUSY`. Fixed by using `udp://` (the 2.x name) and bounding the
  gRPC handshake. This also explains how the old three-format connection loop
  appeared to work: its `connection_state()` loop broke unconditionally on the
  first iteration, reporting a connection it had never made.
- **`PX4_GZ_MODEL_NAME` does not rename a spawned model.** Setting it makes PX4
  *skip spawning* and attach to a pre-existing one, so the plan's P6.2 recipe
  would have produced no drones. PX4 names instances `${PX4_SIM_MODEL}_${i}`
  itself.
- **PX4's `gz_env.sh` unconditionally overwrites `PX4_GZ_WORLDS` and
  `PX4_GZ_MODELS`** on the non-standalone launch path, which would have silently
  discarded the vendored assets. Hence `PX4_GZ_STANDALONE=1` and starting the
  Gazebo server ourselves.
- **Averaging confidences cannot satisfy "confidence > 0.5"**, so the merge had
  to change semantics, not just move files.
- **The plan's P0 target of "12 passed" and "under 30 s"** — the suite is now 124
  tests in ~37 s. `scripts/test.sh -m "not slow"` is the sub-5-second subset.

### Not done

Ten flight procedures in `docs/ACCEPTANCE.md`, none run — they need a live PX4.
`scripts/sitl_landing_trial.py`, needed for the twenty-landing gate, is not
shipped. See STATUS.md § "Where the remaining work is".

---

## Earlier history

Before this release the project's history lived as an "R1–R6" narrative inside
docstrings and status tables, with no way to tell whether a numbered fix had
landed, been superseded, or only been described. Those annotations have been
removed from the code; the underlying commits remain in git.

| | |
| --- | --- |
| `70f7df3` | "Faulty version_fixes not implemented yet" |
| `1e9dcbb` | Restart from basic avoidance toward advanced features |
| `bdd33db` | Review |
| `2e41830` | Initial commit |
