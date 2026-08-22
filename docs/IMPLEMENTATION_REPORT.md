# Implementation report

What was changed, what state each thing is in, and what is left — with the reason
it is left.

This is the handover document. For the chronological narrative see
[CHANGELOG.md](../CHANGELOG.md); for behaviour-by-behaviour evidence see
[STATUS.md](../STATUS.md); for how to run any of it see
[RUN_GUIDE.md](../RUN_GUIDE.md).

> **Shareable versions.** This document is published as
> *Remediation Handover* → <https://claude.ai/code/artifact/c651dab8-d3af-4b06-a7f6-0da1d8364f72>
> and the run guide as
> *Dispatch Runbook* → <https://claude.ai/code/artifact/1a01203f-2d4b-4325-a4e1-79c3991b446a>.
> Both are private until shared from the page's share menu. The Markdown here
> remains the source of truth; update it first, then republish.

| | |
| --- | --- |
| **Branch** | `remediation` (7 commits; `main` untouched) |
| **Scope** | All eight phases of the remediation plan, P0 → P7 |
| **Diff** | 95 files: 51 added, 34 modified, 10 deleted · +14,601 / −5,641 |
| **Tests** | 12 → **126** passing (3 SITL-marked, excluded by default) · 37 s |
| **Gates** | 6 repository gates, each verified to catch its regression |
| **Verified in flight** | F1, F2, F7, preflight, LiDAR self-return mask, MAVLink URL |
| **Not verified** | 10 flight procedures — see [§ 5](#5-remaining-work) |

---

## 1. Executive summary

### The two blockers that started this

| | Was | Now |
| --- | --- | --- |
| `get_state_snapshot` `ImportError` at `fleet_dispatch/app.py:93` | Import sat inside `run_drone_task` *above* its `try:`, and the symbol did not exist. Every dispatch raised before entering the try, so `finally: complete_job(...)` never ran and the drone stayed `BUSY` **forever**. A 3-drone fleet emptied after 3 dispatches, each reported to the operator as successfully assigned. | Real function, imported at module scope. A missing symbol is now a startup crash. `try:` is the first statement in the task body. **9 tests** cover the lifecycle, including 50 rapid dispatch/fail cycles leaving no drone stranded. |
| 5-vs-4 argument mismatch on `execute_delivery` | `TypeError` swallowed by a broad `except`; job recorded `COMPLETED`. | `execute_delivery(job: DeliveryJob, drone_id: str) -> MissionResult`. A dataclass ends the whole class of bug. |

### Phase status

| Phase | Delivered | Bench-verified | Flight-verified |
| --- | --- | --- | --- |
| **P0** Freeze the baseline | ✅ | ✅ | ⚠️ partial (§1 of ACCEPTANCE) |
| **P1** Dispatch integrity | ✅ | ✅ full | n/a — needs no flight |
| **P2** Perception contracts | ✅ | ✅ full | ⚠️ partial (§2 on ground) |
| **P3** Mission core | ✅ | ✅ | ❌ §4 not run |
| **P4** Precision landing | ✅ | ✅ | ⚠️ acquisition confirmed; §7 statistics not run |
| **P5** Escalation → detour → memory | ✅ | ✅ full | ❌ §10 not run |
| **P6** Fleet | ✅ | ✅ full | ❌ §11 not run |
| **P7** Docs and guardrails | ✅ | ✅ full | ❌ §12 not run |

"Delivered" means the code exists, is wired to a call site, and is covered.
It does **not** mean flown. That distinction is the entire point of this
remediation: the previous STATUS.md marked nine features complete on the
strength of the code existing rather than running.

---

## 2. Changes by area

### 2.1 New subsystems

| Module | Lines | Why it exists |
| --- | --- | --- |
| `drone_agent/mission.py` | 560 | `DroneMission` — all per-drone state and the FSM-driven orchestrator. Replaces three **module-level dicts** that every importer shared, which is what actually made a fleet impossible. |
| `drone_agent/setpoint.py` | 300 | The single offboard publisher. Makes "the setpoint stream never gaps" a structural invariant instead of something every code path must remember. |
| `drone_agent/udp_receiver.py` | 260 | **The missing writer.** `last_lidar_ts` and `last_telemetry_ts` were read at three call sites and written by none. |
| `drone_agent/geo.py` | 190 | One definition of the geodesy that had **seven** copies. |
| `drone_agent/contracts.py` | 200 | `DeliveryJob` / `MissionResult` / `JobStatus`. The dispatcher↔mission boundary. |
| `drone_agent/gz_client.py` | 200 | Async Gazebo service calls. Removes the synchronous `subprocess.run` from a 10 Hz async tick (finding F5). |
| `drone_agent/px4_params.py` | 180 | Applies the seven `MPC_*` values that were a comment asking the operator to type them into a shell — and reads them back. |
| `perception/vision_bridge.py` | 365 | Rewritten detector. Emits **all** detections; the old one sent only the first marker, making disambiguation unimplementable. |
| `sim_topics.py` | 120 | Model-scoped topic derivation, in one place (finding F7). |
| `scripts/preflight.py` | 460 | The check whose absence let F1 and F2 hide for months. |
| `scripts/repo_checks.py` | 400 | Six gates, each encoding a regression this codebase had. |

### 2.2 Rewritten

| File | Before → after | The substantive change |
| --- | --- | --- |
| `drone_web/drone_logic.py` | 528 → **90** lines | Was the code that actually flew, duplicating `drone_agent/`. Now the contract and nothing else. Ten shadowed tunables deleted; `grep -c "TARGET_ALT\s*=" → 0`. |
| `avoider_node.py` | 171 → 520 | `decide_action()` extracted as a pure function matching the signature a test had expected for months; `ESCALATE` finally emitted; thresholds read from config instead of shadowed; propeller self-hit mask. |
| `drone_agent/landing.py` | 443 → 640 | Metric control (F4), descent cone, `LandedState` touchdown, a search that holds altitude. |
| `drone_agent/navigation.py` | 418 → 520 | Fail-closed on stale sensors, acceleration limiting, the `ESCALATE → plan_detour` path, and the fix for obstacle reporting that was **unreachable code**. |
| `drone_agent/safety_supervisor.py` | 221 → 400 | Three defects that made it dangerous rather than inert — see [§ 3.2](#32-the-supervisor-was-dangerous-not-inert). |
| `fleet_dispatch/db.py` | 303 → 430 | Atomic claim, honest lifecycle, queue drain, crash recovery. |
| `fleet_dispatch/app.py` | 290 → 380 | Module-scope imports, no stub fallback, lifespan handler, per-drone WebSockets. |
| `fleet_dispatch/templates/index.html` | 111 → 300 | A real fleet view: per-drone rows, markers, trails, and a reason column. |
| `config.py` | 111 → 480 | The single source of tunables, with the F3 budget and F4 constants derived rather than typed. |
| `world/spawn_fleet.sh` | 80 → 230 | Builds **once** outside the loop; one Gazebo server; PX4 standalone. |
| `test_connection_now.py` | 60 → 190 | It used `udpin://` (which this MAVSDK rejects) and its wait loop broke unconditionally after one sample. A diagnostic that lies is worse than none. |

### 2.3 Deleted, with survivors named

Duplication is the mechanism by which a fix gets applied to the copy that is not
running.

| Deleted | Survivor | Verified before deleting |
| --- | --- | --- |
| `drone_web/app.py` | `fleet_dispatch/app.py` | Second dispatcher; the canonical one already had the error handling |
| `obstacle_memory_service.py` (root) | `obstacle_memory_service/` | In-memory dict, private constants, ignored `OBSTACLE_MERGE_RADIUS_M` — and `RUN_GUIDE.md` launched **this** one |
| `working code reference/` (3 files) | git history | One byte-identical; one differing by a trailing newline; one **438 lines of commented-out dead code** |
| `RUN_SYSTEM.sh` | `scripts/run_system.sh` | Started one bridge and one avoider — a single-drone launcher |
| `vision_bridge.py` (root) | `perception/vision_bridge.py` | `perception/` was an empty placeholder |

### 2.4 Simulation assets, vendored (finding F2)

17 new files under `sim/`. The camera and LiDAR were an **uncommitted working-tree
edit inside the `PX4-Autopilot/Tools/simulation/gz` git submodule**. Confirmed on
this host before acting:

```
$ git -C ~/PX4-Autopilot/Tools/simulation/gz status --short
 M models/x500_base/model.sdf
 M worlds/default.sdf
```

A single `git submodule update --force` would have deleted both sensors, and
nothing anywhere checked for them — the drone would arm, take off, and fly with
`/camera/image` and `/lidar/scan` simply absent.

| Asset | Purpose |
| --- | --- |
| `sim/models/x500_delivery/` | Stock x500 + both sensors, **no `<topic>`** so Gazebo scopes them per instance |
| `sim/worlds/delivery.sdf` | World origin + `great_wall`, **without** the undecodable `landing_pad` |
| `sim/models/pad_0\|1\|2/` | 2.0 m pads carrying the decodable `arucotag_N` textures |
| `sim/models/cargo_box/` | The parcel `PayloadBay` had been addressing — it existed **nowhere** on this machine |
| `sim/env.sh` | Path setup, with the load-bearing ordering documented in the file |
| `sim/patches/` | The original submodule edits, recorded, with a README on why not to re-apply them |

### 2.5 Tests

12 → 126. The old suite reported **nothing at all**, for two independent reasons:
ROS Humble's `launch_testing` plugins abort pytest 9 before collection, and
`test_marker_disambiguation.py:80` called `input()`, which never returns without
a TTY.

| File | Tests | Covers |
| --- | --- | --- |
| `test_mission_core.py` | 30 | Per-drone isolation, setpoint stream continuity, leg wiring, PX4 params |
| `test_landing_control.py` | 23 | Altitude-invariant gain, descent cone, marker budget, sign convention |
| `test_detour_memory.py` | 18 | Report placement, dedupe, persistence, confidence merge |
| `test_perception_contracts.py` | 16 | Replay state machine, self-hit masking, both schemas |
| `test_fleet_isolation.py` | 16 | Every shared resource proven per-drone |
| `test_dispatch_lifecycle.py` | 9 | Atomic claim, honest lifecycle, no stranded drones |
| `test_detour.py` · `test_mission_fsm.py` · `test_landing.py` | 10 | Pre-existing, still passing |
| `test_dispatch_endtoend.py` | 3 | Real HTTP dispatch with no autopilot |
| `test_avoider_node.py` | 1 | Pre-existing, **byte-identical** — see [§ 6](#6-one-file-deliberately-untouched) |

### 2.6 Documentation

| Document | Purpose |
| --- | --- |
| [docs/ARCHITECTURE.md](ARCHITECTURE.md) | Contract per hop, both message schemas, FSM diagram, the offboard invariant, the marker budget |
| [docs/CALIBRATION.md](CALIBRATION.md) | Camera intrinsics, the sign derivation, the marker budget, the battery model |
| [docs/ACCEPTANCE.md](ACCEPTANCE.md) | Every criterion with its verification named; 12 flight procedures written out |
| [docs/REMEDIATION_PLAN.md](REMEDIATION_PLAN.md) | The plan, recovered and kept **as written** |
| [STATUS.md](../STATUS.md) | Rewritten from the code, "Verified by" column on every row |
| [RUN_GUIDE.md](../RUN_GUIDE.md) | Operator guide |
| [CHANGELOG.md](../CHANGELOG.md) | One history, replacing the R1–R6 narrative |

---

## 3. The seven findings

All seven were **confirmed on this machine** before any code was written.

| | Finding | Confirmation | Fix | State |
| --- | --- | --- | --- | --- |
| **F1** | `model://arucotag` has no quiet zone and decodes in **zero** of OpenCV's 27 dictionaries. Spawned for all three pads, so `locked` never became true. | Ran the detector over all 27 dictionaries: `NOT DECODABLE`. With a 40 px white border: `DICT_4X4_50 → [0]`. | Vendored `pad_0/1/2` with decodable textures | ✅ **verified in flight** |
| **F2** | Sensors were an uncommitted submodule edit. | `git status` in the submodule | Vendored into `sim/` | ✅ **verified in flight** |
| **F3** | A 0.5 m marker spans **13.9 px** at 10 m; a 4×4 tag needs 25–30. | Reproduced the arithmetic exactly | 2 m pads **and** `SEARCH_ALT_M = 6` | ✅ + measured correction, [§ 4.5](#45-the-f3-budget-was-20-optimistic) |
| **F4** | Effective gain `K_p·fx/h` rose from 0.42 s⁻¹ at 10 m to **4.16 s⁻¹ at 1 m** — oscillation exactly at touchdown. | Reproduced | Convert to metres first, then a fixed metric gain | ✅ bench-proven; flight statistics pending |
| **F5** | `subprocess.run` in a 10 Hz async tick froze the event loop including the setpoint publisher. | Read the call site | Dedicated publisher + async subprocess | ✅ + enforced by a gate |
| **F6** | The sign convention was *correct* — and trivially "fixable" into a crash. | Re-derived from the mount pose | Derived, unit-tested, plus a confirmation script | ⚠️ script written, **not run** |
| **F7** | Absolute topics meant N drones shared two feeds. | Read the SDF | No `<topic>`; Gazebo scopes per instance | ✅ **verified in flight** |

### 3.1 Obstacle reporting was unreachable code

Worth calling out because it explains why fleet obstacle memory was always
empty:

```python
if action != last_action:
    last_action = action            # assigned here
if action != last_action and action in ("DODGE_LEFT", "DODGE_RIGHT"):
    report_obstacle(...)            # so this is never true
```

The second condition tested a variable the first had just made equal. **Every
dodge went unreported.** Fixed by capturing the previous action before
reassigning; report tasks now also hold references and log their exceptions, an
unreferenced task being collectable mid-flight.

### 3.2 The supervisor was dangerous, not inert

Three defects, each of which made it worse than absent:

| Defect | Consequence |
| --- | --- |
| `_trigger_emergency_rtl` set `_emergency_triggered = True` **before** the command was accepted, and `_trigger_emergency_land` began `if self._emergency_triggered: return` | An RTL failure produced **no action at all**, silently, at the moment the drone most needed one |
| The heartbeat check computed `time.time() - 0` | Emergency RTL on the **first tick**, before telemetry could arrive |
| The battery check was commented out; the geofence was Python-only | No battery protection; no protection at all if the process died |

All three verified fixed by test: a rejected RTL now issues `['rtl', 'land']`
where the old code issued `['rtl']` alone.

---

## 4. Corrections to the plan

Recorded because each contradicts something the plan asserted. The plan is kept
as written; editing it to match the outcome would destroy the record of assumed
versus measured.

### 4.1 MAVSDK is 2.12.10, and it rejects `udpin://`

The plan recorded 2.8.4 and treated `udpin://0.0.0.0:14540` as correct.

```
$ mavsdk_server -p 50061 "udpin://0.0.0.0:14540"
Warn  Unknown protocol (cli_arg.cpp:71)
Error Connection failed: Invalid connection URL
Failed to start, exiting...
```

`mavsdk_server` exits, and `System.connect()` then ends in
`aiogrpc.channel_ready_future()` — which waits **forever**. So a dispatch hung
indefinitely with the drone `BUSY`, which is the exact failure P1 exists to
eliminate.

Two fixes: `config.MAVSDK_URL_SCHEME = "udp"` (the 2.x name; `udpin` is 3.x), and
the gRPC handshake now has its own timeout.

**This also explains how the old three-format connection loop appeared to work.**
Its wait was:

```python
async for state in drone.core.connection_state():
    if state.is_connected: ...
    break                    # unconditional
```

It exited after one sample whether or not a heartbeat arrived — reporting a
connection it had never made.

### 4.2 `PX4_GZ_MODEL_NAME` does the opposite of what P6.2 assumed

The plan called for `PX4_GZ_MODEL_NAME=drone_$i`. In this PX4 version that
variable selects a **different launch mode**: `px4-rc.gzsim`'s
`elif [ -n "${PX4_GZ_MODEL_NAME}" ]` branch **skips spawning** and attaches to a
pre-existing model. Following the recipe would have produced no drones at all.
PX4 names instances `${PX4_SIM_MODEL}_${i}` itself.

### 4.3 PX4's `gz_env.sh` clobbers the vendored asset paths

`px4-rc.gzsim` re-sources PX4's own `gz_env.sh` on the non-standalone path, and
that file unconditionally overwrites `PX4_GZ_WORLDS` and `PX4_GZ_MODELS` back to
the PX4 source tree. The plan's `sim/env.sh` exports would have been silently
discarded and the drone would have flown blind. Hence `PX4_GZ_STANDALONE=1` and
starting the Gazebo server ourselves.

### 4.4 Averaging confidences cannot satisfy the P5 criterion

The acceptance criterion is "confidence > 0.5" after re-confirmation. The
existing merge computed `(existing + new) / 2`, so re-reporting a 0.5 obstacle at
0.5 left it at **0.5 forever**. Meeting the criterion required changing the merge
semantics, not just moving files: a probabilistic OR
(`p = p_old + p_new·(1 − p_old)`) gives 0.5 → 0.75 → 0.875, asymptoting to 1.0.

### 4.5 The F3 budget was 20% optimistic

Found by measurement, not reasoning. In flight the bridge consistently reported
~12% fewer pixels than predicted (88.6 px at 5.51 m against a 100.6 px
prediction).

Cause: the budget is written in terms of the pad **plane**, but the decoder only
sees the black **marker**, and these textures carry a white quiet zone — which is
the entire reason they decode at all. Measured on the shipped texture: a 199 px
marker in a 250 px image, so **79.6%**.

**This strengthens F3.** On plane widths the original 0.5 m pad looked merely
"marginal at 5 m" (27.7 px). Marker-corrected it is **22.1 px at 5 m — below the
threshold at every altitude the mission would have searched from**:

| Altitude | Plane | Decodable marker | 0.5 m pad, corrected |
| --- | --- | --- | --- |
| 10 m | 55.4 px | **44.1 px** | 11.0 px ❌ |
| 6 m | 92.4 px | **73.5 px** | 18.4 px ❌ |
| 3 m | 184.8 px | **147.1 px** | 36.8 px ✅ |

`config.decodable_px_at_altitude()` applies the measured factor, and two tests
pin it — one re-measures the ratio from the shipped textures, one pins the
in-flight observation.

### 4.6 The suite is 37 s, not "under 30 s"

P0's criterion assumed 12 tests. There are now 126, including two end-to-end
dispatch tests that wait out real connection probes. `scripts/test.sh -m "not
slow"` is the sub-5-second subset.

---

## 5. Remaining work

Nothing here is blocked on code. Every item needs a **live PX4 flying for
minutes at a time**, which is a different kind of activity from writing and
testing code — and running one of these procedures badly, or claiming it without
running it, is precisely the failure mode this remediation was written to fix.

### 5.1 Flight procedures — not run

Full commands and pass conditions in [docs/ACCEPTANCE.md](ACCEPTANCE.md).
Priority order:

| # | Procedure | Why it is not done | What it needs | Est. |
| --- | --- | --- | --- | --- |
| 1 | **§ 4** Three-leg mission → FSM `DONE` | Needs a complete flight (~3–5 min airborne) plus a PX4 console watched for OFFBOARD rejections | A live sim and an operator | 30 min |
| 2 | **§ 8** Confirm camera signs | Needs the drone hovering over a pad; `scripts/calibrate_camera_signs.py` is written and ready | A live sim | 10 min |
| 3 | **§ 2** 60 s hover, no self-returns | Only the **ground** case was verified; the propeller mask matters most with props spinning | A live sim | 5 min |
| 4 | **§ 3** Fail-closed on a killed avoider | Needs a mid-flight process kill | A live sim | 10 min |
| 5 | **§ 5** Cargo tracks the airframe | Purely observational — watch the Gazebo window during § 4 | Runs alongside § 4 | 0 |
| 6 | **§ 6** Supervisor abort mid-flight | Needs an airborne drone to abort | A live sim | 10 min |
| 7 | **§ 7** 20 landings, ≥19 within 0.5 m | Statistical; also needs a harness that is **not shipped** — see 5.2 | Sim + new script | 3–4 h |
| 8 | **§ 10** Detour, two runs | The most interesting result in the system: run 2 should record **zero** dodges | A live sim | 45 min |
| 9 | **§ 11** Three-drone fleet | Needs 3 PX4 instances + 3 Gazebo drones concurrently; ~4 CPU cores is tight | A live sim, ideally more cores | 1 h |
| 10 | **§ 12** New operator flies from the guide alone | Needs a person who has not seen the system | A second human | 1 h |

Partial credit already banked, logged in ACCEPTANCE § 9: **§ 1 preflight PASS**
(6/6 against a live world), **§ 2 on the ground PASS**, and a **live ArUco
acquisition PASS** — marker ID 0 decoded from the real camera at 6 m, which is
the behaviour that had never once worked.

### 5.2 `scripts/sitl_landing_trial.py` — deliberately not shipped

The 20-landing gate needs a harness that repositions the drone, runs one landing,
measures the touchdown error, and appends a row.

**Left out on purpose.** Writing it without being able to run it would produce an
untested test harness — and a broken harness reporting "19/20 PASS" is strictly
worse than no harness, because it manufactures exactly the false confidence this
whole remediation exists to remove. ACCEPTANCE § 7 says so explicitly and notes
it should drive `landing.execute_precision_landing` directly rather than through
the dispatcher, so a failure is attributable to the controller.

### 5.3 Known limitations — accepted, not oversights

| Limitation | Why accepted | What would change it |
| --- | --- | --- |
| **Battery model uncalibrated** — `BATTERY_ENERGY_PER_M_PCT = 0.01` | A placeholder from typical multirotor figures, not this airframe. Cannot be measured without flying a known distance. `BATTERY_RESERVE_MARGIN_PCT = 15` covers the optimism. | The 6-step procedure in CALIBRATION § 5 |
| **Flat-earth geodesy** — ≈2 m error at 10 km | Irrelevant inside the 500 m geofence, where error is under 10 cm — far below GPS noise | Replace with haversine **before** widening the geofence past a few km. The failure mode is a slow drift, so it will not announce itself. |
| **Fleet capped at 10** | PX4's `px4-rc.mavlink` maps instances >9 onto port 14549. A hard external constraint, enforced by `spawn_fleet.sh`. | Nothing short of patching PX4 |
| **`solvePnP` unused** | `compute_metric_offset()` exists and is not called. The altitude-based conversion delivers most of its value with no calibration rig; full pose estimation adds marker tilt, which nothing needs yet. | A requirement for landing on a sloped pad |
| **SITL ≠ hardware** | Every threshold is in `config.py` | Recalibrate `fx` and pad size per CALIBRATION § 1 and § 3 |

### 5.4 What will probably break first

Roughly a third of `drone_agent/` was sound and unit-tested before this work and
had simply **never been executed against a running autopilot**. Expect the first
full mission to surface two or three defects of the same kind as F5 — bugs that
only exist once the code actually runs. That is planned for, not a setback.

The most likely candidates, in order:

1. **Offboard handover after takeoff.** `arm_and_takeoff` uses PX4's auto-takeoff
   and then enters offboard from the resulting hover. That transition is the least
   exercised code path in the mission.
2. **Descent cone tuning.** `DESCENT_CONE_SLOPE = 0.35` is a reasoned choice, not
   a measured one. Too tight and the drone hovers re-centring; too loose and it
   lands off-pad.
3. **Detour waypoint spacing.** `plan_detour` returns geometric tangent points
   with no minimum leg length, so in tight geometry two waypoints could land
   within `ARRIVAL_M` of each other.

---

## 6. One file deliberately untouched

`tests/test_avoider_node.py` is **byte-identical** to before this work — md5
`f72fc1570fcffaaeac0a7163dd29dc87`, verified after every formatting pass.

It is excluded from ruff's `check` **and** `format` via `per-file-ignores`, and
the one warning it raises is filtered in `pytest.ini` rather than fixed in the
file.

The reason: that test was written months before the API it tests existed. It is
the one artefact in the repository proving `decide_action()` was built to match a
pre-existing contract, rather than the contract being retrofitted to whatever the
implementation happened to do. Any automated pass over it — even a whitespace
fix — weakens that claim.

---

## 7. Guardrails against regression

Six gates in `make check`. Each encodes a regression this codebase **actually
had**, and each was verified by deliberately reintroducing an instance and
confirming the gate fails.

| Gate | Catches | The regression it encodes |
| --- | --- | --- |
| `import-check` | A module that imports only from one working directory | **The original blocker.** This is the exact check that would have caught `get_state_snapshot` at commit time. |
| `tunables` | A threshold re-declared outside `config.py` | Ten were shadowed in `drone_logic.py`, so editing `config.py` had no effect on flight |
| `docs` | A dangling `.md` reference | Seven pointed at a `HOW_TO_RUN` guide that never existed, two inside runtime error strings an operator reads mid-incident |
| `schema` | Sim assets drifting from `config.py` | Camera intrinsics, pad size, world origin vs. map centre, and the F3 budget |
| `blocking` | `subprocess.run` / `cv2.imshow` / `time.sleep` on the flight path | Finding F5. A per-line `# blocking-ok:` opt-out demands a written justification — a whole-file exemption is how the next real one gets in |
| `duplicates` | A shared formula gaining a second definition | `get_distance_m` had **seven** copies |
| `prints` | `print()` in flight code | `drone_logic.py` printed ~40 times from inside the flight loop; with three drones that is unreadable and nothing is parseable afterwards |

Plus `lint` (ruff check + format, now failing rather than `|| true`) and the full
suite. GitHub Actions runs all of it on push — deliberately **without** ROS,
Gazebo or PX4, because a CI job needing a GPU and a 40-minute build is a CI job
that gets disabled.

---

## 8. Verification ledger

What "verified" means for each claim in this report.

| Class | Count | Meaning |
| --- | --- | --- |
| **Automated test** | 126 | Runs in `make check`, no simulator |
| **Repository gate** | 7 | Runs in `make check`; each proven to catch its regression |
| **Live simulator** | 8 | Observed against a running PX4 + Gazebo on 23 Aug 2026, logged in ACCEPTANCE § 9 |
| **Flight procedure** | 10 | Written out with commands and pass conditions; **none run** |

Live-verified, for the record:

| What | Evidence |
| --- | --- |
| F2 — sensors independent of the submodule | Drone spawned as `x500_delivery_0` from `sim/models/` with both sensors |
| F7 — topics model-scoped | Live names matched `sim_topics` character for character |
| F1 — the pad decodes | Marker ID 0 **and nothing else** across 46 frames at 6 m |
| Preflight | PASS 6/6, live camera 320×240 / fx 277.2, live scan 360 samples / range_max 12.0 |
| No self-returns | 81 datagrams in 8 s at 10.0 Hz, `eff_front_m` constant at 15.0 m |
| MAVLink URL correction | `udp://0.0.0.0:14540` connected and reported health |
| Per-drone bridge remap | `/drone_0/scan` at 10.03 Hz |
| Both JSON contracts | Matched ARCHITECTURE § 2 and § 3 field for field |
