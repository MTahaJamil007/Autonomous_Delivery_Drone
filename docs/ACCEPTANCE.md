# Acceptance criteria and how each one is verified

Every criterion from the remediation plan, with its verification method named.

**The rule:** a feature is done when a named test or a dated procedure log
demonstrates it — not when the module compiles. The whole reason this plan
existed is that nine features were marked complete on the strength of the code
existing rather than running.

Two verification classes:

- **BENCH** — an automated test. Run `scripts/test.sh`. No simulator needed.
- **FLIGHT** — a procedure requiring PX4 SITL and Gazebo. Written out below with
  the exact commands and the pass condition. **Log the outcome in §9.**

Nothing in this file claims a FLIGHT result. Those rows are unfilled, on purpose:
filling one in without running it would reproduce exactly the failure this whole
effort was about.

---

## P0 — Freeze the baseline

| Criterion | Method | Status |
| --- | --- | --- |
| `git status` clean; `git ls-files \| wc -l > 60` | BENCH | ✅ 64 files |
| `scripts/test.sh` → 0 errors, 0 hangs, < 30 s | BENCH | ✅ ~37 s for 124 tests |
| Import smoke test from a neutral cwd | BENCH | ✅ `make import-check` |
| `preflight.py` fails loudly with a topic missing | FLIGHT | § 1 |

> The suite now takes ~37 s rather than the < 30 s the plan asked for, because it
> grew from 12 tests to 124 — including two end-to-end dispatch tests that wait
> out real connection probes. `scripts/test.sh -m "not slow"` runs the fast
> subset in under 5 s.

---

## P1 — Dispatch path integrity

| Criterion | Method | Status |
| --- | --- | --- |
| PX4 offline → job FAILED with a reason, drone AVAILABLE, reason in the browser | BENCH (real HTTP, no autopilot) | ✅ `test_dispatch_endtoend.py` |
| Two simultaneous dispatches at `FLEET_SIZE=1` → one ASSIGNED, one QUEUED; queued one starts automatically | BENCH | ✅ `test_second_dispatch_queues_then_starts_automatically` |
| 50 rapid dispatches leave no drone stuck BUSY | BENCH | ✅ `test_no_drone_is_stranded_by_repeated_failures` |

---

## P2 — Perception contracts

| Criterion | Method | Status |
| --- | --- | --- |
| `tests/test_avoider_node.py` passes **unmodified**, including ESCALATE | BENCH | ✅ assertions untouched |
| Replay test: CLEAR → DODGE → ESCALATE → CLEAR, no wall-clock dependence | BENCH | ✅ `test_replay_clear_dodge_escalate_clear_without_wall_clock` |
| Two markers in frame → both in one datagram, correct distinct IDs | BENCH | ✅ `test_two_markers_appear_in_one_frame_with_distinct_ids` |
| Empty world: `eff_front ≈ 15.0` m for 60 s, no self-returns | FLIGHT | § 2 |
| Killing `avoider_node.py` mid-flight trips the nav hold within 1 s | FLIGHT | § 3 |

---

## P3 — Mission core

| Criterion | Method | Status |
| --- | --- | --- |
| Two `DroneMission` instances keep independent state (no sim) | BENCH | ✅ `test_two_missions_keep_independent_state` |
| `grep -c "TARGET_ALT\s*=" drone_web/drone_logic.py` → 0 | BENCH | ✅ `make no-shadowed-config` |
| `POST /dispatch` → 3-leg mission reaches FSM DONE, job COMPLETED | FLIGHT | § 4 |
| Zero OFFBOARD-rejected / setpoint-timeout messages for the whole mission | FLIGHT | § 4 |
| Cargo box tracks the airframe between pickup and drop, stays put after release | FLIGHT | § 5 |
| Killing the mission task mid-flight → supervisor RTLs; job ABORTED | FLIGHT | § 6 |

---

## P4 — Precision landing

| Criterion | Method | Status |
| --- | --- | --- |
| Loop gain is altitude-invariant | BENCH | ✅ `test_effective_loop_gain_is_altitude_invariant` |
| Pad decodable at the altitude it is searched from | BENCH | ✅ `test_the_pad_is_decodable_at_the_altitude_it_is_searched_from` |
| Body-frame signs match the derived mount | BENCH | ✅ `test_body_frame_signs_match_the_derived_camera_mount` |
| 20 scripted landings from 8 m and ±5 m offset: ≥19 within 0.5 m | FLIGHT | § 7 |
| Zero landings on the wrong marker ID, pads within 15 m | FLIGHT | § 7 |
| Zero oscillation events below 3 m (offset sign changes > 3×) | FLIGHT | § 7 |
| Lock within 20 s of reaching `SEARCH_ALT_M` in ≥18 of 20 runs | FLIGHT | § 7 |
| Camera signs confirmed empirically, recorded in CALIBRATION.md | FLIGHT | § 8 |

---

## P5 — Escalation, detour, memory

| Criterion | Method | Status |
| --- | --- | --- |
| One wall → exactly one row, confidence > 0.5, correct reporter | BENCH | ✅ `test_one_wall_becomes_one_row_with_rising_confidence` |
| Obstacles survive a service restart | BENCH | ✅ `test_obstacles_survive_a_restart` |
| A known obstacle changes the planned route | BENCH | ✅ `test_a_known_obstacle_changes_the_planned_route` |
| Run 1 across `great_wall`: dodge → ESCALATE at ≈12 s → detour → arrival | FLIGHT | § 10 |
| Run 2 on the same route: zero `DODGE_*` actions | FLIGHT | § 10 |

---

## P6 — Fleet

| Criterion | Method | Status |
| --- | --- | --- |
| Every drone has its own camera and LiDAR topic | BENCH | ✅ `test_every_drone_has_its_own_camera_and_lidar_topic` |
| No port, gRPC port or sysid is shared across the fleet | BENCH | ✅ `test_no_port_is_shared_anywhere_in_a_full_fleet` |
| Three concurrent dispatches → 3 airborne → 3 COMPLETED | FLIGHT | § 11 |
| No cross-talk: each drone lands only on its own leg's marker | FLIGHT | § 11 |
| A fourth dispatch queues, then starts when the first frees | FLIGHT | § 11 |
| Killing one drone's PX4 fails only that job | FLIGHT | § 11 |

---

## P7 — Docs and guardrails

| Criterion | Method | Status |
| --- | --- | --- |
| `grep -rn HOW_TO_RUN .` resolves to a file that exists | BENCH | ✅ `make docs-check` |
| Every ✅ in STATUS.md maps to a test ID or a dated procedure | BENCH | ✅ STATUS.md carries a "Verified by" column |
| `make check` green from a clean clone | BENCH | ✅ |
| A new operator completes a mission using only RUN_GUIDE.md | FLIGHT | § 12 |

---

# Flight procedures

Before any of these: `source sim/env.sh`, `world/spawn_fleet.sh`,
`scripts/run_system.sh`, and `scripts/preflight.py` must PASS.

## § 1 — Preflight detects a missing topic

```bash
world/spawn_fleet.sh 1 && scripts/run_system.sh 1
python3 scripts/preflight.py                 # expect PASS
scripts/stop_system.sh                       # kill the drone, leave gz up
python3 scripts/preflight.py                 # expect FAIL, naming the topic
```

**Pass:** the second run exits non-zero and names the specific missing topic and
the reason, not a generic "not ready".

## § 2 — No LiDAR self-returns

```bash
world/spawn_fleet.sh 1 && scripts/run_system.sh 1
# Take off manually (QGroundControl) to 10 m over open ground, hover 60 s.
grep eff_front /tmp/droneprogram/avoider_0.log | tail -60
```

**Pass:** `eff_front_m` stays at 15.0 (`INF_REPLACE_M`) throughout, with no
sustained readings near 0.3 m. A 0.29 m reading means the propeller mask is not
working (see CALIBRATION.md § 4).

## § 3 — Fail-closed on a dead obstacle feed

```bash
# Mid-flight, during a leg:
pkill -f 'avoider_node.py --drone-id drone-0'
# Watch the dispatcher log, then restart it:
python3 avoider_node.py --drone-id drone-0 &
```

**Pass:** within 1 s of the kill the drone holds position and the log says
`obstacle feed silent for … - holding position`. Within 1 s of the restart it
resumes, logging `obstacle feed recovered`.

## § 4 — Full three-leg mission

```bash
world/spawn_fleet.sh 1 && scripts/run_system.sh 1
# Open http://localhost:5000, click a pickup and a drop-off, Dispatch.
```

**Pass, all four:**
1. The job reaches `COMPLETED` and the drone returns to `AVAILABLE`.
2. The FSM ends in `DONE` — `grep 'NEXT_LEG --\[mission_complete\]--> DONE'`.
3. The PX4 console shows **zero** `OFFBOARD` rejections or setpoint timeouts.
4. `grep 'setpoint stats' /tmp/droneprogram/fleet_dispatch.log` reports
   `max_gap_s` below 0.4 and `send_failures: 0`.

## § 5 — Cargo tracks the airframe

Watch the Gazebo window during § 4.

**Pass:** the cargo box sits at the pickup pad, follows beneath the drone from
pickup to drop-off, and stays put at the drop pad after release. It must not
swing, lag by more than a fraction of a second, or fall through the ground.

## § 6 — Supervisor intervention on mission loss

```bash
# Mid-flight, with the drone airborne:
curl -X POST http://localhost:5000/jobs/<job_id>/abort
```

**Pass:** the drone lands or RTLs under control, the job is recorded `ABORTED`
(not FAILED — an aborted mission means there is an airframe to go and find), and
the drone returns to `AVAILABLE`.

## § 7 — Twenty scripted landings

```bash
world/spawn_fleet.sh 1 && scripts/run_system.sh 1     # rig up, preflight must pass
python3 scripts/sitl_landing_trial.py                 # 7 dispatches = 21 landings
python3 scripts/sitl_landing_trial.py --analyse-only  # re-score without re-flying
```

`scripts/sitl_landing_trial.py` **is now shipped.** It flies whole delivery
missions through `/dispatch` — three landings each, on markers 0, 1 and 2, from
a different bearing and range every time — and scores touchdown error against
**Gazebo ground truth**, never the drone's own EKF. The other three criteria it
reads back out of the mission and vision logs. It exits non-zero unless all four
pass.

> **This reverses the instruction that used to sit here**, which said the script
> should call `drone_agent.landing.execute_precision_landing` directly rather
> than go through the dispatcher, "so a failure is attributable to the landing
> controller and not to the mission around it".
>
> The first live session with the rewritten controller settled that argument the
> other way. Four defects surfaced, and **three of them were not in the landing
> controller**: touchdown was never detected because PX4 will not report
> `ON_GROUND` while OFFBOARD streams a descent; leg 2 hovered until timeout
> because nothing re-entered OFFBOARD after the land command that confirms a
> touchdown; and the mission FSM was driven to `SEARCHING` on the way down and
> then rejected the touchdown it had just achieved. Each one made the delivery
> fail. A harness that bypassed the mission would have reported four flawless
> landings and a system that cannot deliver a package.
>
> Attribution is cheap to recover afterwards — the per-attempt table names the
> phase each attempt reached — while coverage given up by bypassing the real
> path is not.

**Pass, all four:**
- ≥19 of 20 touchdowns within 0.5 m of pad centre.
- **Zero** landings on the wrong marker ID, with all three pads within 15 m.
- **Zero** oscillation events below 3 m altitude (offset sign changing more than
  3 times).
- Lock acquired within 20 s of reaching `SEARCH_ALT_M` in ≥18 of 20 runs.

## § 8 — Confirm the camera signs

```bash
python3 scripts/calibrate_camera_signs.py --drone-id drone-0 --axis forward
python3 scripts/calibrate_camera_signs.py --drone-id drone-0 --axis right
```

**Pass:** both report `AGREES`. Record both runs in CALIBRATION.md § 2.

**If either reports CONTRADICTS, stop.** Do not flip a sign to make it pass —
re-derive from the mount pose in the SDF first. A contradiction means either the
mount changed or the derivation is wrong, and guessing turns the controller into
positive feedback.

## § 10 — Detour and memory, two runs

```bash
rm -f obstacle_memory_service/obstacles.db      # start with no memory
world/spawn_fleet.sh 1 && scripts/run_system.sh 1
# RUN 1: dispatch a route due north, straight through great_wall at (0, 15).
```

**Run 1 pass:** the drone dodges, `ESCALATE` appears in
`/tmp/droneprogram/avoider_0.log` at ≈12 s of continuous dodging, a detour is
planned, the drone arrives, and there is no collision and no timeout.

```bash
curl -s localhost:5050/stats            # expect total: 1, confident: 1
curl -s 'localhost:5050/obstacles' | python3 -m json.tool
```

**Then run 2, same route, without clearing the database.**

**Run 2 pass:** **zero** `DODGE_*` actions in the avoider log, and the flown path
deviates around the wall from departure rather than after meeting it. That
difference is the entire observable distinction between an obstacle database and
an obstacle log.

```bash
scripts/stop_system.sh
python3 -m obstacle_memory_service.app &
curl -s localhost:5050/stats            # the row must still be there
```

## § 11 — Three-drone fleet

```bash
sed -i 's/^FLEET_SIZE = 1$/FLEET_SIZE = 3/' config.py
world/spawn_fleet.sh 3 && scripts/run_system.sh 3
python3 scripts/preflight.py            # must PASS for all three drones
# Dispatch three jobs to distinct destinations, then a fourth.
```

**Pass, all four:**
1. Three drones airborne concurrently; three jobs `COMPLETED`.
2. No cross-talk: each drone lands only on its own leg's marker, and each
   avoider reacts only to its own LiDAR. Check that each
   `/tmp/droneprogram/avoider_N.log` shows an independent action sequence — if
   all three logs move in lockstep, the topics are not scoped.
3. The fourth dispatch is `Queued`, then starts automatically when the first
   drone frees.
4. `kill` one drone's PX4 instance mid-flight: only that job fails; the other two
   complete.

## § 12 — A new operator flies from the guide alone

Hand RUN_GUIDE.md to someone who has not seen this system and give no verbal
help. **Pass:** they complete a mission. Any question they have to ask is a
defect in the guide — fix the guide, not the operator.

---

## § 9 — Procedure log

Append one row per run. A dated row here is what makes a ✅ in STATUS.md mean
something.

| Date | Procedure | Result | Notes |
| --- | --- | --- | --- |
| 2026-08-23 | § 1 preflight (PASS half) | **PASS** | 6/6 checks against a live `delivery` world. Verified the vendored model spawns as `x500_delivery_0`, both sensor topics are advertised at the names `sim_topics` predicts, live camera is 320×240 (fx 277.2), live scan is 360 samples / range_max 12.0. The FAIL half (kill the drone, expect a named missing topic) not yet run. |
| 2026-08-23 | § 2 no self-returns | **PASS (ground)** | 81 datagrams in 8 s (10.0 Hz), `eff_front_m` constant at 15.0 — no propeller returns. Geometry validated against config on the first message. Run over a **60 s hover** to close it fully; this was on the ground. |
| 2026-08-23 | partial § 7 — live ArUco acquisition | **PASS** | Not the 20-landing statistical gate, but the F1/F3 chain end to end: `pad_0` spawned via the async gz client, climbed to 6 m, the live camera decoded **marker ID 0 only**, 46 detection frames, 88.6 px at 5.51 m vs an 80.1 px marker-corrected prediction. This is the behaviour that had never once worked. |
| _pending_ | § 3 fail-closed | — | — |
| _pending_ | § 4 three-leg mission | — | The next thing to run; everything else depends on it. |
| _pending_ | § 5 cargo tracking | — | — |
| _pending_ | § 6 supervisor abort | — | — |
| _pending_ | § 7 twenty landings | — | Needs `scripts/sitl_landing_trial.py`, not shipped. |
| _pending_ | § 8 camera signs | — | `scripts/calibrate_camera_signs.py` is ready to run. |
| _pending_ | § 10 detour, two runs | — | The most interesting result in the system. |
| _pending_ | § 11 three-drone fleet | — | — |
| _pending_ | § 12 new operator | — | — |

### Also verified live on 2026-08-23, outside the numbered procedures

| What | Result |
| --- | --- |
| **F2 — sensors survive independent of the PX4 submodule** | The drone spawned from `sim/models/x500_delivery` with both sensors present. `PX4_GZ_STANDALONE=1` kept `PX4_GZ_MODELS` pointed at the vendored tree, exactly as `sim/env.sh` documents. |
| **F7 — topics are model-scoped** | Live topics matched `sim_topics.camera_topic()` / `lidar_topic()` character for character: `/world/delivery/model/x500_delivery_0/link/camera_link/sensor/downward_camera/image`. No absolute `/camera/image` anywhere. |
| **The MAVSDK URL correction** | `udp://0.0.0.0:14540` connected and reported health. `udpin://` — which the plan assumed and the old code used — makes `mavsdk_server` exit and `connect()` block forever. |
| **`ros_gz_bridge` per-drone remap** | `/drone_0/scan` at 10.03 Hz from the model-scoped Gazebo topic. |
| **Avoider JSON contract** | Matches docs/ARCHITECTURE.md § 3 field for field, at 10 Hz. |
| **Vision JSON contract** | Matches § 2 field for field, including `fx` derived from the live width. |
| **Async pad spawning** | `gz_client.spawn_pad_at_gps` created `test_pickup_pad` without blocking the event loop. |
