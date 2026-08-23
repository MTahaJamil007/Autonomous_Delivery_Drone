# Codebase audit — what actually exists, and what is verified

Audited 2026-08-23 against `DroneProgram` @ `91abd5e` (branch `remediation`).
16,552 lines across 64 tracked source/doc files.

This document exists so that no research proposal in this folder claims as
"new contribution" something the repository already does. **Read this before
writing any novelty claim.**

---

## 1. Environment (measured, not assumed)

| Component | Version |
| --- | --- |
| PX4 | v1.17.0-alpha1-1225 |
| MAVSDK | 2.12.10 (`udp://`, *not* `udpin://`) |
| Gazebo | Harmonic |
| ROS 2 | Humble |
| Python | 3.10.12 |
| OpenCV | 4.13.0 |

No hardware airframe yet. Everything below is SITL.

---

## 2. Subsystem inventory

| Subsystem | File(s) | What it does | Research-relevant limitation |
| --- | --- | --- | --- |
| **Mission orchestration** | `drone_agent/mission.py` (716 L) | `DroneMission` class; per-drone state; 3-leg pickup→drop→home; module registry for telemetry snapshots | Legs are fixed and hardcoded in order; no re-planning of *which* leg to fly |
| **Mission FSM** | `drone_agent/mission_fsm.py` | 11 states, explicit transition table, `force()` for the safety layer, terminal states absorb events, transition history | Purely reactive event table; no cost or utility reasoning |
| **Offboard stream** | `drone_agent/setpoint.py` (325 L) | One 20 Hz task streaming `VelocityNedYaw`; mission code only mutates a dataclass; tracks `max_gap_s` | Velocity-only. No position/trajectory setpoints, no MPC |
| **Navigation** | `drone_agent/navigation.py` (523 L) | Bearing cruise with proportional slowdown, acceleration slew limit (`MAX_ACCEL_M_S2`), fail-closed on stale LiDAR, ESCALATE→detour hook, `ObstacleReporter` | Constant cruise altitude. No 3D avoidance. No wind. No dynamic obstacles |
| **Reactive avoidance** | `avoider_node.py` (523 L) | Pure `decide_action()` state machine on a 360-sample 2D LiDAR; front cone ±35°; hysteresis `SAFE=6.5 / CLEAR=9.0`; `MIN_LOCK_S=2`, `CLEAR_CONFIRM_S=2.5`; **`ESCALATE` after 12 s** of continuous dodging, level-triggered | Escalation threshold is a **fixed constant**. Dodge direction chosen once, greedily, by side median. No prediction, no cost |
| **Global planner** | `global_planner/detour.py` (243 L) | Geometric tangent-bypass around the nearest blocking circle; recursion depth 3; side chosen by counting nearby obstacles | Deterministic, no cost function, no risk, no uncertainty, no energy term. Ignores obstacle *confidence* entirely |
| **Obstacle memory** | `obstacle_memory_service/` (442 L) | FastAPI + aiosqlite. Position-hashed IDs; merge within 5 m; **probabilistic-OR confidence fusion** `p = p₁ + p₂(1−p₁)`; confidence-weighted position merge; **exponential decay τ = 14 days applied at query time**; bbox query | **Write-only for occupancy.** Nothing ever produces *negative* evidence. A false report is permanent for ~months. Decay is wall-clock, not observation-driven |
| **Perception** | `perception/vision_bridge.py` (358 L) | OpenCV ArUco `DICT_4X4_50`, subpixel corner refinement, **all** detections per frame, JSON/UDP, `fx` derived from live width | No detection confidence, no pose covariance, no `solvePnP` (exists but uncalled), no tilt, no blur/illumination model |
| **Precision landing** | `drone_agent/landing.py` (593 L) | Metric (altitude-normalised) P control `offset_m = err_px·h/fx`; EMA on the *measurement*; descent cone `0.35h + 0.15`; scheduled descent rate; spiral search with footprint-overlap ring spacing; `LandedState` touchdown + disarm confirm; stall watchdog; fail-closed on stale vision | Descent cone is a **hand-tuned heuristic**. No explicit abort/commit decision. Search is **open-loop** spiral — no belief, no information gain |
| **Detectability model** | `config.py` | `marker_px_at_altitude()`, `decodable_px_at_altitude()` with a **measured** quiet-zone factor 0.796; `MARKER_MIN_DECODE_PX = 25` | Used only as a *design-time* constant to pick `SEARCH_ALT_M = 6.0`. **Never used online** by any controller |
| **Safety supervisor** | `drone_agent/safety_supervisor.py` (337 L) | Independent 1 Hz task with direct autopilot authority: heartbeat, soft geofence, PX4-uploaded hard geofence, critical battery → LAND (not RTL), sensor-liveness advisory; RTL with reachable LAND fallback | Fixed thresholds. No reachability, no risk model, no forward simulation |
| **Battery gating** | `drone_agent/battery.py` | Pre-leg gate: `travel_pct = distance × 0.01 %/m` + 5% landing reserve + 15% margin | `BATTERY_ENERGY_PER_M_PCT` is an **explicit placeholder**. No payload mass, no wind, no altitude change, no hover cost |
| **Payload** | `drone_agent/payload.py` (222 L) | Kinematic cargo following at 10 Hz via async `gz` service calls; deliberately not a physical joint | Cosmetic. Mass is **not** modelled — payload does not affect dynamics or energy |
| **Fleet dispatch** | `fleet_dispatch/` (785 L) | FastAPI + SQLite; atomic `claim_drone_for_job` (single conditional UPDATE); honest job lifecycle; queue drain; per-drone WebSockets; Leaflet map | Coordination is **job-level only**. No spatial deconfliction, no shared planning, no inter-drone comms |
| **Simulation** | `sim/` | Vendored `x500_delivery` (downward 320×240 camera @10 Hz + 360-pt 2D LiDAR, 12 m range @10 Hz), `delivery.sdf` world, 2 m ArUco pads 0/1/2, cargo box | One static obstacle (`great_wall`, 30×1×20 m at 15 m N). No wind, no moving obstacles, no lighting variation, no clutter |
| **Fleet scaling** | `config.py`, `world/spawn_fleet.sh`, `sim_topics.py` | Model-scoped Gazebo topics; per-drone UDP/gRPC/sysid derivation; hard ceiling 10 | Currently `FLEET_SIZE = 1`; 3-drone concurrency structurally tested, not flown |

---

## 3. Verification status — the important distinction

The repository is unusually honest about this. `STATUS.md` enforces: *nothing is
marked ✅ without a call site and a named verification.*

### Bench-verified (126 automated tests, 3 SITL-marked, ~37 s)

Marker disambiguation, EMA filter, altitude-invariant loop gain, descent cone,
pad decodability budget, body-frame sign derivation, avoider state machine
replay (CLEAR→DODGE→ESCALATE→CLEAR with no wall-clock dependence), hysteresis
band, per-drone state isolation, setpoint-stream continuity, PX4 param
application, detour planning against known obstacles, obstacle dedupe /
placement / persistence / confidence merge, every shared resource being
per-drone (16 tests), atomic dispatch claim, 50 rapid dispatch cycles stranding
no drone.

Plus 7 repository gates (`make check`): import-from-neutral-cwd, no shadowed
config constants, no dangling doc links, sim-asset/config schema agreement,
no blocking calls on the flight path, no `print()` in flight code, no duplicate
shared formulas.

### Flight-verified — 2026-08-23 session only

| What | Result |
| --- | --- |
| Vendored model spawns with both sensors, independent of the PX4 submodule | PASS |
| Sensor topics are model-scoped and match `sim_topics` character-for-character | PASS |
| **Live ArUco acquisition**: climbed to 6 m, decoded marker ID 0 *and only 0* across 46 frames | PASS — the behaviour that had never once worked |
| Measured apparent size 88.6 px at 5.51 m vs 80.1 px marker-corrected prediction | PASS (+10.6% — model is *conservative*) |
| Preflight 6/6 including live camera geometry and scan geometry vs `config.py` | PASS |
| No LiDAR self-returns: 81 datagrams, 10.0 Hz, `eff_front_m` constant at 15.0 | PASS (on the ground; 60 s hover still owed) |
| `udp://` connects where `udpin://` hangs forever | PASS |
| Async pad spawning without blocking the event loop | PASS |

### NOT flight-verified — 10 procedures still open

| § | Procedure | Why it matters to research |
| --- | --- | --- |
| 3 | Fail-closed on a killed avoider | Baseline for any degradation study |
| **4** | **Full three-leg mission** | **Everything else depends on this** |
| 5 | Cargo tracks the airframe | Cosmetic; low research weight |
| 6 | Supervisor abort mid-flight | Baseline for safety experiments |
| **7** | **20 scripted landings, ≥19 within 0.5 m** | The landing baseline. `scripts/sitl_landing_trial.py` **is not shipped** |
| 8 | Camera sign confirmation | Must pass before trusting any landing result |
| **10** | **Detour + memory, two runs; run 2 = zero dodges** | **The single most research-relevant open result in the system** |
| 11 | Three-drone fleet concurrency | Prerequisite for any multi-agent claim |
| 12 | New operator flies from the guide alone | Reproducibility |

**Implication for research planning:** § 4, § 7, § 8 and § 10 are not research
work. They are *baseline establishment* — the numbers a paper compares against.
They must be run before any proposed method can be evaluated, and the effort to
run them belongs in the timeline as such.

---

## 4. Architectural gaps that are research opportunities

Ranked by how directly they map onto an open question:

1. **The obstacle memory has no disconfirmation channel.** `ObstacleReporter`
   writes *only* on `entering_dodge`. The avoider continuously computes
   `eff_front_m` — direct evidence of *free space* — and that signal is thrown
   away. A false or stale obstacle therefore persists for ~months (τ = 14 d)
   regardless of how many drones fly straight through it. See `D1`.

2. **A calibrated perception forward model exists but is never used online.**
   `decodable_px_at_altitude()` is a measured, validated function of altitude,
   consulted once at design time to choose a constant. The landing controller
   makes altitude decisions using an unrelated geometric heuristic. See `D2`.

3. **Reactive→deliberative handoff is a fixed 12 s timer.** No prediction of
   whether dodging *will* resolve, no accounting for the energy cost of the
   detour it triggers. See `D3`.

4. **No uncertainty anywhere in perception.** Detections are booleans with pixel
   offsets. No covariance, no per-detection quality, no tilt, no blur.

5. **The planner ignores confidence.** `plan_detour` reads `lat/lon/radius_m`
   and never reads `confidence`, even though the database computes and decays
   it carefully.

6. **Energy is a placeholder and payload mass is unmodelled.** Both would need
   real calibration before any energy claim is defensible.

7. **The world has one static obstacle and no environmental variation.** Any
   experimental protocol must first build a richer, parameterised world
   generator. `world/build_world.py` (OSM → boxes) is a starting point but is
   currently unused by the mission path.

---

## 5. What must NOT be claimed as novelty

These are already done and documented. Presenting any of them as a research
contribution would be rejected immediately:

- Precision landing on ArUco markers with disambiguation by ID
- A three-leg pickup / delivery / return-home mission
- Reactive 2D-LiDAR dodging with hysteresis
- Escalation from reactive avoidance to a geometric detour
- Persisting obstacles across missions in a database with confidence and decay
- Multi-drone fleet dispatch with a job queue and per-drone topic isolation
- Integrating PX4 + MAVSDK + ROS 2 + Gazebo + a web UI
- A safety supervisor with heartbeat, geofence and battery checks
- Kinematic payload attachment
