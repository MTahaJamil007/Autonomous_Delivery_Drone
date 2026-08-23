# Research directions

Ten candidate directions grown from `DroneProgram`, each classified, scored and
justified against the literature in `literature-review.md`. Novelty reasoning
lives in `novelty-analysis.md`; this file is the comparison table and the
per-direction cost estimate.

**Scoring convention.** All scores /10. For every row **higher is better
EXCEPT `Implementation risk`, where higher = MORE risky.** "Overall" is a
judgement, not an average — it weights novelty × feasibility × experimental
strength most heavily, because those three are what determine whether a paper
gets finished and survives review.

---

## Comparison table

| # | Direction | Class | Novelty | Research strength | Feasibility | Experimental | Publication | Scholarship | **Risk** | **Overall** |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **D1** | **Disconfirmation-aware shared obstacle memory for UAV fleets** | **Strong research opportunity** | **7** | **7.5** | **9** | **9** | **7** | **8** | **3** | **8.5** |
| **D2** | **Detectability-calibrated landing: descent commit/abort under a measured decode model** | **Strong research opportunity** | 6.5 | 7 | 9 | 9 | 6.5 | 7.5 | 3 | **7.5** |
| D3 | Cost-aware reactive→deliberative arbitration (futility prediction) | Potentially publishable but incremental | 5.5 | 6.5 | 8 | 8 | 6 | 7 | 4 | 6.5 |
| D4 | Fault-injection degradation envelope / benchmark for delivery autonomy | Potentially publishable but incremental | 5.5 | 6 | 8.5 | 8.5 | 6 | 6.5 | 3 | 6.5 |
| D10 | Sim-to-real transfer of the marker detectability model | Potentially publishable but incremental | 5.5 | 6 | 4 | 7 | 5.5 | 8 | 7 | 5.5 |
| D6 | Dynamic obstacle avoidance with 2D LiDAR only | Engineering-heavy / weak novelty | 3.5 | 5 | 6 | 7 | 4.5 | 5.5 | 6 | 4.5 |
| D5 | Payload- and energy-aware delivery planning | Already saturated | 3 | 4.5 | 6 | 6 | 4 | 5 | 5 | 4 |
| D9 | Multi-drone spatial deconfliction | Already saturated | 3 | 5.5 | 5 | 6.5 | 4 | 6 | 6 | 4 |
| D7 | GNSS-degraded navigation using the pad map | Too risky | 3 | 6 | 3.5 | 6 | 4 | 6.5 | 8 | 4 |
| D8 | DRL replacement for the reactive avoidance layer | Not recommended | 2.5 | 6 | 3 | 6 | 3.5 | 6 | 9 | 3 |
| D11 | Single-system bug taxonomy from findings F1–F7 | Not recommended (as a paper) | 4 | 4 | 9 | 3 | 3 | 4.5 | 2 | 3 |

---

## D1 — Disconfirmation-aware shared obstacle memory  ★ STRONG

**One-sentence question.** How should a fleet of delivery UAVs accumulate,
share, *disconfirm* and decay sparse geo-referenced obstacle beliefs so that
route quality improves monotonically with experience without becoming brittle
to false or stale reports?

**Existing engineering (already in repo, NOT a contribution)**
`obstacle_memory_service/` — aiosqlite persistence, position-hashed IDs, 5 m
merge radius, probabilistic-OR confidence fusion, confidence-weighted position
merge, wall-clock exponential decay (τ = 14 d), bbox query API.
`global_planner/detour.py` — tangent-bypass planner consuming that database.
`navigation.ObstacleReporter` — dedupe, forward placement along heading.
Fleet infrastructure, dispatch, per-drone topic isolation.

**New research contribution (what does not exist)**
1. A **disconfirmation channel**: mapping the avoider's continuous
   `eff_front_m` free-space measurements into *negative* evidence against
   stored obstacles, in a sparse (non-grid) representation where there is no
   ray to cast into a map.
2. An **evidence-driven decay** replacing wall-clock τ, so belief erodes as a
   function of *contradicting observations*, not elapsed time.
3. A **per-source reliability** term so one degrading drone cannot poison the
   fleet belief.
4. A **confidence-consuming planner**: `plan_detour` currently ignores
   `confidence` entirely; make clearance a function of belief.
5. **Mission-level evaluation** of shared memory — route length, escalation
   count, energy, and *unnecessary-avoidance rate* over repeated deliveries —
   which the mapping literature does not report.

**New work required.** Disconfirmation model + evidence-decay + source
reliability in `obstacle_memory_service/db.py`; a free-space reporting path in
`navigation.py`; confidence-aware clearance in `detour.py`; a parameterised
world generator with add/remove/phantom obstacle scenarios; a headless
batch-mission runner.

**Simulation sufficient?** Yes, for the core claim. Physical validation is a
*bonus* (one drone, one obstacle, one removal) and strengthens the paper but is
not load-bearing.

**Hardware.** None required for the main result. Optional: the existing
airframe plan + one ArUco pad + one movable barrier.
**Dataset.** Self-generated flight logs. No external dataset needed.
**Compute.** CPU only. PX4 SITL + Gazebo Harmonic on the current host.
Headless batch runs; ~100–300 missions is the target scale.
**Difficulty.** Medium. The algorithms are probabilistic bookkeeping, not deep
learning. The hard part is experimental discipline, not maths.
**Main risk.** The effect size could be small if the simulated world is too
simple — mitigated by designing scenarios (phantom obstacle, removed obstacle,
poisoned source) where the baseline *must* fail.

---

## D2 — Detectability-calibrated landing: commit/abort under a measured decode model  ★ STRONG

**One-sentence question.** If the probability of decoding a landing marker is a
measurable, monotone function of apparent size (hence altitude, tilt, blur and
illumination), can a descent policy derived from that calibrated model
outperform the hand-tuned geometric cone that fiducial-landing systems
universally use — and make abort/go-around a principled decision rather than a
timeout?

**Existing engineering (NOT a contribution)**
Metric altitude-invariant landing controller, EMA on the measurement, geometric
descent cone `0.35h + 0.15`, spiral search with footprint-overlap spacing,
`LandedState` touchdown confirmation, stall watchdog, fail-closed on stale
vision, marker disambiguation by ID, and — importantly — the
`decodable_px_at_altitude()` model with a **measured** quiet-zone factor 0.796
validated in flight (88.6 px observed vs 80.1 px predicted at 5.51 m).

**New research contribution**
1. A **calibrated probability-of-decode surface** P(decode | apparent size,
   tilt, motion blur, illumination) fitted from controlled Gazebo sweeps and
   validated against the one existing live datapoint.
2. A **detectability-aware descent policy** that chooses vertical rate and
   climb-to-reacquire actions from that model plus the current measurement
   covariance, replacing the heuristic cone.
3. A **principled commit/abort rule** — descend only while the predicted
   probability of maintaining lock through touchdown exceeds a threshold
   derived from the cost of a go-around.
4. A **belief-driven re-acquisition search** replacing the open-loop spiral.

**New work required.** A detectability characterisation harness (sweep
altitude × tilt × illumination × blur in Gazebo, log decode outcomes); model
fitting; a new descent policy module; `scripts/sitl_landing_trial.py` (which
ACCEPTANCE § 7 already specifies but does not ship).

**Simulation sufficient?** For the policy comparison, yes. For the *sim-to-real
transfer of the detectability model* — which is the strongest possible version
of this paper — no; needs hardware.
**Hardware.** Optional but high-value: any quadrotor with a downward camera and
a printed 2 m (or scaled) ArUco pad.
**Dataset.** Self-generated. Could be released as a contribution.
**Compute.** CPU only.
**Difficulty.** Medium.
**Main risk.** `Evidence-Landing-2026` is close in spirit. The framing must
stay on *fiducial re-acquisition and descent commitment with a calibrated decode
model*, not on landing-site safety.

---

## D3 — Cost-aware reactive→deliberative arbitration

**Question.** When reactive dodging is not going to resolve an obstacle, when
should the vehicle escalate to deliberative replanning, given that the detour
has a *known energy price* against a delivery mission's budget?

**Existing engineering.** `decide_action()` with a fixed
`ESCALATION_LOCK_S = 12.0` timer, level-triggered ESCALATE, latching in
`navigation.py`, `DETOUR_MAX_ATTEMPTS_PER_LEG = 3`.

**New contribution.** Replace the fixed timer with a futility estimator over
the dodge trajectory (progress-toward-goal rate, clearance trend, lateral drift
without forward gain), and make the escalation decision *cost-aware* by pricing
the expected detour against the remaining battery.

**Why only incremental.** `When2Replan-2023` already learns replan timing and
benchmarks it against patience timers. The aerial / map-free / energy-priced
framing is a real difference but a reviewer will press hard on it.

**Best use.** A strong *second contribution* inside D1 (the detour the
escalation triggers is planned from the shared memory), or a follow-up paper.

Sim sufficient: yes. Hardware: no. Compute: CPU (or small GPU if learned).
Difficulty: medium. Risk: the learned version needs many episodes and a
carefully designed reward; the heuristic version may look thin.

---

## D4 — Fault-injection degradation envelope for delivery autonomy

**Question.** For each sensor/actuator degradation mode and magnitude, what is
the delivery-mission success rate, and where are the cliffs?

**Existing engineering.** Fail-closed staleness architecture (`is_stale` vs
`has_ever_arrived`), safety supervisor, 126 tests, 7 repo gates, and an
acceptance framework that already separates BENCH from FLIGHT — an unusually
good foundation for a benchmark paper.

**New contribution.** A parameterised degradation suite (vision dropout, LiDAR
dropout, GPS noise, wind, marker occlusion, illumination, telemetry latency)
crossed with mission outcomes, yielding published *degradation envelopes* plus
the observation of which failures the fail-closed architecture converts from
crashes into holds.

**Why only incremental.** Aerialist, the sUAS fuzzing work, and BASiC already
occupy the testing/dataset space. A benchmark's value depends on adoption.

**Best use.** An *evaluation chapter* inside D1 or D2, not a standalone paper.

Sim sufficient: yes. Hardware: no. Compute: CPU, but many runs. Difficulty:
low-medium. Risk: low, but so is the ceiling.

---

## D5 — Payload- and energy-aware delivery planning — ALREADY SATURATED

`Drones-MultiTrip-2026`, `EnergyAwareMCPP-2024`, `PayloadMass-Trajectory-2020`,
`LPED-BatteryAware-2018` and a full review (`Hydrogen-Review-2025`) cover
distance- and payload-dependent consumption, optimal speed, multi-trip
feasibility and battery health. The project's `BATTERY_ENERGY_PER_M_PCT = 0.01`
is a placeholder; calibrating it is **necessary engineering** and belongs in the
system, not in a paper. Payload mass is not even modelled in the simulation
(`PayloadBay` is kinematic and `cargo_box` is `static=true`), so a payload claim
would require rebuilding the payload as a dynamic body first.

**Recommendation: do not pursue.** Do the calibration anyway — D1's energy
metric needs it.

---

## D6 — Dynamic obstacle avoidance with 2D LiDAR only — ENGINEERING-HEAVY

Would require adding moving obstacles to the world, tracking from a single 2D
scan, and velocity-obstacle-style avoidance. The tracking-from-sparse-2D-scan
problem is well studied, and single-plane 2D LiDAR is a weak sensor for it. The
result would read as "we implemented VO on a 2D scan". Novelty low.

---

## D7 — GNSS-degraded navigation via the pad map — TOO RISKY

Saturated (`SatNav-GNSSDenied-2025`, `FMC-SVIL-2023`, `SCLAM-2025`,
`Heightmap-SPRIND-2025`) and the project has no VIO, no IMU fusion of its own,
and one downward 320×240 camera. Building a competitive entry means building a
new perception stack against a field with decades of momentum. Very high risk
for a final-year timeline.

---

## D8 — DRL replacement for reactive avoidance — NOT RECOMMENDED

Saturated, compute-hungry, no learning infrastructure in the project, and the
existing classical avoider is *good* — beating it convincingly is hard, and
"our DRL matches the classical baseline" is not a paper.

---

## D9 — Multi-drone spatial deconfliction — SATURATED

ORCA, MADDPG, formation APF variants and a 2025 survey occupy this space. The
project's coordination is job-level only, so this is a from-scratch build into a
crowded field.

---

## D10 — Sim-to-real transfer of the detectability model

Genuinely interesting: the project has a *measured* sim decode model and one
live sim datapoint. Transferring it to a real camera and quantifying the gap
would be a strong result, and `SimToReal-Testbed-2026` shows the community
cares. But it **requires hardware that does not yet exist**, which makes it a
poor primary bet and an excellent *stretch chapter* of D2.

---

## D11 — Bug taxonomy from findings F1–F7 — NOT RECOMMENDED as a paper

The seven findings are a vivid case study (an undecodable marker asset that
made landing impossible; sensors existing only as an uncommitted submodule
edit; a controller whose loop gain varied 10× with altitude; a safety fallback
that was unreachable because a flag was latched too early; blocking subprocess
calls starving an offboard stream; absolute sensor topics shared across a
fleet; percent-vs-fraction unit confusion). But n = 1, and
`FSE21-Autopilot-Bugs` / `ROBUST-2024` already own this space with corpora.

**Best use: motivation.** Finding F5 (a blocking call gapping the setpoint
stream) and the unreachable safety fallback are outstanding opening paragraphs
for a paper about *systems that appear to work and do not*.

---

## Combinations assessed

| Combination | Coherent as ONE question? | Verdict |
| --- | --- | --- |
| **D1 + D3** (shared memory + cost-aware escalation) | Yes — "when the reactive layer fails, what should the fleet's accumulated belief tell the planner, and when should it be consulted?" | **Recommended as the paper's two contributions.** The escalation decision and the memory it plans from are the same loop. |
| **D2 + D10** (detectability model + sim-to-real) | Yes — one model, two domains | **Recommended if hardware appears.** Otherwise D2 alone. |
| D1 + D4 (memory + degradation) | Partly — D4 becomes the robustness evaluation of D1 | Use D4 *as* D1's robustness section (poisoned source, degraded sensor). |
| D2 + D4 | Partly | Use degradation as D2's stress axis (occlusion, illumination, blur). |
| D1 + D2 | **No** — two different subsystems, two different questions | Reject. Would read as an integration paper. |
| D1 + D5 (memory + energy) | Weak | Energy is a *metric* in D1, not a co-contribution. |
| D3 + D8 (learned futility predictor) | Yes but risky | Only if D3's heuristic version works first. |
