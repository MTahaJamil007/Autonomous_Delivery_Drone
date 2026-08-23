# Top 3 research directions, and the final recommendation

---

# TOPIC 1 ★ RECOMMENDED

## Proposed paper title

**"Learning to Forget: Disconfirmation-Aware Shared Obstacle Memory for
Repeated Multi-UAV Delivery Missions"**

Alternatives:
- *"Sparse Obstacle Memory That Can Be Wrong: Evidence-Driven Forgetting and
  Source Reliability for UAV Delivery Fleets"*
- *"Mission-Level Evaluation of Persistent Shared Obstacle Belief in
  Multi-UAV Delivery"*

## Research problem

A fleet of delivery UAVs that shares what it has discovered should get better at
a route with every flight. In practice, shared obstacle knowledge is stored as a
**sparse, geo-referenced belief store** — a small set of `(position, radius,
confidence)` records exchanged through a service — because that is what a
bandwidth-constrained fleet can actually share, and because a geometric planner
can consume it directly.

That representation has an asymmetry nobody has addressed: **it can only ever
learn that something is there.** There is no mechanism by which flying straight
through a previously-reported location reduces belief in it. Confidence is
monotone, and the only counterweight is a wall-clock decay constant that is
independent of all evidence. The consequences are concrete and, once stated,
obvious:

- One spurious LiDAR return during a dodge becomes a permanent phantom obstacle
  that diverts every future route for months.
- A genuine obstacle that is removed keeps being routed around indefinitely.
- One drone with a degrading sensor poisons the belief of the entire fleet, and
  nothing discounts it.
- The system's own metric of success — "run 2 records zero dodges" — measures
  only that memory *accumulates*, never that it is *correct*.

## Research gap

Dense probabilistic mapping solved disconfirmation decades ago: occupancy grids
[`Thrun-2005`] get free-space evidence for free from ray-casting, and evidential
variants [`Bosch-Evidential-2024`, `Pagac-1998`] add explicit ignorance.
Multi-robot merging [`OGM-Merging`] handles overlapping observations. Multi-UAV
collective knowledge is an active learning field [`ARCog-NET-2025`,
`NGASAC-2025`].

**None of it applies to the sparse case.** With no grid there is no ray to cast;
the update must be attributed across *merged, overlapping* records with no fixed
cell geometry, from an observation that is a *sector minimum* rather than a
beam. There is no per-source reliability term anywhere in the merging
literature. And every one of these works evaluates at the **map level** (IoU,
log-loss, map accuracy) — nobody evaluates shared obstacle memory by **delivery
outcome over repeated missions**, which is what an operator actually buys.

## Proposed novelty / contribution

1. **A sparse disconfirmation operator.** Map free-space measurements
   (`eff_front_m` along a known heading from a known position) into negative
   evidence against stored obstacles, weighted by *contradiction depth* (how far
   into the stored radius the clear reading penetrates) and by *observer
   reliability*, with a defined attribution rule for overlapping records.
2. **Evidence-driven forgetting** replacing wall-clock decay, so belief responds
   to contradiction rather than the calendar.
3. **Online per-source reliability estimation** from each drone's agreement with
   fleet consensus, so a degrading sensor is discounted rather than trusted.
4. **A confidence-consuming planner** — clearance margin as a function of
   belief, replacing a constant margin that treats a fifty-times-confirmed wall
   and a one-off phantom identically.
5. **A mission-level evaluation protocol and metric set**, including the
   **unnecessary-avoidance rate** (detours around obstacles that are not there)
   and **time-to-forget** (missions required to stop routing around a removed
   obstacle) — metrics that essentially do not appear in the mapping literature.
6. *(Second contribution, from D3)* **Cost-aware escalation**: the decision to
   abandon reactive dodging and consult the shared memory, priced against the
   remaining energy budget rather than fired by a fixed 12 s timer.

## Closest existing research

| Work | What it does | Why it is not this |
| --- | --- | --- |
| `Bosch-Evidential-2024` (CVPR'24) | Evidence theory for occupancy prediction | Dense grid, single vehicle, automotive, map-accuracy metrics |
| `Thrun-2005` ch. 9 | Log-odds occupancy with inverse sensor models | Dense; free space comes from ray-casting, which does not exist here |
| `OGM-Merging` | Multi-robot occupancy merging | Dense; no source reliability; no cross-mission persistence |
| `ARCog-NET-2025` | Swarm "collective knowledge reuse" | Dense monocular SLAM, indoor, mapping-quality metrics |
| `LLfN-2020` | Lifelong navigation learning | Learned policy, not a shared symbolic belief store |
| `When2Replan-2023` | Learned replan timing | Ground robots with a dense costmap; no energy pricing |

## Why the proposed work is still different

Three defensible claims, in decreasing strength:

1. **Representation.** The sparse, service-backed, cross-mission, multi-source
   belief store is not a special case of an occupancy grid. The observation is a
   sector minimum, the records are merged and overlapping, and there is no cell
   geometry to attribute evidence to. The operator has to be defined.
2. **Reliability.** No published merging rule for shared robot maps estimates
   and applies per-source reliability. Fleets are heterogeneous and sensors
   degrade; this matters and is unaddressed.
3. **Evaluation.** Mission-level metrics for shared obstacle memory do not
   exist. *Unnecessary-avoidance rate* is, as far as this review found, novel as
   a reported quantity.

**Honest concession to make in the paper:** the operator reduces to a log-odds
update in the degenerate single-source, non-overlapping, fully-observable case.
Saying so pre-empts the strongest attack and costs nothing.

## Research questions

- **RQ1.** Does shared obstacle memory without disconfirmation degrade
  mission-level performance in the presence of false or stale obstacles, and by
  how much, as a function of the false-report rate?
- **RQ2.** Can a sparse disconfirmation operator driven by existing free-space
  measurements restore performance, and at what cost in re-collision or
  re-escalation risk? (The stability–plasticity tradeoff, quantified.)
- **RQ3.** Does evidence-driven forgetting dominate wall-clock decay across
  environment change rates, and where is the crossover?
- **RQ4.** Does online source-reliability estimation contain the damage of a
  single degraded reporter in a fleet, and how many healthy drones are needed to
  outvote one bad one?
- **RQ5.** Does making escalation cost-aware (energy-priced) rather than
  fixed-timer improve delivery success under tight battery budgets?

## Proposed methodology

**Phase A — baseline.** Run ACCEPTANCE § 4, § 8, § 10 to establish the existing
system's behaviour. Instrument every mission: path polyline, dodge episodes,
escalations, detour waypoints, energy proxy, obstacle DB snapshots before/after.

**Phase B — scenario generator.** Extend `sim/worlds/` with a parameterised
generator producing: (i) persistent obstacles, (ii) obstacles removed after
mission *k*, (iii) transient obstacles present for a window, (iv) injected false
reports at a controlled rate, (v) a designated "degraded" drone with biased
range readings. `world/build_world.py` (OSM → boxes) is the seed.

**Phase C — free-space channel.** Add a reporting path in `navigation.py` that
emits `(position, heading, eff_front_m, timestamp, drone_id)` clear-corridor
observations, deduplicated the same way obstacle reports are. **This is purely
additive** — no existing behaviour changes.

**Phase D — the operator.** Implement contradiction-depth weighting, evidence
decay, and source reliability in `obstacle_memory_service/db.py` behind a
feature flag so baseline and treatment are the *same binary*.

**Phase E — planner.** Make `plan_detour` read `confidence` and scale clearance.

**Phase F — batch harness.** Headless mission runner: N missions × M scenarios
× K policy variants, one JSON record per mission, resumable.

**Phase G — analysis.** Learning curves, ablations, sensitivity sweeps.

## Experiments

| ID | Experiment | Answers |
| --- | --- | --- |
| **E1** | **Repeated-route learning curve.** 30 missions on a fixed route through `great_wall`. Report dodges, escalations, path length, time, energy per mission index. | Establishes that memory helps at all — the baseline claim |
| **E2** | **Phantom obstacle.** Inject 1 false report on the direct route. 30 missions. Measure unnecessary-avoidance rate and excess path length over time. | RQ1 — the failure mode |
| **E3** | **Removed obstacle.** Real obstacle present for missions 1–10, removed at 11. Measure **time-to-forget**. | RQ2, RQ3 — plasticity |
| **E4** | **False-report rate sweep.** 0%, 5%, 10%, 20%, 40% spurious reports × {baseline, disconfirmation}. | RQ1, RQ2 — dose–response |
| **E5** | **Environment change-rate sweep.** Obstacle lifetime ∈ {1, 5, 20, ∞} missions × {wall-clock τ, evidence decay}. | RQ3 — the crossover |
| **E6** | **Poisoned source.** 3-drone fleet, one with a −40% biased range. {no reliability, with reliability}. | RQ4 |
| **E7** | **Ablation.** Full method minus each of: contradiction weighting, evidence decay, source reliability, confidence-aware clearance. | Attribution of the gain |
| **E8** | **Cost-aware escalation.** Fixed 12 s timer vs futility-predictor vs energy-priced, under battery budgets {loose, tight}. | RQ5 |
| **E9** | **Degradation stress** (imports D4). Vision dropout, LiDAR dropout, GPS noise × the full method. | Robustness |
| **E10** | *(Optional)* **Physical validation.** One drone, one real obstacle, place → 3 flights → remove → 3 flights. | External validity |

Scale: ~10 scenarios × ~30 missions × ~4 policy variants ≈ **1,200 SITL
missions**. At ~3 min headless per mission that is ~60 h of compute — feasible
overnight in batches on the existing host.

## Baselines for comparison

1. **No memory** — plan the direct route every time (reactive avoidance only).
   *This is the true floor and must be reported.*
2. **Current system** — accumulate-only, probabilistic-OR fusion, wall-clock
   τ = 14 d. *This is the paper's primary baseline.*
3. **Wall-clock decay, tuned** — sweep τ to its best value per scenario. *A fair
   baseline: a reviewer will ask whether the gain is just a better τ.*
4. **Dense occupancy grid** over the same area, with log-odds and ray-casting.
   *The strongest baseline, and the one that tests the representation claim.
   Report its bandwidth and memory cost alongside its accuracy.*
5. **Oracle** — perfect knowledge of true obstacles. *The ceiling.*

## Evaluation metrics

**Mission-level (primary)** — success rate; path length vs geodesic optimum;
mission duration; energy proxy (∫|v| dt, plus calibrated Wh once
`BATTERY_ENERGY_PER_M_PCT` is measured); dodge episodes; escalations; detour
waypoints.

**Memory-quality (secondary)** — **unnecessary-avoidance rate** (detours around
non-existent obstacles per mission); **time-to-forget** (missions until a
removed obstacle stops affecting routing); precision/recall of the belief store
against ground truth; calibration (reliability diagram of stated confidence vs
empirical presence — an ECE number is cheap and very persuasive).

**Safety (guardrail)** — minimum clearance distance; near-miss count; any
collision. *A method that forgets faster must not collide more. This is the
critical guardrail and must be reported prominently.*

**Cost** — bytes exchanged per mission; DB rows; query latency.

## Required changes to the current system

| Change | File | Size | Risk |
| --- | --- | --- | --- |
| Free-space observation reporting | `drone_agent/navigation.py` | ~80 L, additive | Low |
| `POST /freespace` endpoint | `obstacle_memory_service/app.py` | ~40 L | Low |
| Disconfirmation + evidence decay + reliability | `obstacle_memory_service/db.py` | ~250 L | Medium |
| Confidence-aware clearance | `global_planner/detour.py` | ~40 L | Low |
| Feature flags for every policy variant | `config.py` | ~30 L | Low |
| Parameterised scenario generator | `sim/scenarios/` (new) | ~300 L | Medium |
| Headless batch mission runner | `scripts/batch_missions.py` (new) | ~250 L | Medium |
| Structured per-mission JSON logging | `drone_agent/mission.py` | ~60 L | Low |
| Analysis notebooks | `research-paper/results/` | — | Low |
| **Prerequisite:** `scripts/sitl_landing_trial.py` and ACCEPTANCE § 4/§ 8/§ 10 | — | — | **Medium — must happen first** |

Everything respects the existing gates: config-only tunables, no blocking calls
on the flight path, no shadowed constants, tests for every new behaviour.

## Hardware / software requirements

**Software:** already installed — PX4 v1.17, Gazebo Harmonic, ROS 2 Humble,
MAVSDK 2.12.10, Python 3.10, OpenCV 4.13. Add: `pandas`, `matplotlib`,
`scipy`, `seaborn` for analysis only.
**Compute:** CPU only. The current host suffices. Headless Gazebo, no GUI.
Budget ~60–100 h of batch simulation.
**Hardware:** **none required.** Optional E10 needs one quadrotor, a downward
camera, one ArUco pad and a movable barrier.
**Dataset:** self-generated. The mission-log corpus is itself a releasable
artefact.

## Main risks

| Risk | Likelihood | Mitigation |
| --- | --- | --- |
| **§ 4 (three-leg mission) does not pass, blocking everything** | Medium | Front-load it. `STATUS.md` already predicts 2–3 first-flight defects. Budget 3 weeks before any research work |
| Effect size too small in a simple world | Medium | Design scenarios where the baseline *must* fail (phantom, removal, poisoning). Report effect sizes with CIs |
| Reviewer collapses it to "sparse log-odds" | **High** | Pre-empt in the paper; implement the dense-grid baseline and report its cost; concede the degenerate case |
| Forgetting faster causes collisions | Medium | Safety is a *guardrail metric*, reported prominently. If the tradeoff is bad, that is itself a publishable finding |
| SITL non-determinism swamps the signal | Medium | Fixed seeds; ≥30 repetitions; report distributions not means; non-parametric tests |
| Scope creep into D3 | Medium | E8 is *one* experiment. If it slips, cut it — the paper stands without it |

## Backup approach if the main idea fails

If disconfirmation shows no mission-level benefit, **the negative result is
still a paper** — but reframe as a *characterisation study*: "Sparse Shared
Obstacle Memory in UAV Fleets: A Mission-Level Characterisation of When It Helps
and When It Hurts", with E1–E5 as the contribution and the operator as an
attempted remedy. This is a genuinely publishable shape (workshop or
journal), and E1–E5 must be run regardless, so the fallback costs nothing extra.

Second fallback: pivot the same infrastructure to D4 (degradation envelopes),
which reuses the batch harness and scenario generator entirely.

## Publication potential

**Realistic targets:** IEEE Access; *Drones* (MDPI); *Robotics and Autonomous
Systems* (Elsevier); ICUAS; IEEE MRS. **Stretch:** IROS or an ICRA/IROS
workshop on multi-robot perception or field robotics; RA-L if the
characterisation is sharp and the physical validation lands.

**Assessment: 7/10.** Solid and finishable. Not a top-tier-main-conference
result on its own, but a genuine, defensible, reviewable contribution.

## Master's scholarship / profile value

**8/10.** Demonstrates: multi-agent systems, probabilistic reasoning under
uncertainty, robot perception, motion planning, experimental design at scale,
and — unusually for an undergraduate — *the judgement to identify a real gap and
run a controlled study against strong baselines*. The narrative is excellent in
an application: "I built a delivery drone system, noticed its shared memory
could only ever learn and never unlearn, and studied what that costs."

## Novelty score: **7.0 / 10**

---

# TOPIC 2

## Proposed paper title

**"Descend or Go Around? Detectability-Calibrated Commit/Abort Decisions for
Fiducial Precision Landing"**

## Research problem

Every fiducial landing system gates its descent on a hand-tuned heuristic — a
cone, a pixel threshold, a timeout — while the underlying sensor model is
*analytically known*: a marker of side `s` at altitude `h` spans `fx·s/h`
pixels, of which only the quiet-zone-corrected fraction is decodable. This
project measures that fraction (0.796) and validated the prediction in flight
(88.6 px observed vs 80.1 px predicted at 5.51 m). The model is *right there*
and no controller uses it. Consequently, go-around is a timeout, not a decision.

## Research gap

`Springer-JIRS-2025` reviews 143 papers and finds accuracy solved.
`Evidence-Landing-2026` brings decision-theory to landing but for *terrain
safety in unstructured environments*, not fiducial decodability.
`Gazebo-Fiducial-Testing-2023` characterises marker recognition quality but does
not feed a controller. **No work derives the descent gate from a calibrated
probability-of-decode model, or formulates commit/abort as a decision.**

## Proposed novelty / contribution

1. A calibrated **P(decode | apparent size, tilt, motion blur, illumination)**
   surface, fitted from controlled Gazebo sweeps.
2. A **detectability-aware descent policy** selecting vertical rate and
   climb-to-reacquire from that model plus measurement covariance.
3. A **commit/abort rule** thresholded on predicted probability of holding lock
   through touchdown, against the cost of a go-around.
4. A **belief-driven re-acquisition search** replacing the open-loop spiral.
5. *(Stretch, D10)* **Sim-to-real transfer** of the detectability surface.

## Closest existing research

`Evidence-Landing-2026` (evidence accumulation for landing decisions — terrain,
not markers); `RArUco-2026` (robustness via marker *design*, not via control);
`Springer-IJASS-2023` (Kalman filtering of marker pose — filters the estimate,
does not decide); `Sim2Real-DRL-Landing-2024` (learns the policy end-to-end,
does not expose a calibrated sensor model).

## Why the proposed work is still different

The latent variable is *decodability*, not terrain safety. The observation model
is closed-form and measurable rather than learned. The action space includes
**climb-to-reacquire**, which a servo-execution stage does not have. And the
contribution is a *model plus a policy derived from it*, not a marker design.

## Research questions

- **RQ1.** How well is decode probability predicted by apparent size alone, and
  how much do tilt, blur and illumination add?
- **RQ2.** Does a model-derived descent gate reduce touchdown error and/or
  time-to-touchdown versus the tuned geometric cone?
- **RQ3.** Does a principled commit/abort rule reduce failed landings without
  inflating the go-around rate?
- **RQ4.** Does belief-driven re-acquisition beat the open-loop spiral in
  time-to-lock?
- **RQ5** *(stretch)*. How far does the sim-fitted surface transfer to a real
  camera?

## Proposed methodology

Characterisation harness (sweep altitude 1–15 m × tilt 0–40° × illumination ×
blur, thousands of frames, log decode outcome and corner error) → fit a
parametric or logistic surface with uncertainty → derive the descent policy →
implement `scripts/sitl_landing_trial.py` (specified in ACCEPTANCE § 7 but not
shipped) → run trials → compare.

## Experiments

| ID | Experiment |
| --- | --- |
| L1 | Detectability characterisation sweep; fit and validate the surface |
| L2 | 100 landings × {geometric cone, tuned cone, model-derived gate}; touchdown error CDF |
| L3 | Adversarial conditions: 0/10/25/40% occlusion, low light, 20°/40° pad tilt |
| L4 | Forced lock-loss mid-descent; measure recovery rate and time |
| L5 | Commit/abort: false-abort and missed-abort rates vs a timeout baseline |
| L6 | Re-acquisition search: belief-driven vs spiral, time-to-lock |
| L7 | *(Stretch)* Real camera + printed pad; measure the sim-to-real gap |

## Baselines

Current geometric cone `0.35h + 0.15`; **the same cone with its slope and
intercept tuned by grid search** (essential — otherwise the gain is just
tuning); a fixed-rate descent; a timeout-only abort; `Springer-IJASS-2023`-style
Kalman-filtered servoing.

## Evaluation metrics

Touchdown error CDF and 95th percentile; success rate within 0.5 m; time to
touchdown; oscillation events below 3 m; go-around rate; false-abort /
missed-abort; lock-loss recovery rate and time; time-to-lock; calibration (ECE)
of the predicted decode probability.

## Required changes

Characterisation harness (new, ~250 L); model fitting (~150 L); descent policy
module (~200 L, replacing the cone behind a flag); belief-driven search
(~200 L); `scripts/sitl_landing_trial.py` (~250 L); scenario variation for
illumination/occlusion/tilt in `sim/`. **Prerequisite: ACCEPTANCE § 8 (camera
signs) must pass before any landing number is trustworthy.**

## Hardware / software

Software as installed. CPU only; the sweep is embarrassingly parallel.
Hardware: none for L1–L6; **L7 needs a real quadrotor, camera and printed pad.**
Dataset: self-generated; the characterisation dataset is releasable.

## Main risks

| Risk | Mitigation |
| --- | --- |
| Crowded field; reviewer asks "what is left?" | Answer in the abstract: the *decision*, not the accuracy |
| Confusion with `Evidence-Landing-2026` | Distinguish latent variables explicitly and early |
| Gain over a *tuned* cone is small | Report it honestly; the commit/abort and adversarial results carry the paper |
| ACCEPTANCE § 8 fails (signs wrong) | Must be resolved first; do not flip signs to make it pass |
| Sim decode behaviour is unrealistically clean | Add blur/noise/lighting; validate against the one real datapoint; L7 if possible |

## Backup approach

If the policy shows no gain, publish the **characterisation** — a calibrated
fiducial detectability model for UAV landing with its sim-to-real validation is
a useful, citable artefact, and the negative control result ("the tuned cone is
already near-optimal, and here is why") is honest and informative.

## Publication potential

**6.5/10.** ICUAS, *Drones*, IEEE Access, *Journal of Intelligent & Robotic
Systems*. Crowded, so acceptance depends on precise framing.

## Master's scholarship / profile value

**7.5/10.** Strong on computer vision, sensor modelling, decision-making under
uncertainty and control. Slightly narrower story than Topic 1.

## Novelty score: **6.5 / 10**

---

# TOPIC 3

## Proposed paper title

**"What Actually Breaks: Mission-Level Degradation Envelopes for a Fail-Closed
Autonomous Delivery Stack"**

## Research problem

Autonomy stacks are validated by whether they complete a nominal mission.
Almost nothing is published about *how they fail as conditions degrade* — where
the cliffs are, which degradations a fail-closed architecture converts from
crashes into safe holds, and which it does not catch at all. This project has an
unusually explicit fail-closed design (`is_stale` vs `has_ever_arrived`,
"never infer a clear path from silence") that has **never been stress-tested**.

## Research gap

`Aerialist-ICSE-2024` and `DroneTestPipeline-2025` provide testing
infrastructure; `sUAS-Fuzzing-2026` fuzzes state transitions; `BASiC-2024`
publishes a sensor-failure dataset; `Uncertainty-Unsafety-2025` shows
uncertainty monitors help on PX4. **Nobody publishes mission-outcome success
rate as a continuous function of degradation magnitude across modes** — the
envelope itself.

## Proposed novelty / contribution

1. A **parameterised degradation suite** for a complete delivery stack: vision
   dropout rate and duration, LiDAR dropout, GPS noise and bias, wind, marker
   occlusion, illumination, telemetry latency, battery model error.
2. **Published degradation envelopes** — success rate surfaces with located
   cliffs.
3. An **empirical evaluation of fail-closed design**: which degradations it
   converts into safe holds, which it merely delays, and which pass through.
4. A **failure-mode attribution method** tracing each mission failure to a
   specific stage of the stack via FSM history and structured logs.

## Closest existing research

`Aerialist-ICSE-2024`, `sUAS-Fuzzing-2026`, `BASiC-2024`,
`Uncertainty-Unsafety-2025`, `SimToReal-Testbed-2026`, `RiskMDP-Contingency-2023`.

## Why different

Those provide *tools and datasets*. This provides *the measured response
surface of one complete, documented, open stack* — and specifically evaluates a
design principle (fail-closed sensing) rather than a detector.

## Research questions

RQ1: Where are the cliffs for each degradation mode? RQ2: Which failures does
fail-closed convert to safe holds? RQ3: Do degradations interact
super-additively? RQ4: Which stack stage dominates failures at each operating
point? RQ5: Do the safety supervisor's fixed thresholds sit in the right place
relative to the measured cliffs?

## Experiments

Single-mode sweeps (7 modes × 6 magnitudes × 30 missions ≈ 1,260 runs);
pairwise interaction grids for the 3 most damaging modes; ablation of the
fail-closed guards; supervisor-threshold sensitivity; failure attribution across
the whole corpus.

## Baselines

The stack with fail-closed guards **disabled** (fail-open) — the crucial
comparison; PX4's own failsafes alone; nominal conditions as the control.

## Evaluation metrics

Mission success rate; safe-hold rate vs unsafe-outcome rate; time-to-detection
per mode; minimum clearance; geofence breaches; energy at termination;
attribution histogram by FSM state; envelope area (a single summary robustness
scalar).

## Required changes

A fault-injection layer (~300 L) intercepting the UDP feeds and telemetry —
clean, because all sensor input already flows through
`drone_agent/udp_receiver.py`; wind and lighting in `sim/`; the batch harness
(shared with Topic 1); structured failure logging.

## Hardware / software

CPU only, ~2,000 headless missions ≈ 100 h batch. No hardware.

## Main risks

Reads as engineering rather than science → mitigate by framing around the
*fail-closed design hypothesis* and testing it, not by cataloguing.
Benchmark value depends on adoption → mitigate by releasing the suite.
Findings may be system-specific → state the threat to external validity plainly.

## Backup approach

Narrow to the fail-closed evaluation alone — a focused paper on one design
principle, empirically tested.

## Publication potential

**6/10.** IEEE Access, *Drones*, ICUAS; software-engineering venues (ICST,
ICSE-SEIP) if framed as testing.

## Master's scholarship / profile value

**6.5/10.** Shows outstanding rigour and systems thinking; less algorithmic
depth than Topics 1 and 2, which matters for robotics-lab applications.

## Novelty score: **5.5 / 10**

---

# FINAL RECOMMENDATION

## Pursue **Topic 1 — Disconfirmation-Aware Shared Obstacle Memory**, with cost-aware escalation as the second contribution.

### 1. Why this topic is strongest

- **It grows from the project's most unusual asset.** Most FYP drone projects
  have a landing controller and an avoider. Very few have a *persistent,
  confidence-fused, decaying, service-backed obstacle database shared across
  missions and drones, already consumed by a planner*. That asset is what makes
  the question askable, and it is already built and tested.
- **The gap is real and easy to state.** "The system can learn that something is
  there and can never learn that it is gone" is a one-sentence problem statement
  that a reviewer, a supervisor, or an admissions committee immediately
  understands — and immediately agrees is a defect.
- **The experiments are cheap, numerous and controllable.** Everything runs
  headless on the existing host. Obstacles can be added, removed, faked and
  poisoned at will. 1,000+ missions is realistic, which means real statistics
  rather than three anecdotal runs.
- **It fails gracefully.** If the method does not help, the characterisation is
  the paper. Very few research plans have a fallback that costs nothing extra.
- **It is the least crowded intersection found.** Dense mapping is saturated;
  multi-UAV learning is saturated; the sparse shared-belief case is not.

### 2. Why it is better than the other two

**Versus Topic 2.** Topic 2 is a good idea in a field with 143 recent papers.
Every framing decision has to be defensive, and the reviewer's first instinct is
"solved". Topic 1's reviewer instinct is "huh, that is a real problem". Topic 1
also exercises multi-agent systems and probabilistic reasoning, which is a
better profile signal for robotics and autonomous-systems programmes than
another landing controller.

**Versus Topic 3.** Topic 3 is valuable and low-risk, but it is a *measurement*
paper. It produces envelopes, not a method. For a scholarship application the
distinction matters: Topic 1 lets the candidate say "I identified a gap and
designed an algorithm to close it"; Topic 3 says "I measured a system
carefully". Topic 3's best use is as Topic 1's robustness section (E9), which is
exactly what is planned.

**Cost consideration.** Topic 1 and Topic 3 share the batch harness, scenario
generator and structured logging. Building Topic 1 gets most of Topic 3 for
free — a second paper, later, at low marginal cost.

### 3. What the paper would claim as its contribution

Verbatim, as it should appear in the abstract:

> We identify a structural asymmetry in the sparse, geo-referenced obstacle
> belief stores used by bandwidth-constrained UAV fleets: they accumulate
> evidence *for* obstacles but have no channel for evidence *against* them, so
> false and stale beliefs persist and systematically degrade routing. We
> quantify this cost at the mission level over N repeated deliveries, and
> introduce (i) a sparse disconfirmation operator that converts free-space
> measurements into negative evidence attributed across overlapping records,
> (ii) evidence-driven forgetting that replaces wall-clock decay, and (iii)
> online per-source reliability estimation that contains the damage from a
> single degraded reporter. Across N simulated missions in PX4/Gazebo we show
> [X]% reduction in unnecessary avoidance and [Y]× faster recovery from removed
> obstacles, with no increase in minimum-clearance violations, and we
> characterise the stability–plasticity tradeoff as a function of environment
> change rate.

### 4. What experiments would prove that claim

- **The cost exists:** E2 and E4 must show that the accumulate-only baseline
  degrades measurably as the false-report rate rises. *Without this the paper
  has no problem.*
- **The method fixes it:** E2, E3, E4 with disconfirmation enabled must recover
  most of the oracle's performance.
- **It is not just tuning:** baseline 3 (τ swept to its best value) must lose.
  **This is the single most important control.**
- **Safety is not traded away:** minimum clearance and near-miss counts must not
  worsen. Reported prominently, not in an appendix.
- **Each component earns its place:** E7's ablation must attribute the gain.
- **It generalises:** E5's change-rate sweep must show where the crossover is,
  including where the baseline wins.
- **The representation claim holds:** baseline 4 (dense grid) must be either
  matched in quality or beaten in cost. Report both honestly.

### 5. What can be reused from the current codebase

Almost all of it. Reused **unchanged**: PX4/MAVSDK/Gazebo/ROS integration;
`DroneMission` orchestration and FSM; `SetpointPublisher`; the reactive avoider
and its `decide_action` state machine; the precision-landing controller;
`geo.py`; the safety supervisor; `fleet_dispatch/`; per-drone topic and port
isolation; `sim/` assets; the 126-test suite and 7 repo gates; the acceptance
framework.

Reused **as the object of study**: `obstacle_memory_service/`,
`global_planner/detour.py`, `navigation.ObstacleReporter`, the escalation path.

**This is the right ratio.** The paper's method is a small, sharp change to a
large, working, verified system — which is what makes the experiment credible.

### 6. What new components must be developed

| Component | New lines (est.) |
| --- | ---: |
| Free-space observation channel (`navigation.py`, `app.py`) | 120 |
| Disconfirmation operator + evidence decay + source reliability (`db.py`) | 250 |
| Confidence-aware clearance (`detour.py`) | 40 |
| Policy feature flags (`config.py`) | 30 |
| Parameterised scenario generator (`sim/scenarios/`) | 300 |
| Headless batch mission runner (`scripts/batch_missions.py`) | 250 |
| Structured per-mission JSON logging | 60 |
| Dense-grid baseline (for comparison only) | 200 |
| Fault injection for E9 (shared with Topic 3) | 300 |
| Cost-aware escalation for E8 | 150 |
| Tests for all of the above | 400 |
| Analysis and figures | 300 |
| **Total** | **≈ 2,400 lines** |

Against a 16,500-line existing system: about a 15% addition. Realistic.

---

# Research roadmap

```
Existing project
      │
      ├─ Weeks 1–3   BASELINE ESTABLISHMENT (not research — prerequisite)
      │              ACCEPTANCE § 4 three-leg mission · § 8 camera signs
      │              § 3 fail-closed · § 10 detour+memory two runs · § 11 fleet
      │              Expect 2–3 first-flight defects (STATUS.md predicts this)
      │              GATE: § 4 and § 10 must pass before anything below
      │
      ├─ Weeks 2–5   LITERATURE REVIEW (parallel with the above)
      │              Read in full: Evidence-Landing-2026, When2Replan-2023,
      │              Bosch-Evidential-2024, OGM-Merging, Thrun ch. 9,
      │              ARCog-NET-2025. Update literature-review.md + references.md
      │              GATE: no closer prior work found → confirm D1, else re-plan
      │
      ├─ Weeks 4–7   RESEARCH PROBLEM FORMALISATION
      │              Write the observation model, the disconfirmation operator
      │              and the reliability update on paper before coding.
      │              Update methodology.md · research-questions.md
      │
      ├─ Weeks 6–10  INFRASTRUCTURE + BASELINE IMPLEMENTATION
      │              Scenario generator · batch runner · structured logging
      │              Dense-grid baseline · τ-sweep baseline · oracle
      │              GATE: E1 learning curve reproduces the § 10 result at scale
      │
      ├─ Weeks 10–15 PROPOSED METHOD
      │              Free-space channel → disconfirmation operator →
      │              evidence decay → source reliability →
      │              confidence-aware clearance. Feature-flagged; tests for each
      │
      ├─ Weeks 14–22 SIMULATION EXPERIMENTS
      │              E1–E7 primary (~1,200 missions) · E8 escalation ·
      │              E9 degradation. Fixed seeds, ≥30 reps, distributions
      │              GATE: E2/E4 must show the baseline degrading, or pivot to
      │                    the characterisation-paper fallback
      │
      ├─ Weeks 20–26 PHYSICAL VALIDATION (optional, high value)
      │              One drone · one pad · one movable barrier · E10
      │              Place → 3 flights → remove → 3 flights
      │
      ├─ Weeks 24–30 ANALYSIS
      │              Learning curves · ablations · sensitivity · calibration
      │              Effect sizes with CIs · non-parametric tests
      │              Negative results written up, not buried
      │
      └─ Weeks 28–36 PAPER WRITING
                     Draft → internal review → supervisor review →
                     submit. Target: IEEE Access / Drones / RAS / ICUAS
                     Stretch: IROS or an ICRA/IROS multi-robot workshop
```

**Total ≈ 8–9 months** with overlap. The critical path is
**ACCEPTANCE § 4 → § 10 → E1**; everything else can be built in parallel.

**Hard gates — do not proceed past these without passing:**
1. Week 3: § 4 three-leg mission completes. *Everything depends on this.*
2. Week 5: no prior work found that closes the D1 gap.
3. Week 10: E1 reproduces the "run 2 has fewer dodges" result at scale.
4. Week 18: E2/E4 show the accumulate-only baseline degrading measurably.

If gate 4 fails, execute the backup: reframe as a characterisation study using
E1–E5, which will already be complete.
