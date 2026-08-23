# Novelty analysis

**Purpose: to argue *against* each idea before a reviewer does.**

The rule applied throughout: *a new combination of existing technologies is not
novelty.* An idea earns a "strong" rating only if there is a question the
literature has not answered, not merely a system nobody has assembled.

Each section states: what exists → the closest paper → what it does not solve →
the remaining gap → the specific contribution → an honest verdict, including
**the attack a reviewer will make and the answer**.

---

## D1 — Disconfirmation-aware shared obstacle memory

### What existing researchers have already done

**Dense probabilistic mapping solved this problem — in a different
representation.** Occupancy grids [`Moravec-Elfes-1985`, `Elfes-1989`,
`Thrun-2005`] update log-odds from an inverse sensor model, where every range
reading contributes occupancy evidence at the hit **and free-space evidence
along the ray**. Isolated false detections decay and are overridden by
consistent free-space observations. Evidential/Dempster–Shafer variants
[`Pagac-1998`, `Bosch-Evidential-2024`, `Evidential-Road-2021`] add explicit
ignorance. Multi-robot merging [`OGM-Merging`] handles overlapping evidence by
taking the greater absolute magnitude rather than summing.

**Multi-UAV collective knowledge is an active learning field.**
[`ARCog-NET-2025`] builds Edge–Fog–Cloud "collective knowledge reuse" for swarm
mapping; [`NGASAC-2025`] and [`Dual-Timescale-MADDPG-2025`] use MARL for
cooperative search in partially observable environments;
[`Swarm-CA-Survey-2025`] surveys MARL/federated/neuro-inspired swarm collision
avoidance.

### Closest recent papers

1. `Bosch-Evidential-2024` — evidence theory for occupancy, CVPR'24. **Dense
   grid, single-vehicle, automotive, map-accuracy metrics.**
2. `OGM-Merging` — multi-robot occupancy merging. **Dense grid, no per-source
   reliability, no cross-mission persistence, no mission-level metrics.**
3. `ARCog-NET-2025` — swarm collective knowledge. **Dense visual SLAM, indoor,
   mapping-quality metrics.**
4. `LLfN-2020` — lifelong navigation learning. **Learned policy, not a shared
   symbolic belief store.**

### What those papers do NOT solve

- **The sparse, geo-referenced, service-backed representation.** A REST-backed
  table of `(lat, lon, radius, confidence, last_confirmed, source)` rows — what
  this project has, and what a bandwidth-constrained commercial fleet actually
  shares over cellular — **has no ray to cast**. There is no free-space channel
  and therefore no disconfirmation mechanism. Belief is monotone until a
  wall-clock decay erodes it.
- **Evidence-independent decay.** τ = 14 days is a proxy for "obstacles change".
  It is independent of how much evidence has accrued, so a wall confirmed fifty
  times and a phantom seen once decay identically. That is wrong in both
  directions.
- **Source reliability in a heterogeneous fleet.** Merging assumes cooperative,
  equally-reliable agents. One drone with a degrading LiDAR poisons everyone,
  permanently.
- **Mission-level evaluation.** Everyone reports IoU / log-loss / map accuracy.
  Nobody reports *delivery outcome over repeated missions* — route length,
  escalation count, energy, and the **unnecessary-avoidance rate** caused by
  stale beliefs. That last metric essentially does not appear in the literature
  and is exactly what a delivery operator cares about.

### The gap, stated precisely

> For a **sparse, geo-referenced, cross-mission, multi-source** obstacle belief
> store consumed by a **geometric planner**, there is no published model of
> disconfirmation, no evidence-driven forgetting rule, no per-source reliability
> weighting, and no mission-level characterisation of the resulting
> stability–plasticity tradeoff.

### The proposed contribution

1. A **sparse disconfirmation operator**. When a vehicle's free-space
   measurement (`eff_front_m` along a known heading at a known position)
   geometrically contradicts a stored obstacle, apply negative evidence
   proportional to the *strength of the contradiction* — how deep into the
   stored radius the clear reading penetrates — and to the *reliability of the
   observer*. This is the sparse analogue of ray-casting, and it is not a
   trivial port: with no grid, the update must be attributed across
   *overlapping* obstacle records whose radii were themselves merged.
2. **Evidence-driven forgetting** replacing wall-clock τ, so belief responds to
   contradiction rather than to the calendar.
3. **Per-source reliability** estimated online from each drone's agreement with
   fleet consensus, so a degrading sensor is discounted rather than trusted.
4. A **confidence-consuming planner**: clearance margin as a function of belief,
   instead of the current constant `DETOUR_MARGIN_M`.
5. **A mission-level evaluation protocol and metric set** for shared obstacle
   memory, including *unnecessary-avoidance rate* and *time-to-forget*.

### Is it actually novel? — verdict

**Yes, moderately. Strong research opportunity. Novelty 7/10.**

Not a breakthrough: the *ingredients* (negative evidence, reliability
weighting, evidential fusion) are all classical. The novelty is in **applying
them to a representation where they do not exist, identifying the failure modes
that absence causes, and evaluating at the mission level.** That is a legitimate
and publishable form of contribution — it is a *problem-identification plus
solution* paper, not a new-algorithm paper, and it must be written as such.

### The attack a reviewer will make, and the answer

> **"This is just log-odds occupancy mapping on a sparse map."**

Answer, prepared in advance: (a) The sparse case is not a subset — there is no
ray, the update must be attributed across merged, overlapping records with no
fixed cell geometry, and the observation model is a *sector minimum* rather than
a beam. (b) Log-odds has no source-reliability term; ours is estimated online.
(c) The contribution includes the empirical characterisation and the
mission-level metric set, neither of which follows from log-odds.
**Weakness to concede honestly in the paper:** the disconfirmation operator
reduces to a log-odds update in the degenerate single-source, non-overlapping,
full-observability case. Say so; it strengthens credibility.

> **"Why not just use an occupancy grid?"**

Answer: bandwidth and persistence. A fleet sharing state through a service over
cellular cannot exchange dense grids; the sparse table is the deployed
representation. Quantify this — report the bytes-per-mission of both. If the
grid turns out to be affordable at this scale, **say so and scope the claim to
bandwidth-constrained deployments.** Do not hide it.

---

## D2 — Detectability-calibrated landing with commit/abort

### What existing researchers have already done

Fiducial precision landing is **mature**. `Springer-JIRS-2025` reviews 143
papers (2018–2025). Centimetre accuracy is routine
(`Embedded-ArUco-2021`: 2.03 cm). Marker design for robustness is solved
(`RArUco-2026`: 100% detection to 30% occlusion). Detector comparison is done
(`Electronics-2026-Fiducial`). Kalman filtering of marker pose is done
(`Springer-IJASS-2023`). DRL landing with sim-to-real is done
(`Sim2Real-DRL-Landing-2024`, `TornadoDrone-2024`). Risk-aware landing-site
selection is done (`RiskAware-EmergencyLanding-2026`,
`NeuroSymbolic-Landing-2025`).

### Closest recent paper

**`Evidence-Landing-2026`** — evidence-based landing site selection with a
latent landing-safety variable inferred by recursive temporal accumulation of
geometric visual cues, explicitly separating decision-under-uncertainty from
visual-servo execution.

### What it does not solve

`Evidence-Landing-2026` asks **"is this patch of ground safe to land on?"** in
*unstructured* terrain. It does **not** ask **"can I still decode the tag I am
descending onto, and is descending further a good bet?"** The two are different
latent variables with different observation models. In the fiducial case the
observation model is *known and measurable in closed form* — apparent size
`fx·s·q/h` — which makes calibration tractable in a way terrain safety never is.

More generally, across the fiducial-landing literature, **the descent gate is
universally a hand-tuned heuristic**: a cone, a pixel threshold, a timeout.
This project's own gate is `0.35·h + 0.15` — a slope and an intercept with no
derivation. I found no paper that derives it from a measured
probability-of-decode model, and no paper that makes go-around a decision
rather than a timeout.

The nearest thing to a characterisation protocol is
`Gazebo-Fiducial-Testing-2023`, which measures fiducial recognition quality in
Gazebo — **but does not feed a controller.**

### The gap

> The fiducial-landing literature has a *known, closed-form, measurable* sensor
> model and does not use it in the control law. Descent gating and go-around
> remain heuristics.

### The proposed contribution

A calibrated P(decode | apparent size, tilt, blur, illumination) surface; a
descent policy that selects vertical rate and climb-to-reacquire from that model
plus the measurement covariance; a commit/abort rule thresholded on the
predicted probability of holding lock through touchdown against the cost of a
go-around; and a belief-driven re-acquisition search replacing the open-loop
spiral.

### Verdict

**Strong research opportunity, but narrower than D1. Novelty 6.5/10.**

The area is crowded, so the contribution must be stated with precision and the
`Evidence-Landing-2026` distinction must be made in the *abstract*, not buried.

### The attack, and the answer

> **"143 papers on precision landing. What is left?"**

Answer: not accuracy — that is solved. What is unsolved is *the decision*: every
system descends on a heuristic gate and aborts on a timer, while the sensor
model is analytically known. We calibrate it and close the loop.

> **"Isn't this `Evidence-Landing-2026` with a marker instead of terrain?"**

Answer: the latent variable is different (decodability vs terrain safety), the
observation model is closed-form rather than learned, and the action space
includes *climb-to-reacquire*, which their servo-execution stage does not have.
Concede that the decision-theoretic *framing* is shared and cite them as the
methodological precedent — that is a strength, not a weakness.

---

## D3 — Cost-aware reactive→deliberative arbitration

### What exists

The reactive/deliberative split and its local-minima failure are textbook
(`Drones-2025-OA-Survey`, `Borenstein-Koren-1991`). Hybrids abound
(`DRPA-MPPI-2025`, `SA-APF-2025`). Standard practice is a fixed patience timer,
tuned by trial and error — which is precisely what this project does
(`ESCALATION_LOCK_S = 12.0`).

### Closest paper

**`When2Replan-2023`** asks exactly this question and answers it with DRL,
benchmarked against periodic and patience-timer baselines over 100 trials per
map layout, concluding the best strategy is environment-dependent.

### What it does not solve

Ground robots with a dense costmap. Aerial, map-free, single-plane-2D-LiDAR is a
genuinely different observability regime — there is no costmap in which to
detect entrapment. And no work prices the escalation decision against the
**energy budget** of a delivery mission, where a detour consumes reserve that
the return leg needs.

### Verdict

**Potentially publishable but incremental. Novelty 5.5/10.**

The gap is real but narrow, and `When2Replan-2023` is close enough that a
standalone paper invites a hard comparison the project may not win. **Use it as
D1's second contribution**, where the detour being triggered is planned from the
shared memory — that makes it part of one loop rather than a competing claim.

---

## D4 — Degradation envelope / benchmark

**What exists.** `Aerialist-ICSE-2024` (simulation testing), `sUAS-Fuzzing-2026`
(state-transition fuzzing), `BASiC-2024` (sensor-failure dataset),
`Uncertainty-Unsafety-2025` (PX4 uncertainty monitors),
`SimToReal-Testbed-2026`, `NASA-RTA-2024`, `REDriver-2024`.

**Gap.** Mission-level *quantitative degradation envelopes* — success rate as a
function of degradation mode and magnitude — are not standard.

**Verdict.** **Potentially publishable but incremental. Novelty 5.5/10.** This
is a *service* contribution whose value depends on adoption. **Use it as the
robustness evaluation section of D1 or D2**, not as the paper.

---

## D5 — Energy / payload-aware planning: **ALREADY SATURATED**

`LPED-BatteryAware-2018`, `PayloadMass-Trajectory-2020`, `EnergyAwareMCPP-2024`,
`Drones-MultiTrip-2026` (payload- *and* distance-dependent consumption from
fitted thrust–power data, multi-trip feasibility), `EnergyAware-Safe-2025`,
`BatteryHealth-2026`, plus a full review. **Novelty 3/10.**
Additionally the project's simulation does not model payload mass at all
(`cargo_box` is `static=true`, following is kinematic), so the claim would need
a physics rebuild before it could even be tested. **Do not pursue.**

---

## D6 — Dynamic obstacles with 2D LiDAR: **ENGINEERING-HEAVY / WEAK NOVELTY**

Tracking from a single 2D scan plane and velocity-obstacle avoidance are both
well-established. The sensor is weak for the task. **Novelty 3.5/10.**

---

## D7 — GNSS-degraded navigation: **TOO RISKY**

`SatNav-GNSSDenied-2025` surveys the field; `FMC-SVIL-2023` already does
fiducial-corrected VIO with ~50% error reduction; `SCLAM-2025` and
`Heightmap-SPRIND-2025` show the state of the art. The project has no VIO, no
independent IMU fusion, and one 320×240 downward camera. **Novelty 3/10, risk
8/10. Do not pursue.**

---

## D8 — DRL navigation: **NOT RECOMMENDED**

Saturated (`MARL-UAV-Survey-2025`, `Coop-MARL-2026`, `Swarm-CA-Survey-2025`),
no learning infrastructure exists in the project, and the classical baseline is
good. "Our DRL matches the classical baseline" is not a paper. **Novelty
2.5/10, risk 9/10.**

---

## D9 — Multi-drone spatial deconfliction: **ALREADY SATURATED**

ORCA, MADDPG variants, formation APF, and a 2025 survey. Coordination in this
project is job-level only, so this is a from-scratch build into a crowded field.
**Novelty 3/10.**

---

## D10 — Sim-to-real transfer of the detectability model

**Genuinely interesting** — the project has a measured sim decode model
(quiet-zone factor 0.796) and one live validation point (88.6 px measured vs
80.1 px predicted at 5.51 m, +10.6%). `SimToReal-Testbed-2026` shows the
community values quantified sim-to-real gaps. But **it requires hardware that
does not exist yet**. **Novelty 5.5/10, feasibility 4/10.** Excellent stretch
chapter for D2; poor primary bet.

---

## D11 — Bug taxonomy from F1–F7: **NOT RECOMMENDED as a paper**

`FSE21-Autopilot-Bugs` (168 UAV-specific bugs from PX4/ArduCopter),
`ROBUST-2024` (221 ROS bugs), `ROS-InteractionBugs-2025` and
`ROS-Misconfig-2024` own this space with corpora. n = 1 is not a study.
**Novelty 4/10, experimental potential 3/10.**

**But the findings are excellent motivation.** Two in particular open a paper
well: F5 — a synchronous `subprocess.run` at 10 Hz gapping the offboard setpoint
stream, so *enabling a documented feature* would have caused mid-flight mode
loss; and the safety supervisor whose RTL-failure fallback was unreachable
because the emergency flag latched before the command was accepted, meaning an
RTL rejection produced **no action at all**, silently, at the worst possible
moment. Both are perfect illustrations of "a system that appears to work and
does not" — which is exactly the framing D1 needs for stale obstacle beliefs.

---

## Summary judgement

| Direction | Classification | Novelty |
| --- | --- | ---: |
| **D1** Disconfirmation-aware shared obstacle memory | **Strong research opportunity** | **7.0** |
| **D2** Detectability-calibrated landing commit/abort | **Strong research opportunity** | **6.5** |
| D3 Cost-aware arbitration | Potentially publishable but incremental | 5.5 |
| D4 Degradation envelope | Potentially publishable but incremental | 5.5 |
| D10 Sim-to-real detectability | Potentially publishable but incremental | 5.5 |
| D11 Bug taxonomy | Not recommended (use as motivation) | 4.0 |
| D6 Dynamic obstacles, 2D LiDAR | Engineering-heavy / weak novelty | 3.5 |
| D5 Energy / payload planning | Already saturated | 3.0 |
| D7 GNSS-degraded navigation | Too risky | 3.0 |
| D9 Multi-drone deconfliction | Already saturated | 3.0 |
| D8 DRL navigation | Not recommended | 2.5 |
