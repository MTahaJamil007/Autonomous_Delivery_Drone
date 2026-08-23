# Literature review

Scope: 2022–2026 primary, with foundational older work where it defines the
problem. Searched 2026-08-23. Organised by the research direction each cluster
constrains, because the purpose of this file is **novelty defence**, not a
general survey.

Citation keys match `references.md`.

---

## 1. Vision-based precision landing on fiducial markers

### State of the field

This is a **mature, crowded** area. [Springer-JIRS-2025] reviews 143 papers
published 2018–2025 on vision-based autonomous UAV landing, with fiducial
markers as a central category. Reported accuracies are routinely centimetric:
[Embedded-ArUco-2021] reports 2.03 cm mean error (σ = 1.53 cm) in simulation;
survey-level figures put marker-assisted landing at ~11 cm average offset
against metre-level GPS-only landing.

### What has been solved

- **Marker design for robustness.** Recursive ArUco [RArUco-2026] nests
  sub-markers so detection survives 30% random occlusion at a 100% rate and
  degrades gracefully past 40% noise; when the parent marker is compromised
  detection falls back to an inner sub-marker. Embedded ArUco
  [Embedded-ArUco-2021] uses the same nesting idea specifically to keep a
  marker decodable across a wide altitude range. ChArUco2 variants supply more
  corners under occlusion.
- **Detector comparison.** [Electronics-2026-Fiducial] experimentally compares
  ArUco and AprilTag detection rate and throughput for UAV landing guidance,
  concluding both are adequate in real time and ArUco is cheaper.
- **Filtering.** [Springer-IJASS-2023] applies image filtering plus a Kalman
  filter to fiducial-based landing to smooth pose estimates.
- **Learning-based landing.** [Sim2Real-DRL-Landing-2024] combines CV detection
  with DRL in AirSim explicitly to close the sim-to-real gap;
  [TornadoDrone-2024] does bio-inspired DRL landing on a 6-DoF moving platform
  under wind.
- **Landing without markers.** [RiskAware-EmergencyLanding-2026] performs
  semantic-segmentation-driven risk-aware emergency landing in urban scenes
  with altitude-dependent safety thresholds and temporal landing-point
  stabilisation.

### What is NOT solved

- **[Evidence-Landing-2026]** is the closest recent work to an
  uncertainty-explicit landing decision: it separates *decision-making under
  uncertainty* from *execution via visual servoing*, treating landing safety as
  a latent variable inferred by recursive temporal accumulation of geometric
  visual cues. But its domain is **unstructured terrain landing-site
  selection** — "is this patch safe to land on?" — not **fiducial
  re-acquisition and descent commitment** — "can I still read the tag I am
  descending onto, and should I keep descending?"
- Descent gating in the fiducial literature is essentially universally a
  **hand-tuned geometric heuristic** (a cone, a threshold, a timeout). I found
  no paper that derives the descent gate from a **calibrated, measured
  probability-of-decode model** as a function of apparent marker size,
  and then evaluates the resulting abort/commit policy.
- **Sensor-model calibration is reported as an implementation detail, not a
  result.** [Gazebo-Fiducial-Testing-2023] builds a testing architecture for
  fiducial recognition quality in Gazebo, which is the nearest thing to a
  detectability-characterisation protocol, but it does not feed a controller.

### Consequence for this project

Do **not** propose "precision landing" as a topic. The only defensible entry is
the narrow one: **closing the loop between a measured detection model and the
descent/abort decision.** Even then a reviewer's first question will be
"why is this not [Evidence-Landing-2026] with a marker?" — the answer must be
prepared in advance. See `novelty-analysis.md` § D2.

---

## 2. Shared, persistent obstacle knowledge across robots and missions

### State of the field — two separate literatures

**(a) Dense probabilistic mapping.** Occupancy grids with log-odds updates are
textbook [Moravec-Elfes-1985; Thrun-2005]. Each observation contributes
positive evidence for occupancy along the hit and **negative evidence for free
space along the ray**; isolated false detections decay and are overridden by
consistent free-space observations. Evidential (Dempster–Shafer) variants add
explicit ignorance modelling — [Bosch-Evidential-2024] (CVPR'24) for automated
driving occupancy prediction, [Evidential-Road-2021] for LiDAR road mapping.
Multi-robot occupancy-grid merging is well studied [OGM-Merging]; a common
merge rule takes the value of greater absolute magnitude rather than summing,
to avoid double-counting overlapping observations.

**(b) Multi-UAV collaborative mapping and search.** Recent work is dominated by
learning: [NGASAC-2025] uses graph-attention MARL for cooperative search in
partially observable low-altitude environments; [Dual-Timescale-MADDPG-2025]
adds a hierarchical two-timescale structure; [ARCog-NET-2025] builds an
Edge–Fog–Cloud hierarchy with "collective knowledge reuse" for swarm indoor
mapping via monocular SLAM; [Swarm-CA-Survey-2025] surveys MARL, federated
learning and neuro-inspired collision avoidance in swarms.

### What is NOT solved

The gap is between (a) and (b), and it is *architectural*:

1. **Sparse geo-referenced obstacle databases are treated as write-only.** The
   dense literature gets disconfirmation for free from ray-casting into a grid.
   A sparse, GPS-keyed record of discrete obstacles — the representation used
   by lightweight fleet services, by this project, and by most industrial
   delivery stacks that cannot afford to share dense maps over cellular — has
   **no equivalent free-space channel**. Belief is therefore monotonically
   increasing until a wall-clock decay term erodes it.
2. **Decay is time-based, not observation-based.** An exponential τ (this
   project: 14 days) is a proxy for "obstacles change eventually". It is
   *independent of how much evidence has accrued since*, so a wall confirmed
   fifty times and a phantom reported once decay at the same rate.
3. **No per-source reliability.** Multi-robot map merging assumes cooperative,
   equally-reliable agents. A fleet in which one drone has a miscalibrated or
   degrading sensor has no mechanism to discount that drone's contributions.
4. **Evaluation is at the map level, not the mission level.** The dense
   literature evaluates map accuracy (IoU, log-loss). Nobody I found evaluates
   shared obstacle memory by **delivery outcome over repeated missions** —
   route length, escalation count, energy, and the rate of *unnecessary*
   avoidance caused by stale beliefs.

### Foundational anchors worth citing anyway

Log-odds inverse-sensor-model updating [Thrun-2005 ch. 9], Dempster–Shafer
occupancy [Pagac-1998], and the stability–plasticity dilemma
[Grossberg-1987] — which is precisely the tension between "remember the wall"
and "forget the wall once it is gone".

### Consequence for this project

This is the **strongest gap available**, because the project already owns a
sparse geo-referenced confidence-fused decaying obstacle database with a REST
API and a planner that consumes it — an unusual and directly relevant asset.
See `novelty-analysis.md` § D1.

---

## 3. Reactive / deliberative arbitration and replanning timing

### State of the field

The reactive-vs-deliberative split is standard [Drones-2025-OA-Survey]:
reactive uses local sensing, responds fast, and **gets stuck in local minima**;
deliberative needs a map, is expensive, and handles clutter. Hybrids abound —
ORCA + online RRT for escaping non-convex obstacles, simulated annealing +
adaptive APF [SA-APF-2025], DRPA-MPPI [DRPA-MPPI-2025] adding dynamic repulsive
potentials to MPPI specifically because MPPI is "susceptible to local minima
near large or non-convex obstacles".

**The directly competing work is [When2Replan-2023]** (OMRON SINIC X). It asks
exactly the arbitration question — *when* should a navigation stack invoke the
global planner — and answers it with DRL, comparing against periodic replanning
and "patience timer" baselines over 100 trials per map layout. Their finding:
a learned replanner is comparable or substantially better than rule-based
strategies, and the best strategy is strongly environment-dependent.

Typical practice is confirmed to be exactly what this project does: a fixed
patience timer, tuned by trial and error.

### What is NOT solved

- [When2Replan-2023] is **ground robots** with a dense costmap and a full 2D
  nav stack. The aerial, **map-free, 2D-LiDAR-only** case is different: there is
  no costmap to detect entrapment in, and the decision has an **energy price**
  (a detour costs battery that the delivery mission has budgeted).
- No work I found makes the escalation decision **cost-aware** — trading the
  expected time-to-resolve of continued dodging against the expected detour
  length *and* the remaining energy budget.

### Consequence

Real but narrow. As a standalone paper the reviewer question "how is this not
When2Replan in the air?" is hard to answer decisively. Better as a *second*
contribution inside a memory paper, or a follow-up. See `novelty-analysis.md`
§ D3.

---

## 4. Perception-aware and information-driven planning

### State of the field

Perception-aware planning incorporates perception constraints into trajectory
generation to preserve localisability: RAPTOR [RAPTOR-2021], Fisher-information
fields, visibility-aware optimisation for aerial tracking
[Visibility-Tracking-2021], star-convex visibility constraints for inspection
[StarConvex-2022], FOV-constrained active perception without a prior map
[FLAP-2026], perception-aware planning in feature-limited environments
[PAP-FeatureLimited-2025], and implicit dual control for visibility-aware
navigation [DualControl-2025].

For **search**, informative path planning explicitly models altitude-dependent
sensor performance: [IPP-TerrainMonitoring] trades field-of-view against sensor
noise by varying altitude to reach high-confidence maps quickly;
[Obstacle-Aware-IPP-2019] does obstacle-aware adaptive IPP for UAV target
search; multi-resolution landing-spot search descends in altitude stages as the
posterior over suitable spots sharpens.

### What is NOT solved

The IPP-with-altitude literature targets **area coverage and target detection
over terrain**. It does not address the **single-known-target re-acquisition**
problem that a delivery landing actually is: the pad's position is roughly
known from GPS, the failure is a *lock loss* mid-descent, and the decision is
climb-to-reacquire versus continue-descending — a decision with an explicit,
measurable, monotone detectability model behind it.

### Consequence

Supports D2 as a *narrow* contribution. The generic claim "perception-aware
planning for UAVs" is entirely saturated.

---

## 5. Safety, runtime assurance, and degradation

### State of the field

- **Simulation-based testing.** Aerialist [Aerialist-ICSE-2024] provides a UAV
  simulation-testing tool at ICSE'24. [DroneTestPipeline-2025] gives a
  step-by-step robust autonomous-drone testing pipeline. Fuzzing sUAS state
  transitions [sUAS-Fuzzing-2026] uncovers cyber-physical failures.
- **Runtime verification / assurance.** LTL-based runtime enforcement for
  autonomous vehicles [REDriver-2024]; NASA's verification framework for runtime
  assurance of autonomous UAS [NASA-RTA-2024]; rule/ontology-based UAV
  verification [Drones-RuleBased-2024]; mission-level RTA for autonomous driving
  [MissionRTA-2026]; hardware-in-the-loop platforms for UAV runtime verification.
- **Uncertainty as a safety signal.** [Uncertainty-Unsafety-2025] (EMSE) gives
  empirical evidence that uncertainty monitoring on PX4-powered UAVs can trigger
  self-healing or pilot alerts.
- **Fault injection.** PX4 SITL + Gazebo supports wind models, GPS noise
  profiles and sensor-failure injection. [BASiC-2024] is a public UAV
  sensor-failure dataset (ArduCopter). [SimToReal-Testbed-2026] quantifies
  sim-to-real discrepancies in PX4 flight logs for hardware-vulnerability
  diagnosis.
- **Contingency policies.** [RiskMDP-Contingency-2023] formulates UAS
  contingency management as an MDP whose policy chooses among emergency landing,
  flight termination and continuation.

### What is NOT solved

Mission-level *quantitative degradation envelopes* — "for each sensor
degradation mode and magnitude, what is the delivery success rate, and where is
the cliff?" — are not standard. But building one is closer to a **benchmark
contribution** than a method contribution, and benchmark papers need scale and
adoption to have impact.

### Consequence

Good supporting material. As a primary topic it is a service contribution.
See `novelty-analysis.md` § D4.

---

## 6. Energy- and payload-aware delivery planning

### State of the field — saturated

[LPED-BatteryAware-2018] models battery-aware energy of drone delivery tasks;
[PayloadMass-Trajectory-2020] does payload-mass-aware trajectory planning;
[EnergyAwareMCPP-2024] (CTU MRS) does energy-aware multi-UAV coverage with
optimal flight speed; [Drones-MultiTrip-2026] optimises multi-trip UAV routing
with a second-degree polynomial power function fitted to empirical
thrust–power data, jointly handling distance- and payload-dependent
consumption, battery capacity and multi-trip feasibility;
[EnergyAware-Safe-2025] combines energy awareness with safety in path planning;
[Hydrogen-Review-2025] reviews the whole area including hybrid propulsion.
[BatteryHealth-2026] does motion-specific battery health assessment for
quadrotors with high-fidelity battery models.

### Consequence

**Do not pursue.** Calibrating `BATTERY_ENERGY_PER_M_PCT` is necessary
engineering for this project, but it is a measurement, not a contribution.

---

## 7. GNSS-denied navigation

### State of the field — saturated

[SatNav-GNSSDenied-2025] surveys GNSS-denied UAV navigation across
computational complexity, sensor fusion and localisation methodology.
Fiducial-corrected stereo VIO for large-scale GPS-denied bridge inspection
[FMC-SVIL-2023] reduces pose error ~50%. [SCLAM-2025] integrates monocular SLAM
with high-level control. [Heightmap-SPRIND-2025] wins a challenge with
kilometre-scale GNSS-denied navigation via heightmap gradients.

### Consequence

**Do not pursue as a primary topic.** The project has no VIO, no IMU fusion
beyond PX4's internal EKF, and no depth sensor. Building a competitive entry
means building a new perception stack from scratch against a field with decades
of momentum.

---

## 8. Learning-based UAV navigation

Saturated and compute-hungry: [MARL-UAV-Survey-2025], [Coop-MARL-2026],
[Swarm-CA-Survey-2025], memory-based SAC with prioritised replay
[MemSAC-2024], lifelong navigation learning [LLfN-2020]. The project has no
learning infrastructure, no dataset, and no GPU budget stated.

**Do not pursue as a primary topic.** A *small* learned component inside a
classical pipeline (e.g. a futility predictor) is the only defensible use.

---

## 9. Empirical software studies of UAV autonomy

[FSE21-Autopilot-Bugs] characterises 168 UAV-specific bugs from 569 real bugs
in PX4 and ArduCopter, producing a taxonomy and five detection challenges;
19.6% are misconfigurations. [ROBUST-2024] is a 221-bug ROS dataset.
[ROS-InteractionBugs-2025] classifies intra-system, hardware and environmental
interaction bugs. [ROS-Misconfig-2024] gives a ROS misconfiguration taxonomy.

The seven findings F1–F7 documented in this repository (undecodable marker
asset; sensors living only in an uncommitted submodule edit; a pixel-gain
controller whose loop gain varies 10× with altitude; unreachable
safety-fallback code; blocking calls starving an offboard stream; shared
absolute sensor topics across a fleet; unit confusion) are a **vivid**
single-system case study, but n = 1. Publishing them requires a corpus.

**Do not pursue as a primary topic**, but they are excellent *motivating
examples* in the introduction of whatever paper is written.

---

## 10. Summary of where the field leaves room

| Area | Saturation | Room for this project |
| --- | --- | --- |
| Fiducial precision landing (general) | **Very high** | None |
| Detection-model-driven descent commit/abort | Medium | **Narrow but real** |
| Dense occupancy mapping with negative evidence | Very high | None |
| **Sparse shared cross-mission obstacle memory with disconfirmation** | **Low** | **Real** |
| Mission-level evaluation of shared memory | **Low** | **Real** |
| Reactive/deliberative arbitration timing | Medium-high ([When2Replan-2023]) | Narrow |
| Perception-aware planning (general) | Very high | None |
| Energy/payload-aware routing | Very high | None |
| GNSS-denied navigation | Very high | None |
| Learning-based UAV navigation | Very high | None |
| UAV degradation benchmarks | Medium | Service contribution |
| UAV software bug taxonomies | Medium | Needs a corpus |
