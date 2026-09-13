# PRE-IMPLEMENTATION AUDIT

**The research contract.** Nothing gets implemented until every question below
has an answer that survives being read aloud to a hostile reviewer.

Dated 2026-09-13. Evidence tags as defined in [`README.md`](README.md).

---

## THE QUESTION AT THE TOP

> *"If a reviewer rejects the paper saying 'this is just an existing technique
> applied to a drone', what exact evidence proves them wrong?"*

### The honest answer, for Paper 1

**Not the mechanism.** Evidence-driven forgetting over a sparse belief store is
an existing technique — the persistence filter (Rosen, Mason & Leonard, ICRA
2016) does exactly that, with public code. Any attempt to claim the mechanism is
new will be rejected, correctly.

**The evidence that proves them wrong is a measurement they cannot dispute:**

> A persistence filter — the strongest published method for exactly this problem
> — is implemented as a baseline in our delivery setting, and **it does not
> converge**. Not because it is badly tuned, but because in a delivery fleet the
> belief chooses the observation process: a route planned to avoid a believed
> obstacle removes that obstacle from sensor coverage, so the missed-detection
> evidence the filter depends on is never generated. We show the fixed point of
> this coupling analytically, measure the resulting residual error over N
> repeated missions, and show it is a function of the planner's clearance margin
> and the sensor's field of view — parameters no existing formulation contains.

That is a claim about a **failure of an existing technique in a setting whose
structure differs**, supported by an implementation of that technique and a
number. It is not defeated by "already known", because the existing technique is
in the paper, running, and losing for a reason the paper explains.

### The honest answer, for Paper 2

**Not "we used RL."** Learned value-of-information for navigation under an
incomplete map already exists (arXiv:2403.03269).

> The verification action in our setting is a **physical detour inside a
> deadline-bearing delivery mission**, whose benefit accrues to *different
> agents on future missions*. That makes it an allocation problem — which drone,
> on which mission, at what cost, for whose benefit — with a credit-assignment
> structure that single-agent information-gathering formulations do not have. We
> show a myopic value-of-information policy, which is near-optimal in the
> single-agent case, is measurably sub-optimal here, and identify the state
> features that account for the difference.

If the myopic VoI baseline is **not** measurably beaten, Paper 2 does not exist
as an RL paper. That criterion is written into `02_paper2_specification.md` § 6
as a kill condition, before any training run.

---

# A. Research-gap audit

## Q1 — What exactly is the unresolved problem?

`JUDGEMENT`, grounded in `VERIFIED-CODE` and `VERIFIED-WEB`.

A fleet flying repeated delivery missions accumulates a shared, sparse,
geo-referenced belief about where obstacles are, and plans routes from it. Three
things then become true simultaneously, and no existing formulation holds all
three:

1. **The belief is used to avoid, not to observe.** Routing around a belief
   removes the evidence that would correct it. The observation process is a
   function of the belief.
2. **The cost of being wrong is a mission cost**, paid in distance, energy,
   time and delivery success, not in map error.
3. **The belief is shared and multi-source**, so a correction made by one drone
   changes what every other drone will fly, and a fault in one drone propagates.

The unresolved problem is: **how should a fleet manage shared environmental
belief when believing something prevents it from being checked, and checking it
costs delivery performance?**

## Q2 — The closest work

`VERIFIED-WEB` (existence and topic confirmed 2026-09-13; **none read in full**).

| # | Work | Cluster |
| --- | --- | --- |
| 1 | Rosen, Mason & Leonard, ICRA 2016 — persistence filter | Sparse persistence |
| 2 | Krajník et al., T-RO 2017 — FreMEn | Temporal/spectral map models |
| 3 | "Better Together: Clique Change Detection in 3D Landmark Maps", 2020 | Sparse persistence |
| 4 | "Perpetua: Multi-Hypothesis Persistence Modeling", 2025 | Sparse persistence |
| 5 | Long-term LiDAR map maintenance / change detection, 2025–2026 | Dense map maintenance |
| 6 | Thrun, Burgard & Fox 2005, ch. 9 — log-odds occupancy | Dense mapping |
| 7 | Pierson & Schwager, ISRR 2013 — inter-robot trust | Multi-robot reliability |
| 8 | "Exploiting Trust for Resilient Hypothesis Testing with Malicious Robots" | Multi-robot reliability |
| 9 | "Active Information Gathering ... Value of Information", 2024 | Learned VoI |
| 10 | `When2Replan-2023` | Replanning arbitration |

Full detail and URLs: [`05_literature_verification.md`](05_literature_verification.md).

## Q3 — What each already solves

- **1, 3, 4:** Bayesian belief over the persistence of *sparse* features from
  positive and negative (missed-detection) evidence. Explicit survival priors.
  Online, recursive, with code.
- **2:** Predicting future occupancy from periodic temporal structure; extreme
  compression of long observation histories.
- **5:** Detecting and applying changes to a prior map across sessions.
- **6:** Free-space evidence from ray-casting; the canonical negative-evidence
  mechanism.
- **7, 8:** Estimating per-agent reliability online and down-weighting or
  excluding unreliable or adversarial contributions.
- **9:** Learning the value of information of an exploratory action for
  navigation under an incomplete map.
- **10:** Learning *when* to invoke a global replanner instead of using a fixed
  patience timer.

## Q4 — What each explicitly does not solve

`JUDGEMENT` — **and this is the column that must be re-derived from full texts
before submission.** Stated here as hypotheses to check, not as facts.

- **1–5:** assume the observation process is **exogenous** to the belief. A
  mapping robot's sensor coverage does not shrink because it became confident.
  None models a planner that actively removes a feature from coverage. None is
  evaluated by delivery outcome.
- **6:** requires a ray and a cell grid; both are absent, and the bandwidth
  argument for sparsity has to be *measured*, not asserted (Q14).
- **7, 8:** estimate trust over *overlapping* observations, and do not address
  the abstention case where an agent's route gives it no overlap at all.
- **9:** single agent; information benefit accrues to the acting agent within
  the same task; no deadline-bearing delivery cost; no shared persistent store.
- **10:** ground robots with a dense costmap; no energy price on the decision.

## Q5 — The gap, in one sentence

> **Existing models of environmental-belief persistence assume observations
> arrive independently of the belief; in a delivery fleet the belief determines
> the route and therefore the observations, so beliefs that are successfully
> avoided are never refuted — and no published method or evaluation addresses
> the resulting stability–plasticity failure at the level of mission
> performance.**

## Q6 — Could a reviewer reasonably say "this is already known"?

**Yes, on three specific grounds.** Assume they will.

| Objection | Reasonable? | Where it is answered |
| --- | --- | --- |
| "This is a persistence filter" | **Yes — very** | Implement it as a baseline; show it does not converge here, and why |
| "This is log-odds occupancy on a sparse map" | Yes | Concede the degenerate case in the paper; report the dense-grid baseline's cost and accuracy |
| "Trust-weighted fusion exists" | **Yes** | Demote reliability to an ablation component; cite the trust literature as its origin |
| "Learned VoI exists" (Paper 2) | **Yes** | Reframe as multi-agent allocation under delivery cost; beat a myopic VoI baseline or do not publish |

## Q7 — The precise answer

Three artefacts, prepared before writing, not after review:

1. **A working implementation of the closest prior method** (persistence filter)
   in this setting, reported fairly, including where it wins.
2. **An analytical result** giving the conditions under which passive
   disconfirmation cannot converge — expressed in the planner's clearance
   margin, the sensor's angular coverage and range, and the belief's radius.
   This is a short derivation, and it is what makes the finding general rather
   than an artefact of one simulator.
3. **A mission-level evaluation protocol** with released code, scenarios, seeds
   and data, so the claim can be checked rather than believed.

---

# B. Contribution audit

## Q8 — One-sentence contribution

**Paper 1:**

> We show that in repeated multi-UAV delivery, a shared sparse obstacle belief
> suppresses its own refutation — because avoidance removes the believed
> obstacle from sensor coverage — and that this bounds the performance of
> evidence-driven forgetting, including a persistence filter; we quantify the
> bound, and evaluate shared belief at the mission level for the first time.

**Paper 2:**

> We formulate deliberate verification of shared environmental belief as a
> cost-bearing allocation decision inside a delivery fleet, and show a learned
> task-conditioned policy outperforms myopic value-of-information and
> fixed-threshold verification under environment change and degraded reporters,
> without relaxing hard safety constraints.

## Q9 — What is actually new

| Component | New? | Honest label |
| --- | --- | --- |
| Sparse belief representation | No | Existing (this project's schema, and standard practice) |
| Negative evidence / forgetting mechanism | **No** | Existing (persistence filter, log-odds) |
| Per-source reliability | **No** | Existing (inter-robot trust) |
| The belief→route→observation coupling and its fixed point | **Yes** | **Problem formulation + analysis** |
| Mission-level evaluation protocol and metric set | **Yes, weakly** | **Evaluation contribution** — claim "not standard in this setting", never "first ever" |
| Active verification as fleet allocation under delivery cost | **Yes** (Paper 2) | **Decision-problem formulation** |
| Learned policy over that decision | Partly | **Method**, defensible only against a VoI baseline |
| PPO / MAPPO | No | Tooling |

The papers are a **problem-identification + analysis + evaluation** contribution
and a **decision-problem formulation + method** contribution. Write them as
those. Do not write them as new-algorithm papers; the algorithm is the weakest
part of both.

## Q10 — Existing technology that must NOT be claimed

Everything in [`10_codebase_evidence.md`](10_codebase_evidence.md) § 5, plus:
pre-emptive routing from a shared store; Bayesian persistence; log-odds; Beta
reputation; PPO/MAPPO; PX4/ROS 2/Gazebo integration; ArUco landing; occupancy
grids; geometric tangent-bypass detour planning.

## Q11 — The strongest competing method we must implement

**Paper 1: the persistence filter** (re-implemented from the ICRA 2016 paper —
not linked, for licence reasons). It is stronger than the current baseline, it
is stronger than tuned exponential decay, and it is the method a reviewer will
name. If it wins, that is the paper's result and the paper reports it.

Second-strongest: a **dense log-odds occupancy grid with ray-casting** over the
same region, sharing the same planner interface, with bytes-per-mission and
query latency reported alongside accuracy.

**Paper 2: a myopic value-of-information policy** — compute the expected
reduction in mission cost from verifying a belief versus the detour cost of
verifying it, and act greedily. On the abstract environment, also run an offline
POMDP or MCTS solver to establish a near-optimal reference. The learned policy
must be compared against both.

---

# C. Experimental audit

## Q12 — The hypotheses

Pre-registered, falsifiable, with directions fixed before running.

| ID | Hypothesis |
| --- | --- |
| **H1** | With memory-conditioned routing, a phantom obstacle imposes a measurable, persistent excess route cost under accumulate-only belief, increasing monotonically with false-report rate |
| **H2** | Passive disconfirmation reduces that cost, but **fails to converge** whenever the planned clearance keeps the belief outside sensor coverage; residual belief is a function of margin, FOV and range |
| **H3** | Evidence-driven forgetting beats *tuned* wall-clock decay specifically when obstacle lifetimes are spatially heterogeneous; tuned decay is competitive when they are uniform |
| **H4** | A persistence filter improves on both but is subject to the same non-convergence as H2 |
| **H5** | Reliability weighting keeps a fleet with one degraded reporter within a pre-specified non-inferiority margin of an all-healthy fleet |
| **H6** (P2) | A learned verification policy beats myopic VoI on delivery cost at equal or better safety, and the margin grows with environment-change heterogeneity |
| **H7** (P2) | The learned policy transfers to unseen layouts, change rates and fault modes with degradation smaller than the gap to the threshold baseline |

## Q13 — What would prove the hypotheses wrong

| Hypothesis | Falsified if |
| --- | --- |
| H1 | Excess route cost is flat in false-report rate, or decays to zero within the mission horizon under the baseline |
| H2 | Passive disconfirmation converges to oracle-level belief at every tested clearance margin — i.e. the coupling does not bite |
| H3 | A single tuned τ matches evidence-driven decay across every change-rate cell, heterogeneous ones included |
| H4 | The persistence filter converges here, in which case the paper's central claim is wrong and the honest result is "the existing method works; here is the mission-level evaluation showing it" |
| H5 | Reliability weighting fails to separate a degraded sensor from a drone legitimately seeing different obstacles |
| H6 | Myopic VoI matches or beats the learned policy — **Paper 2 stops here** |
| H7 | Performance collapses out of distribution — report it; an honest negative generalisation result is publishable, an unreported one is misconduct |

H4 deserves emphasis: **it is the outcome most likely to be true**, and the
project must be structured so that it is still a paper if it is.

## Q14 — Baselines that could make the method look unnecessary

Each of these must be implemented *well*, tuned on the training split, and
reported:

1. **Tuned exponential decay** (τ grid-searched). Answers "it is just a better
   constant."
2. **Persistence filter.** Answers "this is solved."
3. **Dense occupancy grid with ray-casting.** Answers "why sparse?" —
   and the answer must be a **measured** bytes/latency comparison, not the
   assertion that fleets cannot share grids. If the grid turns out to be
   affordable at this scale, **say so and scope the claim to
   bandwidth-constrained deployments.**
4. **Fixed verification threshold** (Paper 2). Answers "why learn?"
5. **Myopic VoI** (Paper 2). Answers "why RL?"
6. **Oracle** and **no-memory floor**. Without both, a percentage is
   uninterpretable.

## Q15 — Variables that must be controlled

**Held constant:** binary and commit hash (baseline and treatment differ only by
configuration flags); PX4/Gazebo/ROS versions; host and CPU governor; number of
concurrent SITL instances during a batch; airframe, cruise altitude, speed
limits; mission geometry within a scenario family; database state at run start
(explicitly reset and seeded).

**Systematically varied (factors):** policy variant; false-report rate; obstacle
lifetime distribution (uniform vs heterogeneous); fleet size 1/2/3; degraded
reporter present/absent; clearance margin; communication delay/loss; world
layout seed.

**Measured, never assumed:** wall-clock per mission; SITL non-determinism
(repeat one cell with identical seeds and report the spread); actual
false-report rate of the deployed reporter; bytes exchanged.

**Confounders with named controls:**

| Confounder | Control |
| --- | --- |
| Memory persists across missions within a run | The **run** is the unit of analysis, not the mission (Q16) |
| Scenario author is the method author | Pre-register scenarios before implementing the operator; include cells where the method is predicted to lose |
| Baseline under-tuned | Tune every baseline on the training split with the same budget as the method, and report the search |
| Metric chosen after seeing data | Fix primary endpoints in the pre-registration template before running |
| Host thermal/load drift over a 200-hour campaign | Randomise cell execution order; record timestamps; check for drift |

## Q16 — How many seeds and runs

**This is the most commonly fatal statistical error in this kind of study, and
the 2026-08-23 plan makes it.**

A 30-mission sequence with persistent memory is **one observation**, not thirty.
The missions inside it are serially dependent by construction — that dependence
*is* the phenomenon. Quantities like time-to-forget, cumulative excess distance
and convergence mission index are **run-level** statistics.

- **Unit of analysis: the run** (one seeded scenario instance, one policy
  variant, one persistent database, K missions).
- **Minimum 30 independent runs per cell** for a primary endpoint; 20 is
  defensible with a reported power analysis, fewer is not.
- **Mission-level metrics** (clearance, per-mission route excess) may be
  analysed within-run, but any across-condition comparison must aggregate to the
  run first, or use a mixed-effects model with run as a random effect. State
  which, in advance.
- **Report a power analysis** based on a pilot: 5 runs per cell, estimate
  variance, compute the runs needed for the effect size you care about. If the
  budget cannot support it, **reduce the number of cells, not the number of
  runs.**
- **Statistics:** non-parametric (Mann–Whitney U / Brunner–Munzel), effect sizes
  (Cliff's δ) with bootstrap CIs. Pre-specify **one primary endpoint per
  experiment**; everything else is secondary or exploratory and labelled so.
  Control family-wise error across primary endpoints (Holm).
- **Safety is a non-inferiority test, not a null result.** "No significant
  increase in collisions" is not evidence of safety — failing to reject H₀ never
  is. Pre-specify a margin (e.g. *the upper bound of the 95% CI on the
  difference in near-miss rate must lie below +10% relative*) and run a TOST or
  a one-sided non-inferiority test.

## Q17 — Unseen test environments

Split by **generative parameters**, not by instance:

| Split | Contents |
| --- | --- |
| **Train** | Layout families A–C; change rates {low, medium}; fault modes {range bias}; fleet sizes {1, 2} |
| **Validation** | Held-out layouts within A–C; used for hyper-parameters and for tuning **every** baseline |
| **Test (in-distribution)** | Held-out layouts from A–C, unseen seeds |
| **Test (out-of-distribution)** | Layout family D (never seen); change rate {high} and heterogeneous schedules; fault modes {dropout, intermittent, latency} never trained on; fleet size 3 |

Rules: the test split is opened **once**; every baseline is tuned on train/val
with the same budget as the method; in-distribution and OOD results are reported
separately and never pooled.

---

# D. Safety and validity audit

## Q18 — Could the method improve efficiency by making safety worse?

**Yes, directly.** Forgetting faster means flying routes that were previously
avoided; reducing clearance margin as a function of confidence means flying
closer to obstacles that may be real. Both are efficiency gains bought with
safety margin. This is the primary internal threat and must be treated as such.

## Q19 — The safety guardrail metrics

Pre-specified, reported for **every** condition in the main body — never an
appendix:

| Metric | Definition | Rule |
| --- | --- | --- |
| **Collisions** | Any contact with a ground-truth obstacle | **Absolute:** any collision in any condition is reported in the abstract |
| **Minimum clearance** | Min distance to a ground-truth obstacle per mission | Distribution and 5th percentile, per condition |
| **Near-misses** | Clearance below a pre-fixed threshold (fix before running) | Non-inferiority test vs baseline, pre-specified margin |
| **Re-collision after forgetting** | A believed-then-forgotten obstacle subsequently encountered | Reported as a rate; this is the specific harm forgetting causes |
| **Escalation rate** | Reactive avoidance failures | Safety proxy and efficiency cost |
| **Geofence / battery-reserve violations** | Supervisor interventions | Must be zero; any non-zero invalidates the run |

The clearance floor is enforced **outside** any learned or adaptive component.
Confidence may scale the margin between `m_min` and `m_max`; `m_min` is a hard
constant and is never a function of belief.

## Q20 — When the sensor is wrong, communication fails, or the environment changes unexpectedly

Each is an experiment, not an assurance:

| Failure | Experiment | Expected honest answer |
| --- | --- | --- |
| Biased or dropping range sensor | Degraded-reporter cells; fault modes held out for OOD | Reliability weighting contains bias; it may not contain intermittent dropout, and that limit is reported |
| Communication delay / loss | Delay and loss sweep; stale-belief cells | Belief staleness grows; the planner must fail *closed*, and the existing fail-closed guards are tested (ACCEPTANCE § 3) |
| Environment changes faster than training | OOD change-rate cells | Performance degrades; the report states where the method becomes worse than the baseline |
| Belief store unavailable | Kill the service mid-mission | The system must degrade to no-memory reactive behaviour, not hold indefinitely or fly blind |
| Learned policy input out of range (P2) | Clamp and log; supervisor overrides | A recommendation outside the safe envelope is discarded and counted; the count is reported |

---

# E. Publication audit

## Q21 — Does the contribution fit a serious venue?

`JUDGEMENT`. Yes, **as re-scoped** — as a problem-formulation-plus-evaluation
paper with a strong baseline comparison. Not as "a new forgetting rule".

Realistic Paper 1 targets: *Robotics and Autonomous Systems* (Elsevier);
*Journal of Intelligent & Robotic Systems* (Springer); *Field Robotics*; IEEE
RA-L (if the analytical result is sharp enough to fit the length); ICUAS or an
IROS/ICRA multi-robot-perception workshop for early visibility. *Drones* (MDPI)
and *IEEE Access* are plausible but weigh their perception and APC against the
master's-application value you want from this.

**Verify every journal's scope, format and policies at submission time, not
from this document.** Journal requirements change; nothing here should be
treated as current. What to check explicitly: simulation-only acceptability;
preprint policy; data and code availability requirements; and the publisher's
**disclosure requirements for generative-AI assistance in manuscript
preparation** — all major publishers now require a disclosure statement and
prohibit listing an AI system as an author.

## Q22 — Is simulation alone sufficient?

Depends entirely on the venue, and **this must be decided before the experiments
are designed**, because it determines whether physical validation is on the
critical path.

- Field-robotics venues generally expect field results or a credible analogue.
  Do not target one without hardware.
- Methodological and evaluation-focused robotics/AI journals accept
  simulation-only work when the experimental design is strong: many independent
  seeds, strong baselines, ablations, released code and data, and an explicit
  external-validity section.
- **The decision here:** target a venue that accepts simulation-only, and treat
  physical validation as a bonus that is cut without regret. With
  `FLEET_SIZE = 1`, no airframe, and ACCEPTANCE § 4 unrun, a hardware commitment
  is not credible.

## Q23 — What additional validation the strongest target would expect

1. **Sensor realism**: injected range noise, dropout and bias calibrated against
   a real 2-D LiDAR datasheet, with the calibration reported. Cheap, and it
   removes the "SITL LiDAR is too clean" objection.
2. **A second world family** structurally unlike the first (e.g. OSM-derived
   urban clutter via `world/build_world.py`), to show the effect is not a
   property of one map.
3. **A second planner**, even a simple one, to show the coupling result is a
   property of avoidance rather than of `plan_detour`'s geometry. This is the
   single highest-value robustness experiment for generality.
4. **Sensitivity to the parameters in the analytical result** — clearance
   margin, FOV, range — swept, with the predicted and measured non-convergence
   compared. This is what turns an observation into a result.
5. **A hardware demonstration** if one ever becomes possible; n = 6 flights with
   a movable barrier is worth a figure, and nothing more should be promised.

## Q24 — Can another researcher reproduce the figures?

Must be **yes**, by construction, from day one:

- One command per figure, from released configs and archived raw data.
- Every mission record carries commit hash, config hash, seeds, versions, host.
- The scenario generator is seeded and deterministic.
- Containerised environment with **pinned** versions. Note: the current stack
  records PX4 `v1.17.0-alpha1-1225` — **an alpha build is not a reproducible
  pin.** Pin to a stable tag or record and archive the exact commit SHA and
  build flags.
- Data archived with a DOI (Zenodo or institutional repository), licensed
  explicitly, with the schema documented.

Details in [`08_reproducibility_standard.md`](08_reproducibility_standard.md).

---

# GO / NO-GO

Implementation starts only when all five are true. Current state: **0 of 5.**

| # | Condition | State | Blocking item |
| --- | --- | --- | --- |
| 1 | The research gap is stated in one precise paragraph that survives the searches in `05` § 7 | ✗ | Q5 draft written; searches not run |
| 2 | The closest prior work is known **from reading it**, and the difference is precise | ✗ | 6 papers identified, 0 read |
| 3 | At least one strong baseline that could defeat the method is specified and implementable | ✗ | Persistence filter specified, not implemented |
| 4 | Experiments exist that can falsify the hypothesis, with enough runs to detect the effect | ✗ | H1–H7 drafted; no pilot, no power analysis, no throughput measurement |
| 5 | It is known what evidence makes the paper publishable, and the target venue accepts it | ✗ | Venue class chosen; requirements not checked |

**Additional hard blockers specific to this system**, which the five generic
conditions do not cover:

| # | Blocker | Why it blocks |
| --- | --- | --- |
| 6 | **F8** — memory does not influence routing | The phenomenon under study does not occur |
| 7 | **F9** — no free-space observation channel | The method has no input |
| 8 | ACCEPTANCE § 4 — three-leg mission never flown | Every experiment is a mission |
| 9 | ACCEPTANCE § 11 — three-drone fleet never flown | Both titles say "Multi-UAV" |

**Order of work:** 8 → 6 → 7 → 2 → 1 → 3 → 9 → 4 → 5.

Note that item 2 (reading the literature) is cheap, has no dependencies, and can
kill the project. Do it in parallel with item 8, and do not let it slip behind
the engineering.
