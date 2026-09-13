# 01 — Paper 1, re-scoped

**Status:** specification. Not a draft, not a plan to write a draft. This is the
claim the experiments must be built to test.

---

## 1. What changed, and why

The 2026-08-23 specification listed **six contributions**: a disconfirmation
operator, evidence-driven forgetting, source reliability, a confidence-consuming
planner, a mission-level protocol, and cost-aware escalation.

That is too many contributions for one paper, and — more seriously — **three of
them are not contributions at all** (see
[`05_literature_verification.md`](05_literature_verification.md)):

| 2026-08-23 contribution | Verdict |
| --- | --- |
| Sparse disconfirmation operator | **Prior art** — persistence filters do this |
| Evidence-driven forgetting | **Prior art** — same |
| Per-source reliability | **Prior art** — inter-robot trust |
| Confidence-consuming planner | Engineering; obvious once stated |
| Mission-level evaluation protocol | **Keep** — weak but real |
| Cost-aware escalation (from D3) | **Cut.** `When2Replan-2023` owns it, and it dilutes the paper |

A paper with six contributions, three of which a reviewer can name prior work
for, reads as a system paper. **One idea, defended hard, is stronger.**

---

## 2. Title

> **Beliefs That Avoid Their Own Refutation: Shared Obstacle Memory in Repeated
> Multi-UAV Delivery**

Alternatives, in order of preference:

- *When Avoidance Prevents Correction: Mission-Level Evaluation of Shared
  Obstacle Belief in UAV Delivery Fleets*
- *The Belief a Delivery Fleet Cannot Unlearn*

Do not reuse *"Learning to Forget"* — it reads as a mechanism claim, and the
mechanism is exactly what is not new.

---

## 3. The core idea

This is the paper. Everything else supports it.

### 3.1 The observation

In mapping, the belief and the observation process are independent. A robot
mapping a corridor revisits it, and a feature that disappeared produces missed
detections, which a persistence filter converts into forgetting. That is why the
mechanism works.

In **delivery**, they are coupled, and coupled adversarially:

```
belief in obstacle  ──►  planner adds clearance  ──►  route passes wide
        ▲                                                    │
        │                                                    ▼
   belief persists  ◄──  no contradicting observation  ◄── obstacle outside
                                                            sensor coverage
```

A belief that is successfully avoided **stops generating the evidence that would
refute it**. The better the planner, the more complete the starvation.

### 3.2 Why this is not hand-waving — it is measurable in this system

`VERIFIED-CODE`, from [`10_codebase_evidence.md`](10_codebase_evidence.md) § 2:

- planned clearance ≈ `OBSTACLE_DEFAULT_RADIUS_M + DETOUR_MARGIN_M` = 3.0 + 6.5
  = **9.5 m** from the belief's centre;
- the reactive sensor's decision cone is **±35°**;
- a belief passed abeam at 9.5 m is **never inside that cone**.

So the disconfirmation signal that `methodology.md` § 2.3 proposed to use
(`eff_front_m`) is structurally unavailable for exactly the beliefs that matter.

### 3.3 The general form

Let a belief have radius `r`, let the planner enforce clearance `m`, let the
sensor have angular half-width `φ` and range `R`. A belief is **observable on a
route that avoids it** only if some part of its disc falls inside the sensor's
swept coverage of that route. This gives a condition of the form

```
observable( r, m, φ, R, route geometry )
```

and a **fixed point**: when the condition fails, belief evolves only under the
decay term, so any evidence-driven method — including a persistence filter —
reduces to its prior, and residual belief error is bounded below by a quantity
that depends on `(r, m, φ, R)` and **not** on the quality of the update rule.

Deriving this cleanly, and then **measuring it** across a margin/FOV/range sweep,
is the paper's central result. It is general: it holds for any avoidance planner
and any persistence model.

### 3.4 Why it matters practically

Three consequences follow, and each is an experiment:

1. **Better avoidance makes forgetting worse.** Increasing the clearance margin
   improves safety and degrades belief correction. That is a real trade-off
   nobody has stated.
2. **Passive disconfirmation has a ceiling.** No amount of tuning the update rule
   escapes it.
3. **Escaping it requires acting against the plan** — deliberately flying
   somewhere the planner would avoid. That is Paper 2, and Paper 1 ends by
   showing why it is necessary rather than by asserting it would be nice.

---

## 4. Contributions, as they will appear in the abstract

Exactly four. Numbered, so they can be checked against the results.

1. **A formulation** of shared environmental belief in repeated delivery as a
   closed loop in which the belief determines its own observation process, with
   an observability condition and a fixed-point bound on residual belief error
   in terms of clearance margin, sensor FOV and range.
2. **An empirical characterisation** over N independent runs of repeated
   missions, comparing: no memory, accumulate-only with wall-clock decay,
   *tuned* wall-clock decay, a **persistence filter**, a passive disconfirmation
   operator, and a dense occupancy grid — against an oracle, under phantom,
   removal, transient, heterogeneous-lifetime and degraded-reporter scenarios.
3. **A mission-level evaluation protocol** — metrics, scenario families,
   independence structure, safety guardrails — released with code, configs,
   seeds and data, for a setting currently evaluated only at map level.
4. **A quantified statement of the limit**: how much of the oracle gap passive
   methods can close, as a function of planner clearance, and where they cannot.

Note what is *not* in the list: any claim to have invented a forgetting rule.

---

## 5. Method (what actually gets implemented)

Everything is feature-flagged; baseline and treatment are the **same binary**.

### 5.1 Platform work — not contributions, but prerequisites

| Item | Purpose | Reference |
| --- | --- | --- |
| **P-A. Memory-conditioned pre-planning** | Makes the phenomenon exist at all | `10` § 1 (F8) |
| **P-B. Sectorised free-space channel** | Provides the observation | `10` § 2 (F9) |
| **P-C. Scenario generator** | Phantom / removal / transient / heterogeneous / degraded-source / comms | new |
| **P-D. Headless batch runner with resume** | 10³–10⁴ runs | new |
| **P-E. Structured mission records** | Every metric, with provenance | `templates/MISSION_RECORD_SCHEMA.md` |
| **P-F. Ground-truth oracle** | Precision/recall, unnecessary avoidance, oracle bound | new |

State P-A and P-B explicitly in the paper's system section, and say plainly that
they are standard. Hiding them invites the accusation that the baseline was
crippled.

### 5.2 Belief update variants

| Key | Variant | Role |
| --- | --- | --- |
| `V0-nomem` | No shared memory; reactive only | Floor |
| `V1-acc` | Prob-OR accumulation + wall-clock decay (τ = 14 d), memory-conditioned routing | **Primary baseline** |
| `V2-tau*` | `V1` with τ grid-searched on the training split | Tuning control |
| `V3-pf` | **Persistence filter**, re-implemented from Rosen et al. 2016 | **Strongest prior-art baseline** |
| `V4-disc` | Passive disconfirmation from the free-space channel | Treatment |
| `V5-disc+rel` | `V4` + Beta-reputation source weighting with abstention | Treatment |
| `V6-conf` | `V5` + confidence-scaled clearance, `m ∈ [m_min, m_max]` | Treatment |
| `V7-grid` | Dense log-odds occupancy grid + ray-casting, same planner interface | Representation control |
| `V8-oracle` | Ground truth | Ceiling |

### 5.3 The disconfirmation operator

Keep `methodology.md` § 2.3's shape — contradiction depth `κ ∈ [0,1]`,
multiplicative update `c ← c(1 − λκρ)` — with four corrections:

1. **Input is the sectorised free-space vector**, not `eff_front_m`.
2. **Altitude gating stays** (±2 m of the reporting altitude), and is stated as a
   limitation of a single-plane sensor, not hidden.
3. **Attribution across overlapping merged records** is an ablation, not an
   assumption: independent application vs `λ` normalised by the number of
   intersecting records. Both run.
4. **Present it as a variant of known negative-evidence updating**, and note
   explicitly that it reduces to a log-odds update in the single-source,
   non-overlapping, fully-observable case. Concede this in the method section.

### 5.4 What is cut

- Cost-aware escalation (old D3 / RQ5 / E8) — `When2Replan-2023` territory,
  dilutes the claim, requires a calibrated energy model that does not exist.
- Source reliability as a headline claim — demoted to `V5` and an ablation.
- Physical validation as a plan — kept only as an opportunistic bonus.

---

## 6. Metrics

**Primary endpoint (one, pre-specified):** *cumulative excess route distance
over a run, relative to the oracle*, in the removal scenario.

**Run-level secondary:** time-to-forget (missions until a removed obstacle stops
affecting routing, right-censored at the horizon — report a survival curve, not
a mean); residual belief error at horizon; unnecessary-avoidance count;
convergence mission index.

**Mission-level secondary:** route length ÷ geodesic; duration; energy proxy
(∫|v| dt — label it a proxy, `BATTERY_ENERGY_PER_M_PCT` is a placeholder);
dodges; escalations; delivery success.

**Belief-quality:** precision/recall against ground truth; calibration
(reliability diagram + ECE of stated confidence vs empirical presence).

**Safety guardrail (reported everywhere, non-inferiority tested):** collisions;
minimum clearance distribution and 5th percentile; near-misses;
re-collision-after-forgetting rate; supervisor interventions.

**Cost:** bytes/mission, rows, query latency — `V6` vs `V7`. This is what makes
or breaks the "why sparse?" answer, and it must be a measurement.

> **Naming discipline.** Call it *"unnecessary-avoidance rate"* and define it
> precisely; do **not** claim it is a novel metric. Related quantities exist in
> the mapping and detection literature under other names.

---

## 7. Results this paper must be prepared to report

Including the ones that hurt.

| Outcome | Probability | The paper then says |
| --- | --- | --- |
| `V3-pf` converges; coupling does not bite | **Medium-high** | "The existing method works in this setting; here is the first mission-level evaluation of it, and here is the margin regime where it starts to fail." Still a paper — weaker, honest, publishable |
| Coupling bites; all passive methods plateau | Medium | The paper as specified. Strongest outcome |
| Effect exists but is small | Medium | Report effect sizes and CIs; the margin/FOV sweep carries the generality |
| `V7-grid` matches `V6` at acceptable cost | Medium | Scope the claim to bandwidth-constrained deployment and **report the numbers** |
| Reliability weighting fails on disjoint routes | Medium | Report it; it is a concrete finding about a known hard case |

Write the paper so that **every one of these five rows produces a submittable
manuscript.** That is what makes the project safe to start.

---

## 8. Threats to validity to state explicitly

Inherit `methodology.md` § 5 and add:

| Threat | Type | Handling |
| --- | --- | --- |
| The coupling result may be a property of `plan_detour`'s geometry | External | Run a second, structurally different planner. **Highest-value robustness experiment in the paper** |
| Single-plane LiDAR cannot speak for out-of-plane obstacles | Construct | Altitude-gated updates; stated as a limitation |
| Free-space channel is new code written by the method's author | Internal | It is used identically by every variant including baselines; unit tests; released |
| Phantom injection rate is chosen, not observed | Construct | Measure the deployed reporter's actual false-report rate from pilot logs and report it. If it is ~0, the *removal* scenario carries the paper and the phantom scenario becomes a stress test |
| SITL LiDAR is cleaner than hardware | Construct | Noise/dropout/bias calibrated to a real datasheet; reported |
| Energy is a proxy | Construct | Say so in the metric definition, every time |
