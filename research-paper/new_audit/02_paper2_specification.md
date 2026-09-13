# 02 — Paper 2, re-scoped

**Status:** specification, conditional on Paper 1. Do not begin this work until
Paper 1's gates are passed. Everything here is written so that it can be
abandoned cheaply.

---

## 1. What changed, and why

The plan proposed:

> *"Learning When to Remember, Forget, and Verify: Risk-Aware Shared
> Environmental Memory for Multi-UAV Delivery"* — a learned task-conditioned
> memory-management policy over persistent shared beliefs.

Two problems.

**Problem one: the framing is already occupied.** Learning the value of
information for navigation under an incomplete map exists
([arXiv:2403.03269](https://arxiv.org/pdf/2403.03269)), as does learned active
perception and learned viewpoint selection. "An RL policy decides when to gather
information" is a 2024 result.

**Problem two: the action set is the wrong centre of gravity.** `KEEP /
DOWNWEIGHT / FORGET / VERIFY / REPLAN` over a single belief is a small
single-agent decision. A reviewer will ask why a decision tree, a bandit or a
one-step Bayes calculation does not solve it — and on a 12-dimensional state
with five actions, they may well be right.

**What the plan had right, and should be built on:** the *verification* action.
That is the part Paper 1 proves is necessary, and it is the part that becomes
genuinely hard in a fleet.

---

## 2. Title

> **Who Goes and Looks? Allocating Verification Effort over Shared
> Environmental Belief in Multi-UAV Delivery**

Alternative: *Paying to Be Sure: Cost-Bearing Belief Verification in Delivery
Fleets*.

---

## 3. The core idea

Paper 1 ends with: passive disconfirmation cannot refute a belief it is
successfully avoiding. The only escape is to **act against the plan** — fly
somewhere the route would not otherwise go, to look.

That act has four properties that no existing formulation combines:

1. **It costs delivery performance now.** A verification detour spends distance,
   time and battery inside a mission with a deadline and a reserve.
2. **Its benefit is deferred and external.** The corrected belief helps
   *whichever* drone flies that corridor *next*, possibly days later. The agent
   that pays is usually not the agent that benefits.
3. **It is an allocation, not a decision.** With `B` stale beliefs, `K` drones
   and a schedule of missions, the question is *which belief, verified by which
   drone, on which mission* — under a fleet-wide budget.
4. **It interacts with source reliability.** A belief supported only by a
   suspect reporter is worth more to verify than one confirmed by three healthy
   ones — and a suspect drone is worth less as the verifier.

Properties 2 and 3 are what make this a research problem rather than a threshold.
They create a credit-assignment structure that single-agent VoI does not have.

---

## 4. Contributions

1. **Formulation** of shared-belief verification in a delivery fleet as a
   constrained sequential allocation problem: cost paid now by one agent,
   benefit realised later by others, under a fleet verification budget and hard
   safety constraints.
2. **Baselines that could win**: a myopic value-of-information policy and, on
   the abstract environment, a near-optimal offline solver — so the learned
   policy is measured against the right thing.
3. **A learned task-conditioned allocation policy** operating over belief
   features, route consequence and reporter reliability, with the safety
   envelope enforced outside the policy.
4. **An ablation-derived finding** about which information actually drives the
   decision — the plan's own instinct here is right, and it is the most likely
   source of transferable insight ("route consequence dominates belief age").
5. **A generalisation study** over unseen layouts, change schedules, fault modes
   and fleet sizes.

---

## 5. Problem definition

Written before implementation, as the plan correctly insisted.

### 5.1 Decision structure

The policy runs **once per belief per mission-planning event** — not in the
control loop. It emits a recommendation; a supervisor filters it; the planner
executes. No learned component ever produces a setpoint.

```
shared belief store ──► candidate beliefs affecting this mission's route
                                    │
                                    ▼
                    policy π(s) → {KEEP, DOWNWEIGHT, FORGET, VERIFY}
                                    │
                                    ▼
                    safety supervisor (hard constraints, non-negotiable)
                                    │
                                    ▼
                       route planner ──► PX4
```

### 5.2 State (per candidate belief, plus fleet context)

| Group | Features |
| --- | --- |
| Belief | confidence; age since last confirmation; confirmations; contradictions; radius; merge count |
| Source | number of distinct reporters; min/mean reporter reliability; whether the only reporter is suspect |
| Consequence | detour distance this belief imposes on **this** mission; distance to verify it; whether any alternative route exists |
| Fleet | missions expected to traverse this corridor in the next horizon; number of drones; remaining fleet verification budget |
| Mission | remaining battery; reserve margin; deadline slack; payload state |
| Comms | staleness of the local belief copy; link quality |

Keep the state **small and interpretable**. Interpretability is what makes the
ablation a finding instead of a number.

### 5.3 Actions

`KEEP` · `DOWNWEIGHT` · `FORGET` · `VERIFY(belief)` — where `VERIFY` inserts an
inspection waypoint that brings the belief inside sensor coverage.

`REPLAN` is removed from the action set: it is the deterministic consequence of
the other three, not a separate choice. Keeping it creates aliasing.

### 5.4 Reward — and the constraint that is not a reward

```
+  delivery completed
−  excess distance / time / energy proxy
−  verification detour cost
−  unnecessary detour caused by a belief that was false
−  wrong forgetting (a forgotten belief later re-encountered)
```

**Collisions, clearance floors, geofence and battery reserve are NOT reward
terms.** They are hard constraints enforced by the supervisor outside the
policy. A reward term invites the optimiser to trade a collision against energy
at some exchange rate; a constraint does not. Use CMDP/Lagrangian formulation or
action masking, and say which.

This single choice does more for the paper's defensibility than the algorithm
does.

---

## 6. Kill criteria — read these before starting

Written in advance so that the decision to stop is cheap and unembarrassing.

| Gate | Criterion | If failed |
| --- | --- | --- |
| **K1** | On the abstract environment, the learned policy beats a **myopic VoI** policy on the primary endpoint, with a non-overlapping bootstrap CI | **Stop.** Publish the VoI formulation and Paper 1's limit result; no RL paper |
| **K2** | The margin over VoI **grows** in the multi-agent / deferred-benefit setting versus the single-agent one | The allocation framing is not doing work; reconsider the whole paper |
| **K3** | The policy transfers to held-out layouts and change rates with degradation smaller than the VoI gap | Report as a negative generalisation result; do not claim generality |
| **K4** | Safety is non-inferior to the classical system under a pre-specified margin | **Stop.** An unsafe efficiency gain is not a result |
| **K5** | The full-stack SITL confirmation reproduces the abstract-environment ordering of methods | The abstract environment is not a valid surrogate; the paper must be rebuilt on SITL alone, at much greater cost |

K1 is the one that matters. Reach it in the **abstract environment**, cheaply,
before any PX4 integration. The plan's instinct here — build
`MemoryManagementEnv` first — is correct and should be followed exactly.

---

## 7. Method progression

The plan's ladder is right. Tightened:

| Stage | Setting | Purpose | Exit condition |
| --- | --- | --- | --- |
| 0 | Abstract env, 1 belief, 1 agent | Sanity; does anything learn? | Beats random and always-KEEP |
| 1 | Abstract env, N beliefs, 1 agent | Selection among beliefs | **Beats myopic VoI (K1)** |
| 2 | Abstract env, N beliefs, K agents, deferred benefit | The actual contribution | **Margin grows vs stage 1 (K2)** |
| 3 | Abstract env, degraded reporter, comms loss | Robustness | Contains the fault |
| 4 | Held-out distributions | Generalisation (K3) | Degradation < VoI gap |
| 5 | PX4/Gazebo SITL, 3 drones, subset of cells | Full-stack confirmation (K5) | Same ordering of methods |

Stages 0–4 run in seconds-to-minutes per episode. Stage 5 is expensive and is
**confirmation only** — never the place where the result is discovered.

### On algorithm choice

PPO is a reasonable default for stage 0–2 and needs no defence beyond "standard,
stable, widely used". Do **not** claim it as a contribution, and do not reach for
MAPPO until stage 2 demonstrably needs it. If a centralised-critic method is
used, justify it by the credit-assignment structure (deferred, cross-agent
benefit) — which is a real reason, not a fashionable one.

Report: seeds (≥5 training seeds, reported individually as well as aggregated),
hyper-parameter search budget, wall-clock, and the same budget given to the
baselines' tuning. Learning-curve variance across seeds must be shown; a single
best run is not a result.

---

## 8. The signature experiment

The degraded-reporter study the plan proposed, with the allocation question
added:

```
UAV-1 healthy · UAV-2 healthy · UAV-3 range-biased
                     │
        beliefs supported only by UAV-3 accumulate
                     │
     ┌───────────────┴────────────────┐
classical                          learned
threshold / VoI                    allocation
     │                                │
 verify by rule                which belief, which drone,
                               which mission, within budget
     └───────────────┬────────────────┘
                     ▼
        delivery cost · safety · time-to-correct · verification spend
```

Measure: does the fleet (a) detect the inconsistency, (b) reduce reliance on the
suspect source, (c) allocate verification to healthy drones, (d) recover
delivery performance, (e) avoid over-spending on verification when the
environment is in fact static.

(e) is the discipline check: a policy that verifies everything looks good on
adaptation and terrible on cost. Report both.

---

## 9. Relationship to Paper 1 — and the redundancy risk

Paper 2 reuses Paper 1's simulator, scenarios, metrics and classical baselines.
That is efficient and correct. It is also a **publication-ethics hazard** if
handled carelessly.

Rules:

1. Paper 2 **cites** Paper 1 for the platform, the metrics and the limit result;
   it never re-presents Paper 1's results as new.
2. If Paper 1 is still under review, declare it in the cover letter as related
   unpublished work and **supply the manuscript to the editor**. Most publishers
   require this; failing to do it is how redundant-publication complaints start.
3. The contributions must be separable in one sentence each. If they cannot be,
   it is one paper, and it should be submitted as one.
4. Shared text (system description) must be rewritten, not copied. Self-plagiarism
   detectors do not care that you wrote both.

---

## 10. Honest assessment

Paper 2 is the **higher-risk, higher-ceiling** half. Its risk is concentrated in
K1: if a one-step Bayesian calculation solves the problem well, there is no
learning paper, and that is a genuinely plausible outcome for a small
interpretable state space.

The mitigation is structural, not optimistic: **K1 is cheap and early.** It runs
in an abstract environment, in days, before any integration work. The correct
attitude is that reaching K1 and failing it is a *successful* outcome of a
two-week investigation, not a wasted month.

If Paper 2 dies at K1, the fallback is strong and already half-built: a paper on
**cost-aware verification of shared belief with a value-of-information
formulation** — classical, principled, evaluated at mission level, with Paper 1's
limit result as its motivation. That is still novel in the delivery-fleet
setting and it needs no learning at all.
