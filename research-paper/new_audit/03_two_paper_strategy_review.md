# 03 — Review of the 26-phase two-paper plan

A direct response to the plan, phase by phase, against research best practice.

**Overall:** the plan is unusually good. The governance instincts — freeze a
baseline, separate engineering from research, permanent experiment IDs, gates
before RL, safety outside the learned policy, hostile review before submission,
reproducibility as a first-class artefact — are what distinguishes a research
project from a large FYP. Most of it should be executed as written.

Three things are wrong, and one is serious.

| | Issue | Severity |
| --- | --- | --- |
| 1 | The plan assumes the phenomenon already exists in the system. It does not (F8) | **Blocking** |
| 2 | All-SITL evaluation cannot supply the statistical power the claims need | **Blocking** |
| 3 | Repository restructuring would break a working, gated, tested system for no research benefit | High |
| 4 | Several timeline and novelty assumptions are optimistic in named ways | Medium |

---

## Phase-by-phase

### Phase 0 — Research governance · **KEEP, with additions**

Freezing and tagging is right, and nothing is tagged today (`git tag` is empty).

Add:
- Tag the baseline **only after ACCEPTANCE § 4 passes**. A frozen baseline that
  cannot fly a mission is not a reference point.
- Record the full environment in the tag message: PX4, Gazebo, ROS 2, MAVSDK,
  Python, OpenCV, kernel, CPU. **Pin PX4 to a stable tag or an archived commit
  SHA** — the current stack is on `v1.17.0-alpha1-1225`, and an alpha build is
  not a reproducible pin (`PRE_IMPLEMENTATION_AUDIT.md` Q24).
- Authorship: settle it now, as the plan says. Add two items the plan omits —
  your institution's IP and publication policy, and the publisher's
  **generative-AI disclosure** requirement, which now applies to essentially all
  major venues and must be stated in the manuscript.

### Phase 0.2 — Repository restructuring · **DO NOT DO THIS**

The proposed layout renames the flight stack into `drone_system/`, `navigation/`,
`obstacle_memory/` and so on. That would break imports, 126 tests and 7
repository gates, in exchange for nothing a reviewer can see.

Do this instead — purely additive, zero risk:

```
Autonomous_Delivery_Drone/
├── drone_agent/  global_planner/  obstacle_memory_service/   ← untouched
├── perception/   fleet_dispatch/  sim/  world/  tests/       ← untouched
├── research/
│   ├── scenarios/      parameterised world + belief generators
│   ├── experiments/    one runnable module per experiment ID
│   ├── envs/           the abstract MemoryManagementEnv (Paper 2)
│   ├── baselines/      persistence filter, dense grid, VoI, thresholds
│   ├── evaluation/     metrics, statistics, figure generation
│   └── configs/        one file per experiment cell
├── results/            raw mission records (git-ignored; archived externally)
├── models/             trained policies + training configs
└── research-paper/
    ├── new_audit/  paper1/  paper2/
```

Research code imports flight code. Flight code never imports research code —
enforce it with a repo gate, the same way the existing gates work.

### Phase 0.3 — Experiment identifiers · **KEEP exactly**

`P1-E01` style IDs with full provenance per run is exactly right. See
`templates/MISSION_RECORD_SCHEMA.md` for the record, and
`04_experiment_protocol.md` § 2 for the registry.

### Phase 1 — Make the system experimentally trustworthy · **KEEP, and expand**

Correct priority, correctly placed before RL. `engineering-failures.md` is a
genuinely good idea; a template is provided.

Expand it with three items the plan does not have:

1. **Fix F8 and F9** (`10_codebase_evidence.md`). Without F8 there is no
   phenomenon; without F9 there is no observation.
2. **Measure throughput.** Time 20 headless missions end-to-end, including
   PX4/Gazebo start and teardown, and measure how many instances run in parallel
   before per-mission time degrades. **Every number in the experiment budget
   depends on this and it has never been measured.**
3. **Measure non-determinism.** Run one cell 20 times with identical seeds.
   The spread is the noise floor; it determines how many runs each comparison
   needs, and it belongs in the paper.

### Phase 2 — Scenario classes S1–S7 · **KEEP, with one addition**

S1–S7 are well chosen. Add **S8 — heterogeneous lifetimes**: permanent and
transient obstacles in the same world. This is the cell where evidence-driven
forgetting should beat a single tuned τ, and the plan's own hypothesis
(correctly) predicts that. Without it, the tuned-τ baseline may simply win
everywhere, which is a much weaker paper.

Also: the scenario generator must **seed the belief store**, not only the world.
A run begins from a defined database state, recorded in the mission record.

### Phase 3 — Versions A–E · **RESTRUCTURE**

The A→E ladder (accumulate → disconfirmation → contradiction weighting → source
reliability → confidence-aware planning) is an implementation order, and it is
fine as one. But it is **not a contribution ladder**, and the plan treats it as
one. Versions B–D are prior art (persistence filters, trust-weighted fusion) —
see `05_literature_verification.md`.

Replace with the variant table in `01_paper1_specification.md` § 5.2, whose
important addition is `V3-pf`: **implement the persistence filter as a
baseline.** The plan's note that "the 2026 map-fusion literature already has
dynamic trust mechanisms" is correct and should be followed to its conclusion —
if the mechanism exists, implement it and compete with it.

### Phase 4 — Experiment campaign · **KEEP the shape, fix the statistics**

Baselines 0–4 plus oracle is the right structure, and the removal experiment is
the right centrepiece.

The dose–response sweep (0/5/10/20/40% false reports) is excellent design — but
first **measure the deployed reporter's actual false-report rate** from pilot
logs. The 2026-08-23 folder already flags this: if the real rate is ~0, the
phantom scenario becomes a stress test and the *removal* scenario carries the
paper. That is fine, but it must be known before the campaign, not after.

Statistics fix in `04_experiment_protocol.md` § 3.

### Phase 5 — Statistical layer · **KEEP, and move it earlier**

"Run this before writing the results" understates it: **run it before running the
experiments.** The power analysis determines the number of runs, which
determines the compute budget, which determines how many cells are affordable.
Doing it afterwards means discovering that a completed 200-hour campaign is
underpowered.

The plan's independence instinct is right — "30 consecutive missions are not 30
independent observations" is exactly the error to avoid — but it then designs
experiments as 30-mission sequences and counts them as reps. Resolve it: **the
run is the unit; ≥30 runs per cell; missions within a run are a time series.**

Add: non-inferiority testing for safety. "No significant increase in collisions"
is not evidence of safety, and a good reviewer will say so.

### Phase 6 — Paper 1 writing · **KEEP**

Writing during experimentation is right. Related work organised by cluster is
right. Add **lifelong/persistence mapping** as a cluster — its omission is the
biggest hole in the current literature review.

### Phase 7 — Submission · **KEEP the sequencing, defer the venue**

Submitting Paper 1 without waiting for Paper 2 is correct.

Do not lock a venue this early, and do not rely on any policy statement in this
document or the plan. Verify at submission time: simulation-only acceptability,
preprint policy, data/code requirements, AI-disclosure requirements. Choose a
venue class now (methodological robotics/autonomous-systems journal that accepts
simulation-only), and the specific journal at the point of submission.

### Phase 8 — Freeze and branch · **KEEP**

Nothing to add. Tag, branch, reuse.

### Phases 9–10 — Toy environment before PX4; learn PPO properly · **KEEP — the best decision in the plan**

Building `MemoryManagementEnv` before touching PX4 is exactly right, for a
reason beyond learning RL: it is where kill criterion **K1** is tested cheaply.

One correction: the plan says "the environment doesn't even need a drone". True,
and it should go further — **the abstract environment is not just a toy, it is
the primary experimental vehicle for Paper 2**, with SITL used for confirmation.
That reframing is what makes the compute budget feasible (§ 2 below).

### Phase 11 — Define the learning problem · **KEEP, with two fixes**

1. Remove `REPLAN` from the action set — it is the consequence of the other
   actions, not a peer choice, and it aliases the state.
2. **Collision must not be a reward term.** The plan says so later (Phase 19) and
   then lists `collision ---` in the reward. Pick the constraint. Details in
   `02_paper2_specification.md` § 5.4.

### Phase 12 — The verifier · **KEEP — this is the heart, and the plan is right**

Sharpen the baseline: the worked example ("confidence 0.64, LiDAR clear, detour
180 m, verification cost 22 m") is a **value-of-information calculation**. Write
that calculation down and make it the baseline the learned policy must beat. If
it cannot, there is no learning paper — and knowing that in week two instead of
month six is worth more than any result.

### Phase 13 — Training architecture · **KEEP**

1 agent → 1 agent/many beliefs → 2 → 3 → multi-agent learning is the correct
ladder. Add an exit condition to each rung (`02_paper2_specification.md` § 7) so
progression is a decision rather than a habit.

### Phase 14 — Intelligence levels B0–B5 · **KEEP, upgrade B3**

`P2-B3 "value-based heuristic"` is the most important baseline in the paper and
is under-specified. Make it an explicit myopic VoI policy. Add, on the abstract
environment only, an offline near-optimal solver (POMDP/MCTS) to bound how much
headroom exists at all. If the gap between VoI and optimal is small, no learned
policy can win by much, and that is worth knowing before training anything.

### Phase 15 — Data leakage · **KEEP, strengthen**

Splitting 70/15/15 by layout is necessary but not sufficient — split by
**generative parameters**, and hold out fault modes and change-rate regimes, not
only layouts (`PRE_IMPLEMENTATION_AUDIT.md` Q17).

Add the fairness direction the plan misses: **classical baselines must be tuned
on the training split too**, with the same budget. A learned method compared
against an untuned threshold is not a comparison.

Open the test split **once**.

### Phases 16–18 — Experiments, poisoned drone, ablations · **KEEP**

The ablation reasoning ("perhaps route consequence matters more than belief age")
is the plan's best scientific instinct — that is a transferable finding rather
than a performance number. Keep the state space small and interpretable so the
ablation can actually say something.

One framing note on the poisoned drone: keep it as **sensor degradation**, not
adversarial behaviour. The moment it is framed as Byzantine or malicious, the
resilient-consensus and security literature becomes prior art and the novelty
argument gets much harder.

### Phase 19 — Safety architecture · **KEEP — non-negotiable, and a selling point**

`RL → recommendation → supervisor → planner → PX4`, with hard constraints
outside the policy, is exactly right. Say it prominently in both papers: the
learned component never touches control, which is what makes the safety case
tractable. Reviewers reward this.

### Phases 20–22 — Writing, reproducibility, dataset · **KEEP**

"Every figure reproducible with one command" is the right standard.

On the dataset, the plan's own caution is correct and should be held to: logs
plus a schema, splits, baselines and evaluation scripts make a resource;
uploaded logs do not make a benchmark. Do not use the word "benchmark" until the
package is complete. Add a licence, a DOI, and a datasheet describing collection
and limitations.

### Phase 23 — Preprint · **KEEP, verify at the time**

Preprint policies vary and change. Check the specific journal's current policy
immediately before posting, and keep the preprint version identical to the
submitted version so there is no ambiguity.

### Phases 24–25 — Hostile review, independent sanity review · **KEEP — both**

The pre-written attack lists are good. `07_reviewer_rebuttal_pack.md` extends
them with the objections this audit surfaced, which are harder than the ones in
the current folder.

The 60-second test ("if they cannot say what is new, the paper isn't ready") is
an excellent bar. Note that under the current framing, the honest answer to
"what is new?" would be *"a forgetting rule"* — which is prior art. Under the
re-scoped framing it is *"beliefs that avoid their own refutation, and what that
costs a delivery fleet"*, which passes the test.

### Phase 26 — Journal strategy · **KEEP the logic, not the specifics**

Two venues, verified at submission, simulation-only compatible for Paper 1.
Everything in the plan about checking validation requirements is right; just do
the checking against the live author guidelines, not against a remembered
summary.

---

## The three structural corrections

### 1. The phenomenon does not exist yet (blocking)

See `10_codebase_evidence.md` § 1. Shared memory never influences a route until
after a 12-second escalation. Phantom cost is zero, time-to-forget is undefined,
and ACCEPTANCE § 10's own pass criterion cannot be met. Fix F8 first, label it
platform, and rebuild the baseline definition around it.

### 2. All-SITL evaluation cannot supply the required power (blocking)

The arithmetic, with the plan's own assumption of 3 min/mission:

| Design | Runs/cell | Missions/run | Cells | Missions | Hours |
| --- | ---: | ---: | ---: | ---: | ---: |
| As planned (missions counted as reps) | — | 30 | ~40 | 4,250 | ~214 |
| **Statistically valid** (runs as reps) | 30 | 25 | ~40 | **30,000** | **~1,500** |

1,500 hours is two months of continuous compute for Paper 1 alone, on an
unmeasured 3-minute assumption.

**The fix is a two-tier design**, and it is standard practice:

| Tier | Simulator | Cost | Role |
| --- | --- | --- | --- |
| **T1** | Abstract kinematic simulator: point-mass motion, the real planner, the real belief store, a modelled sensor cone | seconds/mission | All factorial sweeps, all power, all RL training |
| **T2** | PX4 + Gazebo SITL, full stack | minutes/mission | Confirmation subset: does T1's ordering of methods survive full dynamics? |

T1 uses the **actual** `plan_detour` and the **actual** `obstacle_memory_service`
— only vehicle dynamics and rendering are abstracted. That is what makes it a
credible surrogate rather than a separate experiment.

Then report **T1↔T2 agreement as a result**: same ordering, and the size of the
discrepancy. A reviewer who sees that comparison accepts the T1 sweeps; one who
sees only T1 does not.

This also solves Paper 2: T1 *is* `MemoryManagementEnv`, so Phase 9 stops being a
detour and becomes shared infrastructure.

Secondary lever: measure whether `PX4_SIM_SPEED_FACTOR`-style acceleration works
with this Gazebo Harmonic setup. **Measure it, do not assume it** — if it works
it changes the budget materially; if it breaks lockstep timing it corrupts
results silently.

### 3. Do not restructure the repository

Covered under Phase 0.2 above. Add research directories; touch nothing that the
existing tests and gates protect.

---

## Timeline

The plan's 7–8 months for two submissions assumes full-time work, no engineering
surprises, and no literature setback. Two facts argue against it: ACCEPTANCE § 4
has never been flown and `STATUS.md` itself predicts 2–3 first-flight defects;
and the multi-UAV claim in both titles depends on § 11, also never flown.

A realistic plan with the same content is in `09_timeline_and_gates.md`:
Paper 1 submitted around month 7–8, Paper 2 around month 13–15, with named
decision points where scope is cut rather than deadlines missed.

That is not a worse outcome. **Two well-defended papers submitted late beat two
rushed papers rejected on the first round**, and a rejection cycle costs 4–8
months anyway.

---

## The weekly split

40% experiments / 25% implementation / 20% analysis and writing / 15% learning is
sensible, and "your research itself should become the learning environment" is
right.

One adjustment for the first two months: that period is ~70% engineering
(ACCEPTANCE § 4, F8, F9, harness, throughput) and ~30% literature. Reading is the
only activity that can kill the project cheaply, so it must not be the thing that
slips when the simulator misbehaves — which it will.
