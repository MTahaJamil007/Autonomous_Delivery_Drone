# 04 — Experiment protocol

The rules every experiment obeys, fixed **before** any experiment runs. Changing
anything here after data exists requires a dated entry in
`templates/DECISION_LOG.md` explaining why, and the change is reported in the
paper.

---

## 1. Two-tier evaluation

| Tier | What it is | Cost | Role |
| --- | --- | --- | --- |
| **T1 — abstract** | Point-mass kinematics + the **real** `plan_detour`, the **real** `obstacle_memory_service`, a modelled planar sensor (FOV, range, noise, dropout) | seconds per mission | Every factorial sweep; all statistical power; all RL training |
| **T2 — SITL** | PX4 + Gazebo Harmonic + ROS 2, full stack, 1–3 drones | minutes per mission | Confirmation of a subset of cells |

**T1 is a surrogate, and its validity is itself a result.** For every primary
comparison, run the same cell in T2 and report:

1. whether the **ordering** of methods is preserved;
2. the magnitude of the discrepancy in the primary endpoint;
3. any cell where T1 and T2 disagree in direction — reported prominently, not
   buried.

A reviewer accepts a large T1 sweep backed by a T2 agreement study. A reviewer
does not accept a large sweep in an abstraction the authors built themselves
with no cross-check.

**T1 must reuse the real components.** If T1 reimplements the planner or the
belief store, it stops being a surrogate and becomes a different experiment.

---

## 2. Experiment registry

IDs are permanent. An ID is never reused, never renumbered, and never silently
redefined. A cell is `<ID>/<variant>/<factor levels>/<seed>`.

### Paper 1

| ID | Name | Tier | Question |
| --- | --- | --- | --- |
| `P1-E00` | Throughput and non-determinism characterisation | T2 | How long is a mission; how noisy is SITL |
| `P1-E01` | Repeated-route learning curve | T1+T2 | Does memory-conditioned routing help at all |
| `P1-E02` | Phantom obstacle | T1+T2 | What one false belief costs |
| `P1-E03` | **Removed obstacle (time-to-forget)** | T1+T2 | **Primary experiment** |
| `P1-E04` | False-report dose–response | T1 | Monotonicity and the shape of the cost |
| `P1-E05` | Change-rate sweep, incl. heterogeneous lifetimes | T1 | Where tuned τ wins and loses |
| `P1-E06` | **Clearance-margin / FOV / range sweep** | T1+T2 | **The coupling result — the paper's core** |
| `P1-E07` | Persistence-filter comparison | T1+T2 | Does the strongest prior method converge here |
| `P1-E08` | Dense-grid comparison incl. bytes and latency | T1+T2 | Why sparse |
| `P1-E09` | Degraded reporter, 1–3 drones | T1+T2 | Containment, and the disjoint-route abstention case |
| `P1-E10` | Ablation of the operator's components | T1 | Attribution of the gain |
| `P1-E11` | Second planner (structurally different) | T1 | Is the coupling a property of avoidance or of `plan_detour` |
| `P1-E12` | Sensor-noise and dropout realism sweep | T1 | Construct validity |
| `P1-E13` | T1↔T2 agreement study | both | Surrogate validity |

### Paper 2

| ID | Name | Tier | Question |
| --- | --- | --- | --- |
| `P2-E00` | Abstract env sanity: random / always-KEEP / always-VERIFY | T1 | Does anything learn |
| `P2-E01` | **Myopic VoI vs learned, single agent** | T1 | **Kill criterion K1** |
| `P2-E02` | Near-optimal offline solver reference | T1 | How much headroom exists at all |
| `P2-E03` | Multi-agent, deferred benefit | T1 | K2 — does the allocation framing earn its place |
| `P2-E04` | Degraded reporter + allocation | T1 | The signature experiment |
| `P2-E05` | Communication degradation | T1 | Stale belief copies |
| `P2-E06` | Generalisation to held-out layouts / rates / faults | T1 | K3 |
| `P2-E07` | Ablation over state features | T1 | The transferable finding |
| `P2-E08` | Safety non-inferiority | T1+T2 | K4 |
| `P2-E09` | SITL confirmation, 3 drones | T2 | K5 |
| `P2-E10` | Verification-spend discipline under a static world | T1 | Does it over-verify when nothing changes |

---

## 3. Statistics

### 3.1 Unit of analysis

> **The run is the unit. A run is one seeded scenario instance × one policy
> variant × one persistent belief store × K missions.**

Missions within a run are serially dependent by construction — that dependence
*is* the phenomenon under study. Counting them as replicates inflates n by ~K
and invalidates every p-value.

- Run-level endpoints: cumulative excess distance, time-to-forget, residual
  belief error, convergence index, total verification spend.
- Mission-level quantities are aggregated to the run before comparison, **or**
  analysed with a mixed-effects model with run as a random effect. Decide which
  per experiment, in the pre-registration, before running.

### 3.2 Sample size

1. **Pilot:** 5 runs per cell on the two most important cells.
2. **Power analysis** from the pilot variance for the smallest effect worth
   detecting. State that effect size in advance — for `P1-E03`, a candidate is
   *a 30% reduction in cumulative excess distance*.
3. **Minimum 30 runs per cell** for primary endpoints. 20 is acceptable only
   with the power analysis printed. Below 20, do not make the claim.
4. If the budget will not cover it: **cut cells, never runs.** An underpowered
   factorial is worth less than a well-powered pair of conditions.

### 3.3 Tests

- Non-parametric: Mann–Whitney U or Brunner–Munzel. No normality assumptions.
- Effect sizes always: Cliff's δ with bootstrap 95% CIs (≥10,000 resamples).
- **One pre-specified primary endpoint per experiment.** Everything else is
  secondary or exploratory, and labelled as such in the text.
- Holm correction across primary endpoints within a paper.
- Report distributions — box or violin plus individual run points. Never a bare
  mean.
- Report the **floor** (`V0-nomem`) and the **ceiling** (`V8-oracle`) in every
  figure where a percentage appears. A percentage improvement without both is
  uninterpretable.

### 3.4 Safety is a non-inferiority test

Never conclude safety from a non-significant difference. Pre-specify:

> *The upper bound of the 95% CI on the difference in near-miss rate between
> treatment and baseline must lie below +10% relative.*

Test with TOST or a one-sided non-inferiority test. Any collision in any
condition is reported in the abstract, regardless of significance.

### 3.5 Sequential looking

Do not peek at accumulating results and stop when a comparison becomes
significant. Fix the run count in advance. If a campaign must be interrupted,
report the planned and achieved counts.

---

## 4. Data splits

Split by **generative parameters**, not by instance.

| Split | Contents | Used for |
| --- | --- | --- |
| Train | Layout families A–C; change rates {low, med}; faults {range bias}; fleet {1,2} | Method tuning **and every baseline's tuning** |
| Validation | Held-out layouts within A–C | Hyper-parameters, τ grid search, VoI parameters, early stopping |
| Test (ID) | Held-out layouts A–C, unseen seeds | Reported results |
| Test (OOD) | Family D; change rate {high}, heterogeneous schedules; faults {dropout, intermittent, latency}; fleet {3} | Generalisation |

Rules:

1. **The test split is opened once.** If it is opened and the method is then
   changed, a new test split must be generated and the first result reported as
   development data.
2. **Every baseline is tuned on train/val with the same budget as the method.**
   Record the budget and report it. An untuned baseline is not a baseline.
3. ID and OOD results are reported separately, never pooled.
4. Held-out fault modes must be *structurally* different, not just
   different-valued (dropout is not "more bias").

---

## 5. Provenance

Every run emits one record conforming to
[`templates/MISSION_RECORD_SCHEMA.md`](templates/MISSION_RECORD_SCHEMA.md),
carrying at minimum:

```
experiment_id · cell_id · run_id · mission_index · policy_variant
git_commit · config_hash · scenario_seed · world_seed · rng_seed
tier (T1|T2) · fleet_size · drone_id
software: px4 · gazebo · ros2 · mavsdk · python · numpy · torch
host: cpu · cores · ram · os · parallel_instances
started_at · finished_at · wall_clock_s
```

A record missing any provenance field is **discarded, not repaired.** Repairing
provenance after the fact is how irreproducible results happen.

Belief-store state is snapshotted at run start and run end, and ground truth is
emitted by the scenario generator, so every belief-quality metric is computable
exactly rather than inferred.

---

## 6. Compute budget — to be recomputed after `P1-E00`

Every number below is conditional on a measurement that has not been taken.

| Quantity | Current status |
| --- | --- |
| T2 wall-clock per mission (incl. start/teardown) | **Unmeasured.** The 3-minute figure is an assumption |
| Parallel T2 instances before degradation | **Unmeasured** |
| SITL run-to-run spread at fixed seed | **Unmeasured** |
| T1 wall-clock per mission | Not built |
| Whether sim time acceleration is safe here | **Unverified** — must not be assumed |

`P1-E00` produces all five. **Do not commit to an experiment matrix before it
exists.** The matrix is then sized as:

```
affordable_cells = (available_hours × parallel_instances)
                   ÷ (runs_per_cell × missions_per_run × hours_per_mission)
```

with `runs_per_cell ≥ 30` held fixed and `cells` as the free variable.

Checkpointing and resume are mandatory: a 30-hour batch will be interrupted, and
a batch that cannot resume will silently produce a biased subset.

---

## 7. Pre-registration

Before any experiment runs, complete
[`templates/EXPERIMENT_PREREGISTRATION.md`](templates/EXPERIMENT_PREREGISTRATION.md)
and commit it. The commit timestamp is the evidence.

The form fixes: hypothesis and direction; primary endpoint; sample size and its
justification; the test; the exclusion rules; and what result would falsify the
hypothesis.

A hypothesis edited after data exists is not a hypothesis. If it must change,
the old one stays in the file with a dated strike-through and a reason.

---

## 8. Exclusion rules — decided now, not later

A run is excluded **only** for these reasons, all machine-detectable:

- Infrastructure failure (PX4 crash, Gazebo crash, service unreachable, host
  OOM) — logged, counted, and the exclusion rate reported per cell.
- Provenance incomplete.
- Geofence or battery-reserve violation (these invalidate the run *and* are
  reported as safety events).

A run is **never** excluded for producing an unexpected result. Exclusion counts
and reasons go in the paper. If any cell's exclusion rate exceeds 5%, the
infrastructure is the finding and must be fixed before the campaign continues.
