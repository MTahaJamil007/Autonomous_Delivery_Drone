# Experiment plan

Status: **design only — nothing run.** Following the repository's own
convention (`STATUS.md`, `docs/ACCEPTANCE.md`): *no result is recorded here
until it has actually been produced.* Every row below is `PLANNED`.

**Pre-registration rule.** Before running an experiment, fill in its
*Hypothesis* and *Expected direction*. After running it, fill in *Result*
without editing the hypothesis. A hypothesis edited after seeing data is not a
hypothesis.

---

## Phase 0 — Baseline establishment (prerequisite, NOT research)

These are `docs/ACCEPTANCE.md` procedures that must pass before any experiment
below is meaningful. They are not contributions; they are the ground the
experiments stand on.

| ID | Procedure | Blocks | Status |
| --- | --- | --- | --- |
| P0.1 | ACCEPTANCE § 4 — full three-leg mission | **Everything** | PLANNED |
| P0.2 | ACCEPTANCE § 8 — camera sign confirmation | Any landing metric | PLANNED |
| P0.3 | ACCEPTANCE § 3 — fail-closed on a killed avoider | E9 | PLANNED |
| P0.4 | ACCEPTANCE § 10 — detour + memory, two runs | E1 | PLANNED |
| P0.5 | ACCEPTANCE § 11 — three-drone fleet | E6 | PLANNED |
| P0.6 | Calibrate `BATTERY_ENERGY_PER_M_PCT` (CALIBRATION § 5) | E8, energy metrics | PLANNED |
| P0.7 | Write `scripts/batch_missions.py` + structured logging | All | PLANNED |
| P0.8 | Write `sim/scenarios/` parameterised generator | All | PLANNED |

> `STATUS.md` predicts 2–3 first-flight defects at P0.1 "of the same kind as
> finding F5 — bugs that only exist once the code actually runs". Budget for
> them; they are planned for, not a setback.

---

## Policy variants (the columns of every experiment)

| Key | Variant |
| --- | --- |
| `V0-nomem` | No shared memory. Direct route + reactive avoidance only. **The floor.** |
| `V1-base` | Current system: accumulate-only, prob-OR fusion, wall-clock τ = 14 d. **The primary baseline.** |
| `V2-tau*` | As V1 with τ grid-searched per scenario. **The critical control.** |
| `V3-grid` | Dense occupancy grid with log-odds and ray-casting. **The representation control.** |
| `V4-disc` | V1 + disconfirmation operator |
| `V5-disc+ev` | V4 + evidence-driven forgetting |
| `V6-full` | V5 + source reliability + confidence-aware clearance |
| `V7-oracle` | Perfect obstacle knowledge. **The ceiling.** |

---

## Primary experiments

### E1 — Repeated-route learning curve

**Question:** does shared memory help at all, at scale?
**Setup:** fixed route through `great_wall`. 30 sequential missions, memory
persists. Variants `V0`, `V1`, `V7`. 5 seeds.
**Hypothesis:** *(pre-register before running)* V1 converges to near-V7 path
length within ~3 missions; V0 is flat.
**Metrics:** dodges/mission, escalations/mission, path length ÷ geodesic,
duration, energy, min clearance.
**Result:** — PLANNED

### E2 — Phantom obstacle

**Question:** what does one false report cost?
**Setup:** inject 1 spurious obstacle on the direct route at mission 1. 30
missions. `V1`, `V2`, `V4`, `V6`, `V7`.
**Hypothesis:** V1 routes around the phantom for all 30 missions; V4/V6 stop
within a small number of traversals.
**Metrics:** **unnecessary-avoidance rate**, excess path length, missions until
the phantom stops affecting routing.
**Result:** — PLANNED

### E3 — Removed obstacle (time-to-forget)

**Question:** how fast does the fleet unlearn a genuine obstacle that is gone?
**Setup:** real obstacle present missions 1–10, removed at 11, 30 missions
total. `V1`, `V2`, `V5`, `V6`, `V7`.
**Hypothesis:** V1's time-to-forget is bounded only by τ (i.e. never, within the
experiment); V5/V6 recover in a small number of traversals.
**Metrics:** **time-to-forget**, excess path length after removal, min clearance
during re-traversal.
**Result:** — PLANNED

### E4 — False-report-rate sweep (dose–response)

**Setup:** spurious report probability ∈ {0, 0.05, 0.10, 0.20, 0.40} × `V1`,
`V4`, `V6`, `V7`. 30 missions per cell. **20 cells, 600 missions.**
**Hypothesis:** V1 degrades monotonically; V4/V6 are much flatter.
**Metrics:** unnecessary-avoidance rate, path length, belief-store precision.
**Result:** — PLANNED

### E5 — Environment change-rate sweep (the crossover)

**Setup:** obstacle lifetime ∈ {1, 5, 20, ∞} missions × `V1`, `V2`, `V5`, `V7`.
30 missions per cell. **16 cells, 480 missions.**
**Hypothesis (stated in advance):** tuned τ (V2) is *competitive* under uniform
change rates; evidence-driven decay (V5) wins where change is spatially
heterogeneous, because one τ cannot serve both permanent and transient
obstacles. **Include a heterogeneous-lifetime cell specifically to test this.**
**Result:** — PLANNED

### E6 — Poisoned source

**Setup:** 3-drone fleet, one with −40% range bias. `V5` (no reliability) vs
`V6` (with). Also sweep healthy:degraded ratio 2:1 and 1:1.
**Hypothesis:** V6 with one degraded drone ≈ all-healthy; V5 degrades.
**Must also test:** a healthy drone flying a *disjoint* route, to confirm the
estimator abstains rather than penalising (RQ4's known hard case).
**Result:** — PLANNED

### E7 — Ablation

**Setup:** V6 minus each of {contradiction weighting → binary disconfirmation,
evidence decay → wall-clock, source reliability → uniform, confidence-aware
clearance → constant margin, per-record attribution → normalised attribution}.
Run on E2, E3, E4 scenarios.
**Purpose:** attribute the gain. If one component provides all of it, say so.
**Result:** — PLANNED

---

## Secondary experiments

### E8 — Cost-aware escalation *(secondary; cut first if time slips)*

**Setup:** fixed 12 s timer vs futility predictor vs energy-priced decision,
under battery budgets {loose, tight}. Requires P0.6.
**Metrics:** delivery success rate, escalations, energy at completion, aborts.
**Result:** — PLANNED

### E9 — Degradation stress (imports Topic 3)

**Setup:** V6 under vision dropout, LiDAR dropout, GPS noise, wind. 4 modes × 4
magnitudes × 20 missions.
**Purpose:** robustness section, and a check that disconfirmation does not
amplify sensor faults into belief corruption.
**Result:** — PLANNED

### E10 — Physical validation *(optional, high value)*

**Setup:** one drone, one ArUco pad, one movable barrier. Place barrier → 3
flights → remove → 3 flights. `V1` vs `V6`.
**Purpose:** external validity. Even n = 6 is worth a figure.
**Result:** — PLANNED

---

## Budget

| | Missions | Est. wall-clock (headless, ~3 min each) |
| --- | ---: | ---: |
| E1 | 450 | 23 h |
| E2 | 600 | 30 h |
| E3 | 750 | 38 h |
| E4 | 600 | 30 h |
| E5 | 480 | 24 h |
| E6 | 360 | 18 h |
| E7 | 450 | 23 h |
| E8 | 240 | 12 h |
| E9 | 320 | 16 h |
| **Total** | **≈ 4,250** | **≈ 214 h** |

Reducible to ~2,000 missions / ~100 h by cutting E7 to the three most important
ablations and E1 to 3 seeds. Runs overnight in batches on the existing host.
**Checkpoint and resume are mandatory** — a 30 h batch will be interrupted.

---

## Figures the paper needs

1. Learning curves: path length vs mission index, per variant (E1).
2. Unnecessary-avoidance rate vs false-report rate (E4). *The money figure.*
3. Time-to-forget, box plots per variant (E3).
4. Change-rate crossover: V2 vs V5 across obstacle lifetime (E5).
5. Poisoned-source containment (E6).
6. Ablation waterfall (E7).
7. Safety guardrail: min-clearance distributions per variant, all experiments.
8. Belief-store calibration: reliability diagram, stated confidence vs empirical
   presence.
9. Cost comparison: bytes/mission and rows, sparse vs dense grid (V6 vs V3).
10. System architecture diagram, marking exactly which blocks are new.

---

## Results log

*(One dated row per completed run. Nothing gets written here that was not
actually executed — same rule as `docs/ACCEPTANCE.md` § 9.)*

| Date | Experiment | Variants | Missions | Outcome | Notes |
| --- | --- | --- | --- | --- | --- |
| — | — | — | — | — | *nothing run yet* |
