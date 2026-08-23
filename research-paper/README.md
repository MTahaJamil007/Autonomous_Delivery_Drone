# research-paper/

The permanent research workspace for turning `DroneProgram` from a final-year
engineering project into a defensible research contribution.

**This folder is the single source of truth for the research journey.** Nothing
important is allowed to exist only in a conversation. Every new paper found,
idea evaluated, novelty question raised, decision made, experiment designed or
result produced gets written here.

---

## Current state — 2026-08-23

| | |
| --- | --- |
| **Phase** | Direction selected; baseline establishment not yet started |
| **Selected direction** | **D1** — Disconfirmation-aware shared obstacle memory for multi-UAV delivery |
| **Working title** | *Learning to Forget: Disconfirmation-Aware Shared Obstacle Memory for Repeated Multi-UAV Delivery Missions* |
| **Novelty assessment** | 7.0 / 10 — strong research opportunity |
| **Blocking prerequisite** | `docs/ACCEPTANCE.md` § 4 (full three-leg mission) has not been run |
| **Experiments run** | **0** |

---

## The one-sentence problem

> The fleet's shared obstacle memory can learn that something is there, and can
> never learn that it is gone.

`obstacle_memory_service/` accumulates obstacle confidence monotonically via a
probabilistic OR, and the only counterweight is a wall-clock decay
(τ = 14 days) that is independent of all evidence. Meanwhile
`avoider_node.py` measures free space (`eff_front_m`) ten times a second and
throws it away. A single spurious report therefore diverts every future route
for months, a removed obstacle is routed around forever, and one drone with a
failing sensor poisons the whole fleet.

Dense occupancy grids solved this decades ago by ray-casting free space into
the map. **Sparse, geo-referenced, service-backed belief stores — what a
bandwidth-constrained fleet actually shares — have no equivalent, and nobody
has studied what that costs at the mission level.**

---

## Reading order

**If you have five minutes:** this file, then `selected-topic.md` § *Final
recommendation*.

**If you are evaluating the research plan:**

1. [`notes/codebase-audit.md`](notes/codebase-audit.md) — what exists, what is
   verified, and **what must not be claimed as novelty**
2. [`literature-review.md`](literature-review.md) — the field, organised by
   which direction it constrains
3. [`novelty-analysis.md`](novelty-analysis.md) — the adversarial pass: every
   idea attacked before a reviewer gets to
4. [`research-directions.md`](research-directions.md) — 10 directions, scored
5. [`selected-topic.md`](selected-topic.md) — top 3 in full, the final
   recommendation, and the roadmap
6. [`research-questions.md`](research-questions.md) — RQ1–RQ5 with falsification
   criteria
7. [`methodology.md`](methodology.md) — the formal setup and the design
   principles
8. [`experiments.md`](experiments.md) — E1–E10, budgets, and the results log
9. [`references.md`](references.md) — every citation, with verification status
10. [`progress-log.md`](progress-log.md) — the dated audit trail

---

## Files

| File | Purpose | Update when |
| --- | --- | --- |
| `README.md` | This orientation page | The phase or selected topic changes |
| `research-directions.md` | All candidate directions, scored and classified | A direction is added, re-scored or re-classified |
| `literature-review.md` | The field, by area, with saturation judgements | A paper is found or read |
| `novelty-analysis.md` | Adversarial novelty defence per direction | Novelty is questioned or prior work found |
| `selected-topic.md` | Top 3 in detail + final recommendation + roadmap | The selection or the plan changes |
| `research-questions.md` | RQ1–RQ5 with falsification criteria | A question is added, refined or answered |
| `methodology.md` | Formal setup, design principles, threats to validity | Any methodological decision |
| `experiments.md` | Experiment design, budgets, pre-registrations, results | An experiment is designed or run |
| `references.md` | Bibliography with verification status | A citation is added or verified |
| `progress-log.md` | Dated decision trail | Every substantive session |
| `notes/` | Working notes, paper summaries, derivations | Freely |
| `papers/` | PDFs of key references | A paper is downloaded |
| `results/` | Raw mission JSON, processed CSVs | An experiment produces data |
| `figures/` | Generated plots for the paper | Analysis is run |

---

## Working rules

1. **Nothing gets a ✅ without evidence.** Inherited from the parent repo's
   `STATUS.md`, and it applies here too: no experiment result appears in
   `experiments.md` until it has actually been produced.
2. **Pre-register hypotheses.** Write the expected direction before running,
   and do not edit it afterwards.
3. **Cite honestly.** `references.md` marks which entries have been *read* and
   which are only search hits. Do not cite an unread paper in a submission.
4. **Attack your own idea first.** `novelty-analysis.md` exists so that the
   reviewer's objection is already answered on the page.
5. **Negative results are results.** The fallback plan in `selected-topic.md`
   depends on this being genuine.
6. **Separate engineering from research.** `notes/codebase-audit.md` § 5 lists
   what the project already does; none of it is a contribution.

---

## Immediate next actions

1. **Run `docs/ACCEPTANCE.md` § 4** — the full three-leg mission. Everything is
   blocked on this. Expect 2–3 defects on first flight.
2. Run § 8 (camera signs). No landing number means anything until it passes.
3. Run § 10 (detour + memory, two runs). This is experiment E1 in miniature.
4. Read the six key papers listed in `progress-log.md`, and record notes in
   `notes/`. **Week-5 gate: confirm no closer prior work exists.**
5. Formalise the disconfirmation operator on paper (`methodology.md` § 2 is the
   current draft) before writing any code.

---

## What this is not, yet

There is no paper draft, and there should not be one. The sequence is:
understand the system → survey the field → find where genuine novelty exists →
establish baselines → build → experiment → *then* write. Writing earlier
produces a paper defending a position that has not been tested.
