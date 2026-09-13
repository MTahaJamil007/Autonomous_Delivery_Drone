# 08 — Reproducibility standard

The target: **another researcher, with only the public release, reproduces every
figure.** Not "could in principle" — actually does, because someone tried it
before submission.

This is also the artefact that makes the project worth more than its papers: a
reproducible multi-UAV persistent-belief evaluation platform is citable,
reusable, and demonstrable in a master's application in a way a PDF is not.

---

## 1. Repository layout

Additive only. Nothing existing moves — see
[`03_two_paper_strategy_review.md`](03_two_paper_strategy_review.md), Phase 0.2.

```
Autonomous_Delivery_Drone/
├── drone_agent/ global_planner/ obstacle_memory_service/    ← flight code, untouched
├── perception/ fleet_dispatch/ sim/ world/ tests/           ← untouched
│
├── research/
│   ├── scenarios/        parameterised world + belief-store generators
│   ├── envs/             T1 abstract simulator (= MemoryManagementEnv)
│   ├── baselines/        persistence_filter.py · dense_grid.py · voi.py · thresholds.py
│   ├── experiments/      P1_E01_learning_curve.py ... one module per ID
│   ├── evaluation/       metrics.py · statistics.py · figures.py
│   ├── configs/          one YAML per cell; hashed into every record
│   └── tests/            research code has tests too
│
├── results/              raw records, git-ignored, archived externally
├── models/               trained policies + exact training configs
└── research-paper/
    ├── new_audit/  paper1/  paper2/
```

**Repo gate to add:** flight modules must not import `research/`. Enforce it
alongside the existing seven gates, so the rule survives being forgotten.

---

## 2. One command per figure

```bash
# regenerate one experiment's data
python -m research.experiments.P1_E03_removed_obstacle --config research/configs/P1_E03.yaml --runs 30

# regenerate every figure from archived data
python -m research.evaluation.figures --paper 1 --data results/ --out figures/
```

Rules:

- A figure is **never** produced by a manual step, a notebook cell run out of
  order, or a hand-edited file. Notebooks may explore; they may not generate
  anything that appears in the paper.
- Every figure script prints the provenance of the data it consumed — commit,
  config hash, run count, date — and that provenance appears in the figure
  caption or the supplementary material.
- Figures regenerate from **archived raw data** without re-running simulation, so
  a reviewer without a PX4 install can still check the analysis.

---

## 3. Determinism

| Layer | Control |
| --- | --- |
| Scenario generation | Seeded; identical seed ⇒ identical world and identical initial belief store |
| Policy variants | Same binary, different config; no separate branches for baseline and treatment |
| Numerical | `numpy`, `torch`, Python `random` seeded per run and recorded |
| SITL | **Not deterministic.** Measure the spread at fixed seed (`P1-E00`) and report it as the noise floor |
| Software | Pinned versions in a container image; image digest recorded |

**PX4 pinning:** the stack currently records `v1.17.0-alpha1-1225`. An alpha
build is not a pin. Either move to a stable tag or record and archive the exact
commit SHA plus build flags, and state it in the reproducibility section.

---

## 4. Data release

Release a **dataset**, not a log dump. The difference is five things:

1. **Records** — one JSON per mission, schema in
   [`templates/MISSION_RECORD_SCHEMA.md`](templates/MISSION_RECORD_SCHEMA.md).
2. **A documented schema** with units and semantics for every field.
3. **Defined splits** — train / validation / test-ID / test-OOD, released as the
   split files themselves, not as a description.
4. **Baseline scripts** that reproduce the reported baseline numbers from the
   released data.
5. **A datasheet** — what was collected, how, by which software versions, with
   what known limitations and intended uses.

Only with all five is the word *benchmark* defensible. Until then it is a
dataset, and calling it a benchmark invites a reviewer to point out that it is
not one.

Archive with a **DOI** (Zenodo or an institutional repository) and cite that DOI
in both papers. Include the ground truth from the scenario generator — without it
the belief-quality metrics cannot be recomputed.

---

## 5. Licensing — decide before the first release

| Item | Issue | Action |
| --- | --- | --- |
| **Persistence-filter reference implementation** | **GPL-3.** Linking it makes the whole repository GPL-3 | **Re-implement from the paper**, or keep it in a separate, clearly-licensed comparison repository. Decide before writing `V3-pf` |
| Vendored `sim/models/x500_delivery` and world assets | Derived from PX4 ecosystem assets, which carry their own licences | Audit provenance; preserve upstream notices and attribution |
| PX4 / ROS 2 / Gazebo | Permissive, but attribution matters | Cite and attribute in the release |
| Research code | Choose deliberately | A permissive licence (BSD-3 / Apache-2.0) maximises reuse and matches the ecosystem; **only if institutional IP policy allows** — check first |
| Dataset | Needs its own licence | CC-BY-4.0 is the usual choice for research data |

R29 and R30 in the risk register. A licence problem discovered at release time
is much more expensive than one decided at design time.

---

## 6. Provenance in every record

Non-negotiable fields (full schema in `templates/`):

```
experiment_id · cell_id · run_id · mission_index · policy_variant · tier
git_commit · git_dirty · config_hash · container_digest
scenario_seed · world_seed · rng_seed
px4 · gazebo · ros2 · mavsdk · python · numpy · torch
host_cpu · cores · ram · os · parallel_instances
started_at · finished_at · wall_clock_s
```

`git_dirty = true` invalidates the run. Uncommitted changes mean the run cannot
be reproduced, and no amount of care afterwards fixes that.

---

## 7. Pre-submission reproducibility check

Two weeks before submission, on a **clean machine**, by someone who did not
build it:

1. Clone the public repository.
2. Pull the container image by digest.
3. Download the archived dataset by DOI.
4. Run the figure-generation command.
5. Diff the output against the figures in the manuscript.

Any difference that is not explainable by SITL non-determinism is a bug in the
release. Record the outcome in the decision log; fix before submitting.

---

## 8. What goes public, and when

| Artefact | When | Why |
| --- | --- | --- |
| Research code | At submission | Reviewers increasingly expect it; anonymise if the venue is double-blind |
| Dataset + DOI | At submission | The claim is not checkable without it |
| Trained policies + training configs | At Paper 2 submission | Weights without configs are not reproducible |
| Container image | At submission | The only reliable way to pin PX4/Gazebo/ROS |
| Preprint | After internal + supervisor review, before or at submission — **check the venue's current policy first** | Visibility, and a timestamped record of priority |
| `research-paper/` working notes | Optional | The audit trail is a strength; publish it if the supervisor agrees |

Anonymisation warning for double-blind venues: a public GitHub repository with
your name on it, linked from the manuscript, breaks anonymity. Use an anonymised
mirror during review if the venue requires it.
