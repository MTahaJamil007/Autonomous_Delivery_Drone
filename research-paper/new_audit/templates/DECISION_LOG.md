# Decision log

Copy to `research-paper/new_audit/DECISION_LOG.md` and append to it. Newest
entry at the top.

**What goes here:** gate outcomes, scope changes, claim changes, baseline
changes, metric changes, venue changes, authorship decisions, and anything a
reviewer or a future you would ask *"why did they do that?"* about.

**What does not:** routine progress. `progress-log.md` already covers that.

---

## Entry template

### YYYY-MM-DD — one-line decision

| Field | Value |
| --- | --- |
| **Type** | gate / scope / claim / baseline / metric / venue / authorship / other |
| **Trigger** | The observation or result that forced the decision |
| **Options considered** | At least two. If there was only one, it was not a decision |
| **Decision** | |
| **Rationale** | |
| **Consequence** | What changes downstream — documents, experiments, timeline |
| **Reversible?** | If yes, under what condition would you reverse it |
| **Recorded by** | |

---

## Standing entries

### 2026-09-13 — Paper 1's novelty claim moved off the mechanism

| Field | Value |
| --- | --- |
| **Type** | claim |
| **Trigger** | Literature search found persistence filters (Rosen et al., ICRA 2016), FreMEn (T-RO 2017), clique change detection (2020), Perpetua (2025) — all doing evidence-driven persistence over sparse maps |
| **Options considered** | (a) keep the mechanism claim and argue the sparse-geo case differs; (b) move the claim to the belief→route→observation coupling and mission-level evaluation; (c) abandon the direction |
| **Decision** | (b) |
| **Rationale** | (a) loses to any reviewer who knows the persistence-filter literature, and it is a twelve-year-old result with public code. (c) discards a genuinely under-explored setting. (b) produces a claim that survives the prior work being *in the paper and running* |
| **Consequence** | `01_paper1_specification.md` rewritten; `V3-pf` added as a required baseline; `P1-E06` (margin/FOV/range sweep) becomes the core experiment; `novelty-analysis.md` § D1's gap statement is superseded |
| **Reversible?** | Only if reading the three papers in full shows they do not cover the sparse-belief case. Check during Gate A |
| **Recorded by** | pre-implementation audit |

### 2026-09-13 — Source reliability demoted from contribution to ablation

| Field | Value |
| --- | --- |
| **Type** | claim |
| **Trigger** | Inter-robot trust (Pierson & Schwager 2013), resilient fusion with malicious robots, uncertainty-weighted multi-robot mapping all exist |
| **Options considered** | (a) keep as headline; (b) demote to ablation component; (c) cut entirely |
| **Decision** | (b) |
| **Rationale** | The sentence "no published merging rule estimates per-source reliability" is false and would be fatal. The abstention case for disjoint routes remains a real, specific engineering result worth reporting |
| **Consequence** | `V5-disc+rel` stays as a variant; `P1-E09` stays; the abstract makes no reliability claim |
| **Reversible?** | No |
| **Recorded by** | pre-implementation audit |

### 2026-09-13 — Cost-aware escalation (old D3 / RQ5 / E8) cut from Paper 1

| Field | Value |
| --- | --- |
| **Type** | scope |
| **Trigger** | `When2Replan-2023` occupies the arbitration-timing question; the energy model is a placeholder (`config.py:398`) |
| **Options considered** | (a) keep as second contribution; (b) cut |
| **Decision** | (b) |
| **Rationale** | It dilutes a paper whose strength is one sharp claim, it invites a comparison the project cannot win, and it depends on an energy calibration that does not exist |
| **Consequence** | RQ5 and E8 removed; `BATTERY_ENERGY_PER_M_PCT` calibration leaves the critical path |
| **Reversible?** | Yes, as a later separate paper — never as a section of this one |
| **Recorded by** | pre-implementation audit |

### 2026-09-13 — Two-tier evaluation (T1 abstract + T2 SITL) adopted

| Field | Value |
| --- | --- |
| **Type** | scope |
| **Trigger** | Statistically valid design (runs, not missions, as replicates) implies ~30,000 SITL missions ≈ 1,500 h for Paper 1 alone, on an unmeasured 3 min/mission assumption |
| **Options considered** | (a) all-SITL with fewer cells; (b) all-SITL with fewer runs; (c) two-tier with a T1↔T2 agreement study |
| **Decision** | (c) |
| **Rationale** | (b) is invalid — underpowered results are worse than none. (a) sacrifices the factorial structure the claims need. (c) preserves both, and T1 doubles as Paper 2's training environment |
| **Consequence** | T1 simulator added to Phase A; `P1-E13` agreement study added; the T1 surrogate's validity becomes a reported result |
| **Reversible?** | Yes, if `P1-E00` shows SITL throughput is far better than assumed |
| **Recorded by** | pre-implementation audit |

### 2026-09-13 — Paper 2 gains a pre-committed kill criterion (K1)

| Field | Value |
| --- | --- |
| **Type** | gate |
| **Trigger** | Learned value-of-information for navigation under an incomplete map already exists (arXiv:2403.03269); the state space is small and interpretable, which is where classical decision theory is strongest |
| **Options considered** | (a) proceed and see; (b) pre-commit to beating a myopic VoI baseline before writing an RL paper |
| **Decision** | (b) |
| **Rationale** | Failing K1 at week 34 costs seven weeks. Discovering the same thing after PX4 integration costs eight months. The fallback (a classical cost-aware verification paper) is strong and already half-built |
| **Consequence** | `P2-E01` and `P2-E02` precede all training; `02_paper2_specification.md` § 6 |
| **Reversible?** | No |
| **Recorded by** | pre-implementation audit |

### 2026-09-13 — Repository restructuring rejected

| Field | Value |
| --- | --- |
| **Type** | scope |
| **Trigger** | The plan proposed renaming the flight stack into `drone_system/`, `navigation/`, `obstacle_memory/`, etc. |
| **Options considered** | (a) restructure as proposed; (b) add `research/` alongside and leave flight code untouched |
| **Decision** | (b) |
| **Rationale** | (a) breaks imports, 126 tests and 7 repo gates in exchange for nothing a reviewer sees. (b) gets the same separation additively, with a gate forbidding flight code from importing research code |
| **Consequence** | Layout in `08_reproducibility_standard.md` § 1; one new repo gate |
| **Reversible?** | Yes, but there will never be a good reason |
| **Recorded by** | pre-implementation audit |

*(Append new entries above this line, newest first.)*
