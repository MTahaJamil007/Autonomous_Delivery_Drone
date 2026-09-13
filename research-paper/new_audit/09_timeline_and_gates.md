# 09 — Timeline and gates

A gate is a **decision point with a written rule**, not a milestone. At each one
the honest options are: proceed, cut scope, or change direction. "Work harder" is
not an option, and neither is "carry on and hope".

Weeks are relative to the start of full-time work. The plan's 7–8 months for two
submissions assumes no engineering surprises on a system whose central mission
has never been flown. This schedule has the same content with the surprises
priced in.

---

## Phase A — Foundation (weeks 1–6)

Two tracks in parallel. **The literature track must not slip behind the
engineering track**; it is the only activity that can end the project cheaply.

### Engineering track

| Week | Work | Output |
| --- | --- | --- |
| 1 | ACCEPTANCE § 4 three-leg mission. Expect defects | `engineering-failures.md` entries |
| 1–2 | § 8 camera signs, § 3 fail-closed, § 6 supervisor abort | Acceptance log |
| 2 | **`P1-E00`** — throughput, parallel scaling, fixed-seed spread | **The numbers every budget depends on** |
| 2–3 | § 11 three-drone fleet | The "Multi-UAV" claim, or its retraction |
| 3–4 | **F8** — memory-conditioned pre-planning, feature-flagged + tests | The phenomenon becomes observable |
| 4 | **F9** — sectorised free-space channel + tests | The observation model exists |
| 4–5 | Structured mission records with full provenance | `templates/MISSION_RECORD_SCHEMA.md` realised |
| 5–6 | Scenario generator (S1–S8), belief-store seeding | Experiments become expressible |
| 5–6 | T1 abstract simulator reusing the real planner + belief store | The compute problem becomes tractable |

### Literature track

| Week | Work |
| --- | --- |
| 1–2 | Read papers 1–3 from `05` § 6 (persistence filter, FreMEn, learned VoI) |
| 3–4 | Read papers 4–6; run every search in `05` § 7 and record the protocol |
| 5 | Rewrite the gap paragraph (`PRE_IMPLEMENTATION_AUDIT.md` Q5) against what was actually read |
| 6 | Update `references.md` with `READ-<date>` tags and one-paragraph notes per paper |

### ▣ GATE A — end of week 6

| # | Condition | If failed |
| --- | --- | --- |
| A1 | § 4 passes; a mission completes end to end | **Stop research work.** Fix the system. Re-gate in 3 weeks. If still failing at week 12, reduce to a two-leg mission and state the simplification |
| A2 | § 11 passes, or the fleet claim is formally reduced | Retitle both papers to the fleet size that actually flies. Do not claim three drones from two |
| A3 | F8 + F9 landed, tested, flagged | Blocking — nothing downstream is meaningful |
| A4 | Throughput measured; budget recomputed | Blocking — every plan below is conditional on it |
| A5 | Papers 1–3 read; the gap paragraph survives | **Change direction.** Pivot to the evaluation-only paper, or re-scope from what the reading showed |

---

## Phase B — Baselines and the first phenomenon (weeks 7–12)

| Week | Work |
| --- | --- |
| 7–8 | Baselines: `V0`, `V1`, `V2-tau*`, `V8-oracle`; batch runner with resume |
| 8–9 | **`V3-pf` persistence filter**, re-implemented from the paper (licence — `08` § 5) |
| 9–10 | `V7-grid` dense occupancy grid, same planner interface, byte accounting |
| 10–11 | Pilot: 5 runs/cell on `P1-E03` and `P1-E06`. Power analysis. Size the matrix |
| 11–12 | Pre-register `P1-E01`–`P1-E13`. Run `P1-E01`, `P1-E03`, `P1-E06` in T1 |

### ▣ GATE B — end of week 12 — the scientific gate

| # | Condition | If failed |
| --- | --- | --- |
| B1 | With F8 in place, memory changes routing, and a removed obstacle produces measurable excess cost under `V1` | **No phenomenon.** Re-examine the scenario design; if it still does not appear, the direction is dead — pivot to the characterisation paper |
| B2 | `P1-E06` shows residual belief error rising with clearance margin — the coupling | The paper's core result is absent. Fall back to the mission-level evaluation contribution alone |
| B3 | `V3-pf` behaviour is characterised (converges or not) | Blocking — the strongest baseline must be understood before the campaign |
| B4 | Power analysis supports the planned matrix | Cut cells, never runs |

**This is the point of no return.** Everything before it is reversible; the
campaign after it is months of compute.

---

## Phase C — Paper 1 campaign and writing (weeks 13–26)

| Week | Work |
| --- | --- |
| 13–18 | T1 campaign: `P1-E01`–`P1-E12`, checkpointed, randomised cell order |
| 16–20 | T2 confirmation subset + `P1-E13` agreement study |
| 18–21 | Analysis: run-level statistics, effect sizes, CIs, non-inferiority safety tests |
| 15–24 | Writing, in parallel. Method and protocol first — they are stable earliest |
| 24–25 | Internal hostile review (`07`) + independent sanity review |
| 25–26 | Supervisor review; venue requirements verified **against live guidelines**; submit |

### ▣ GATE C — week 26

| # | Condition | If failed |
| --- | --- | --- |
| C1 | Primary endpoint has a CI that supports the claim | Report the null result; reframe as characterisation. **Still submit** |
| C2 | Safety non-inferiority holds | Report the trade-off as the finding. Do not claim an improvement |
| C3 | Independent reviewer answers "what is new?" in 60 s | Rewrite the abstract until they can |
| C4 | Figures regenerate on a clean machine | Fix before submitting |

---

## Phase D — Paper 2 feasibility (weeks 27–34)

Deliberately cheap and deliberately early. Nothing here touches PX4.

| Week | Work |
| --- | --- |
| 27 | Tag `paper1-submission-v1.0`; branch `paper2-verification` |
| 27–28 | RL fundamentals against the real problem, not a course |
| 28–30 | `MemoryManagementEnv` = T1 with the verification action. `P2-E00` |
| 30–31 | **Myopic VoI baseline** + near-optimal offline reference (`P2-E02`) |
| 31–33 | PPO, stages 0–1. `P2-E01` |
| 33–34 | Stage 2 multi-agent, deferred benefit. `P2-E03` |

### ▣ GATE D — week 34 — the kill gate

| # | Condition | If failed |
| --- | --- | --- |
| D1 (**K1**) | Learned policy beats myopic VoI with non-overlapping bootstrap CIs | **Stop the RL paper.** Write the classical cost-aware verification paper instead. This is a planned, respectable outcome |
| D2 (**K2**) | The margin over VoI grows from single-agent to multi-agent | The allocation framing is not doing work. Re-scope or stop |
| D3 | The headroom between VoI and the offline reference is large enough to be worth chasing | If the gap is tiny, no method can win by much — stop early |

Reaching D1 and failing it at week 34 costs **seven weeks**. Discovering the same
thing at week 60, after full PX4 integration, costs eight months. That asymmetry
is the entire reason the gate exists.

---

## Phase E — Paper 2 campaign and writing (weeks 35–56)

| Week | Work |
| --- | --- |
| 35–40 | Stages 3–4: degraded reporter, comms loss, generalisation (`P2-E04`–`P2-E06`) |
| 40–44 | Ablations (`P2-E07`) and safety non-inferiority (`P2-E08`) |
| 44–48 | SITL confirmation, 3 drones (`P2-E09`, K5) |
| 46–52 | Writing in parallel |
| 52–54 | Hostile review; independent review; reproducibility check |
| 54–56 | Supervisor review; submit |

### ▣ GATE E — week 56

Same structure as Gate C, plus: K3 (generalisation), K4 (safety), K5 (T1↔T2
ordering preserved).

---

## Summary

| Milestone | Optimistic | Realistic | If Gate A or B forces a pivot |
| --- | --- | --- | --- |
| Foundation complete | wk 6 | wk 8 | — |
| Scientific gate passed | wk 12 | wk 16 | Direction changes here |
| **Paper 1 submitted** | **wk 26** | **wk 30–34** | Characterisation paper, similar date |
| Paper 2 kill gate | wk 34 | wk 40 | — |
| **Paper 2 submitted** | **wk 56** | **wk 62–68** | Classical verification paper, ~wk 50 |

**≈ 7 months to Paper 1, ≈ 15 months to Paper 2**, full time, with slack for two
engineering surprises and one literature setback. Publication dates depend on
editors and revision rounds and should never be promised.

The optimistic column is achievable only if § 4 flies in week 1 and the
literature reading produces no surprises. Plan against the realistic column and
treat the optimistic one as upside.

---

## The rules that keep the schedule honest

1. **A gate is passed or failed on the written criterion**, not on how much work
   went in. Record the outcome in `templates/DECISION_LOG.md` on the day.
2. **When time runs short, cut scope, not runs.** An underpowered experiment is
   worth less than no experiment, because it produces a claim nobody should
   believe — including you.
3. **Reading is never the thing that slips.** It is the cheapest way to end the
   project, and ending early is a good outcome when the alternative is six
   months toward a rejection.
4. **Negative results ship.** Two of the five gates above have "report it and
   submit anyway" as their failure branch. That is deliberate, and it is what
   makes starting safe.
