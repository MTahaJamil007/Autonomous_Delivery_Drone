# `new_audit/` — the pre-implementation research contract

Created **2026-09-13**. Supersedes nothing in `research-paper/`; it *constrains*
it. The 2026-08-23 folder chose a direction. This folder decides whether that
direction survives contact with (a) the code as it actually is, (b) the prior
art as it actually is, and (c) the statistics the claims will actually need.

The governing instruction for this folder was:

> *"I need this research paper idea accurate without any future issues."*

So every statement here is marked with how it was established:

| Tag | Meaning |
| --- | --- |
| `VERIFIED-CODE` | Read in this repository; a `file:line` is given |
| `VERIFIED-WEB` | A live source was retrieved on 2026-09-13; URL given |
| `INHERITED` | Taken from the 2026-08-23 research folder, **not** re-verified |
| `JUDGEMENT` | An analytical conclusion, argued not measured |
| `UNVERIFIED` | Asserted somewhere in this project with no evidence behind it |

Nothing in this folder is an instruction to change code. No code was changed
producing it.

---

## The headline

**The research direction is sound. Three of its load-bearing claims are not.**

1. The system **cannot currently exhibit the phenomenon the paper is about.**
   Shared memory is fetched but never used to plan a route; it is consulted only
   *after* a 12-second reactive-avoidance escalation. A phantom obstacle in the
   database therefore costs nothing, and "time-to-forget" is undefined.
   → `10_codebase_evidence.md` § F8. **This is the single most important finding.**
2. The mechanism claimed as novel — negative evidence and forgetting in a
   **sparse** map — **already exists in the literature** (persistence filters,
   FreMEn, clique change detection, multi-hypothesis persistence). The
   2026-08-23 literature review misses this entire subfield.
   → `05_literature_verification.md` § 2.
3. Per-source reliability weighting in multi-robot mapping **also already
   exists** (inter-robot trust, resilient fusion with malicious agents). The
   sentence *"no published merging rule for shared robot maps estimates and
   applies per-source reliability"* is false and would be fatal in review.
   → `05_literature_verification.md` § 3.

**And one finding that makes the project stronger than it was.** The
belief→route→observation loop in a *delivery* fleet is closed in a way it is not
in a *mapping* robot: a drone that successfully avoids a believed obstacle never
generates the observation that would refute it. Passive disconfirmation is
therefore **structurally unable** to clear a belief it is successfully avoiding.
That is a real, sharp, unclaimed result — and it is the honest reason Paper 2
(active verification) has to exist. → `01_paper1_specification.md` § 3.

---

## Reading order

**Ten minutes:** `EXECUTIVE_SUMMARY.md`.

**Deciding whether to start:** `PRE_IMPLEMENTATION_AUDIT.md` — the 24 questions,
answered, plus the go/no-go rule. This is the contract.

**Building it:**

| # | File | What it settles |
| --- | --- | --- |
| — | [`PRE_IMPLEMENTATION_AUDIT.md`](PRE_IMPLEMENTATION_AUDIT.md) | The research contract. A–E, Q1–Q24, and the go/no-go gate |
| 01 | [`01_paper1_specification.md`](01_paper1_specification.md) | Paper 1 re-scoped: claim, method, baselines, what must **not** be claimed |
| 02 | [`02_paper2_specification.md`](02_paper2_specification.md) | Paper 2 re-scoped: why learning is justified, and the criteria that kill it |
| 03 | [`03_two_paper_strategy_review.md`](03_two_paper_strategy_review.md) | Phase-by-phase response to the 26-phase plan: keep / change / drop |
| 04 | [`04_experiment_protocol.md`](04_experiment_protocol.md) | Experiment IDs, independence, power, splits, statistics, compute budget |
| 05 | [`05_literature_verification.md`](05_literature_verification.md) | Prior art the old review missed; citation-integrity protocol |
| 06 | [`06_risk_register.md`](06_risk_register.md) | Every risk with a trigger and a named mitigation |
| 07 | [`07_reviewer_rebuttal_pack.md`](07_reviewer_rebuttal_pack.md) | Hostile review, both papers, with the answers written in advance |
| 08 | [`08_reproducibility_standard.md`](08_reproducibility_standard.md) | Provenance, one-command figures, licensing, dataset release |
| 09 | [`09_timeline_and_gates.md`](09_timeline_and_gates.md) | Revised schedule with hard gates and abort rules |
| 10 | [`10_codebase_evidence.md`](10_codebase_evidence.md) | What the code actually does, `file:line`, and F8 |
| — | [`templates/`](templates/) | Pre-registration, mission-record schema, failure log, decision log |

---

## Status board

| Item | State |
| --- | --- |
| Direction (sparse shared belief in repeated multi-UAV delivery) | **Confirmed, re-scoped** |
| Paper 1 claim as written 2026-08-23 | **Rejected** — see `01` § 1 for the replacement |
| Paper 2 claim as written in the plan | **Rejected** — see `02` § 1 for the replacement |
| Blocking engineering prerequisite | ACCEPTANCE § 4, § 10, § 11 — none run |
| Blocking research prerequisite | F8 (memory does not affect routing) must be resolved |
| Literature gate | **Not passed.** 6 must-read papers identified, 0 read |
| Experiments run | **0** |
| Citations verified by reading | **0 of ~70** |
| Go/no-go | **NO-GO for implementation.** 5 of 5 conditions unmet |

---

## How to keep this folder honest

1. A claim without a tag from the table above does not belong here.
2. When a `VERIFIED-WEB` item is read in full, move it to `references.md` with a
   `READ-<date>` status and record what it actually says — not what the search
   snippet said.
3. When a gate in `09_timeline_and_gates.md` is passed or failed, record it in
   `templates/DECISION_LOG.md` on the day it happens.
4. `INHERITED` is not a licence. Anything inherited that becomes load-bearing in
   a manuscript must be re-verified first.
