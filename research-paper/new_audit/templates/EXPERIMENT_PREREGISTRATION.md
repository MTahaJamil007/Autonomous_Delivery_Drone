# Pre-registration — `<EXPERIMENT_ID>`

Copy to `research-paper/new_audit/preregistrations/<ID>.md`, fill in **every**
field, and **commit before the first run**. The commit timestamp is the
evidence.

Nothing above the line marked *filled in after running* may be edited once data
exists. If it must change, strike it through, leave it visible, and add a dated
reason.

---

## Identity

| Field | Value |
| --- | --- |
| Experiment ID | `P1-Exx` / `P2-Exx` |
| Title | |
| Paper | 1 / 2 |
| Tier | T1 (abstract) / T2 (SITL) / both |
| Registered by | |
| Date registered | |
| Git commit at registration | |

## Question

One sentence. What is being asked, not what is being run.

## Hypothesis

> **H:** …

State the **direction**, not just the existence of an effect. "A differs from B"
is not a hypothesis; "A produces lower cumulative excess distance than B" is.

## What would falsify it

> **Falsified if:** …

If this is blank, the experiment is not a test. Stop and rewrite the hypothesis.

## Design

| Field | Value |
| --- | --- |
| Factors and levels | |
| Policy variants compared | |
| Scenario family / families | |
| Missions per run (K) | |
| **Runs per cell (n)** | |
| Total cells | |
| Total runs | |
| Belief-store initial state | |
| Fleet size | |
| Split used (train / val / test-ID / test-OOD) | |

## Sample size justification

| Field | Value |
| --- | --- |
| Pilot runs performed | |
| Observed variance of the primary endpoint | |
| Smallest effect worth detecting | |
| Power target | |
| Resulting n | |

If n was set by budget rather than by power, **say so here** and state the
effect size the design can actually detect.

## Analysis plan

| Field | Value |
| --- | --- |
| **Primary endpoint (exactly one)** | |
| Unit of analysis | run (default) / mixed-effects with run as random effect |
| Test | Mann–Whitney U / Brunner–Munzel / other |
| Effect size | Cliff's δ with bootstrap 95% CI, ≥10,000 resamples |
| Multiple-comparison handling | |
| Secondary endpoints (labelled as such) | |
| Exploratory quantities (no inference claimed) | |

## Safety guardrails

| Field | Value |
| --- | --- |
| Metrics reported for every condition | collisions · min clearance · 5th pct clearance · near-misses · re-collision-after-forgetting · supervisor interventions |
| Near-miss threshold (fixed **now**) | |
| Non-inferiority margin | |
| Non-inferiority test | TOST / one-sided |

## Exclusion rules

Exclusions permitted only for: infrastructure failure · incomplete provenance ·
geofence or battery-reserve violation. Never for an unexpected result.

| Field | Value |
| --- | --- |
| Expected exclusion rate | |
| Action if exclusions exceed 5% in any cell | Stop; fix the infrastructure; re-run the cell |

## Provenance

| Field | Value |
| --- | --- |
| Config file(s) | |
| Config hash | |
| Container digest | |
| Seed generation scheme | |
| Where raw records will be written | |

## Pre-declared expectations

Write what you expect, including where you expect the method to **lose**. An
experiment with no predicted losing cell has probably not been designed to test
anything.

| Cell | Expected direction | Confidence (low / med / high) |
| --- | --- | --- |
| | | |

---

---

# Filled in after running

*(Nothing above this line may be edited.)*

| Field | Value |
| --- | --- |
| Date run | |
| Git commit at run | |
| Runs completed / planned | |
| Exclusions and reasons | |
| Wall-clock consumed | |

## Result

Primary endpoint, with effect size and CI. State the outcome relative to the
pre-registered direction, in one sentence, before any interpretation.

## Deviations from this pre-registration

Every one, with a reason. "None" is an acceptable and preferable answer.

## Surprises

Things that were not predicted. Record them as surprises — they are the most
valuable content in this file, and they are the seed of the next experiment.

## Consequence

What this changes: the paper's claim, the next experiment, or nothing.
