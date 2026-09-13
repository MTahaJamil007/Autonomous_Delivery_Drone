# Engineering failures log

Copy to `research-paper/engineering-failures.md` and keep it there.

**Purpose:** so that an engineering defect is never mistaken for an algorithmic
result, and so that a defect fixed in month two is not re-diagnosed in month
eight.

This is the single highest-value document during Phase A. Every hour spent
fighting the simulator is an hour of data that only exists if it is written
down.

---

## The rule

> **Classify before fixing.** A symptom fixed without a named layer is a symptom
> that returns.

Layers: `PX4` · `Gazebo` · `ROS2` · `DDS` · `MAVSDK` · `sensor` · `navigation` ·
`planner` · `mission-FSM` · `memory-service` · `timing` · `network` · `logging` ·
`research-code` · `host`

---

## Entry template

### F-NN — one-line symptom

| Field | Value |
| --- | --- |
| **Date** | |
| **Layer** | |
| **Severity** | blocks-research / degrades-results / cosmetic |
| **Reproducible** | always / intermittent (rate) / once |
| **First seen at** | experiment ID, commit, seed |
| **Symptom** | What was observed, not what you think caused it |
| **Diagnosis** | What was actually wrong, with evidence |
| **Fix** | Commit, files, and whether it is a fix or a workaround |
| **Affects prior results?** | **Which runs are invalidated.** Name them. Re-run or discard |
| **Detection** | The test or gate that would have caught it. If none, add one |
| **Paper relevance** | Motivating example / limitation / none |

---

## Why "affects prior results" is the important row

A defect discovered in week 20 that has been silently corrupting runs since week
12 invalidates eight weeks of data. If nobody wrote down when it started, the
only safe response is to discard everything — which is how campaigns get run
twice.

Every entry must name the runs it touches, or state positively that none are
affected and why.

---

## Worked example (format reference — from the repository's own history)

### F05 — offboard setpoint stream gapped by a blocking call

| Field | Value |
| --- | --- |
| **Date** | pre-audit (recorded in `CHANGELOG.md`) |
| **Layer** | timing |
| **Severity** | blocks-research |
| **Reproducible** | always, when the feature was enabled |
| **Symptom** | Offboard setpoint gaps large enough to risk PX4 dropping offboard mode |
| **Diagnosis** | A synchronous `subprocess.run` on a 10 Hz path starved the 20 Hz setpoint task |
| **Fix** | Async `gz` client; repo gate forbidding blocking calls on the flight path |
| **Affects prior results?** | None — predates any experiment |
| **Detection** | Repo gate: no blocking calls on the flight path |
| **Paper relevance** | **Motivating example.** "A system that appears to work and does not" — the same shape as a stale belief that appears correct |

---

## Live log

### F08 — shared obstacle memory does not influence routing

| Field | Value |
| --- | --- |
| **Date** | 2026-09-13 (found by audit, not by flight) |
| **Layer** | navigation |
| **Severity** | **blocks-research** |
| **Reproducible** | always — structural |
| **Symptom** | Prefetched obstacles never change a planned route; ACCEPTANCE § 10 run-2 criterion unachievable |
| **Diagnosis** | `navigation.py:247` initialises the route to the straight line; `known_obstacles` is read only inside the `ESCALATE` branch (`navigation.py:352,383`) |
| **Fix** | **Not yet applied.** Memory-conditioned pre-planning at leg start, feature-flagged |
| **Affects prior results?** | None (no experiments run). Invalidates the phantom/time-to-forget experiment design |
| **Detection** | Integration test: a seeded belief must change `planned_route` before takeoff |
| **Paper relevance** | Platform prerequisite. **Not a contribution.** Stated in the system section |

### F09 — free-space evidence discarded before the consumer

| Field | Value |
| --- | --- |
| **Date** | 2026-09-13 (audit) |
| **Layer** | sensor |
| **Severity** | blocks-research |
| **Reproducible** | always — structural |
| **Symptom** | No observation channel can disconfirm a belief the drone routes around |
| **Diagnosis** | `avoider_node.py:421-429` reduces 360 samples to 3 scalars; `eff_front_m` covers only ±35°, while a detour passes ≈9.5 m abeam |
| **Fix** | **Not yet applied.** Sectorised min-range vector alongside the control scalars |
| **Affects prior results?** | None |
| **Detection** | Contract test on the UDP payload schema |
| **Paper relevance** | **The observation model.** Belongs in the methodology section |

*(Append new entries below, newest last.)*
