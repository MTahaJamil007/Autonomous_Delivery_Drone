# Executive summary

**Date:** 2026-09-13 · **Verdict:** direction confirmed, claims re-scoped,
**implementation not yet authorised** (0 of 5 go/no-go conditions met).

---

## The one-paragraph verdict

The research direction — persistent shared environmental belief in repeated
multi-UAV delivery — is the right choice, for the reason the 2026-08-23 folder
gives: it grows from the project's most unusual asset and it is the least
crowded intersection available. But the specific claim built on it does not
survive contact with either the code or the literature. The **mechanism** claimed
as novel (evidence-driven forgetting in a sparse belief store) is prior art with
public code, the **reliability** claim is prior art, and the **phenomenon** the
paper is about cannot currently occur in the system, because shared memory never
influences a route. Fixing those is not a retreat: the audit produced a sharper,
more defensible claim than the one it replaced, and one that makes the second
paper necessary rather than optional.

---

## Six findings that change the plan

### 1. The system cannot exhibit the phenomenon · `VERIFIED-CODE`

Shared obstacle memory is fetched once per mission
(`drone_agent/mission.py:386`) and read in exactly one place
(`drone_agent/navigation.py:383`) — inside the escalation branch, which fires
only after 12 seconds of continuous dodging. The route is initialised to the
straight line (`navigation.py:247`) and `plan_detour()` is never called at leg
start.

So a phantom obstacle in the database costs nothing, "time-to-forget" is
undefined, and the dose–response experiment is flat by construction. The
project's own acceptance criterion — ACCEPTANCE § 10, *"run 2 deviates around
the wall from departure"* — cannot be met by the current code.

**Fix:** add memory-conditioned pre-planning (~20 lines, feature-flagged), call
it platform, and redefine the primary baseline around it.

### 2. The disconfirmation signal never reaches the consumer · `VERIFIED-CODE`

`avoider_node.py` reduces the 360-sample scan to three scalars before it crosses
the UDP boundary. `methodology.md` proposed deriving negative evidence from
`eff_front_m` — the ±35° front cone. But a detour passes ≈9.5 m abeam
(`OBSTACLE_DEFAULT_RADIUS_M` 3.0 + `DETOUR_MARGIN_M` 6.5), which is never in the
front cone.

**Fix:** a sectorised free-space channel. This is the observation model, not an
implementation detail.

### 3. The mechanism is prior art · `VERIFIED-WEB`

The persistence filter (Rosen, Mason & Leonard, ICRA 2016) is a Bayesian model
of **sparse feature persistence updated from negative evidence**, with public
code. FreMEn (T-RO 2017), clique change detection (2020) and Perpetua (2025)
extend the family. The 2026-08-23 literature review misses this subfield
entirely.

**Fix:** implement the persistence filter as a baseline (`V3-pf`) and move the
claim off the mechanism.

### 4. The reliability claim is false as written · `VERIFIED-WEB`

> ~~"No published merging rule for shared robot maps estimates and applies
> per-source reliability."~~

Inter-robot trust (Pierson & Schwager 2013), resilient fusion with malicious
robots, uncertainty-weighted multi-robot mapping and explicit reputation
frameworks all exist.

**Fix:** demote source reliability to an ablation component, cite the trust
literature as its origin, and keep only the specific abstention case (a drone on
a disjoint route has no overlap, so the estimator must abstain rather than
penalise).

### 5. The statistics as designed are invalid · `JUDGEMENT`

A 30-mission sequence with persistent memory is **one observation**, not thirty.
Missions inside a run are serially dependent — that dependence is the
phenomenon. The plan correctly warns about this and then designs experiments
that do it.

**Fix:** the *run* is the unit of analysis; ≥30 independent runs per cell, sized
by a power analysis from a pilot. And safety must be a **non-inferiority test**,
not a failure to reject the null.

### 6. And the finding that makes the project better · `JUDGEMENT`

In mapping, belief and observation are independent — a robot revisits features
and missed detections accumulate. In **delivery** they are coupled adversarially:

> **A belief that is successfully avoided stops generating the evidence that
> would refute it. The better the planner, the more complete the starvation.**

This is measurable in this system, general in form (it depends on clearance
margin, sensor FOV, range and belief radius — not on the simulator), and it
bounds *every* passive method including the persistence filter. It is also the
honest reason Paper 2 has to exist: escaping the loop requires deliberately
flying somewhere the planner would avoid.

This is the strongest idea in the project. It was not in the original plan.

---

## What the two papers become

| | Before | After |
| --- | --- | --- |
| **P1 claim** | A new disconfirmation operator, evidence-driven forgetting, source reliability, confidence-aware planning, mission metrics, cost-aware escalation (6 contributions) | Beliefs suppress their own refutation in delivery; here is the observability condition, the bound, and what it costs at mission level (4 contributions) |
| **P1 novelty basis** | Mechanism | **Problem formulation + analysis + evaluation** |
| **P1 key baseline** | Tuned τ | **A persistence filter that must be beaten or explained** |
| **P2 claim** | A learned memory-management policy (KEEP/FORGET/VERIFY) | Verification as **cost-bearing fleet allocation** with deferred cross-agent benefit |
| **P2 novelty basis** | RL applied to memory | **Decision-problem formulation**, defended against a myopic VoI baseline |
| **P2 kill criterion** | none | **K1: beat myopic VoI on the abstract env, or do not write an RL paper** |

---

## The answer to the question at the top of the audit

> *"If a reviewer says 'this is just an existing technique applied to a drone',
> what exact evidence proves them wrong?"*

Not an argument — a measurement:

> The persistence filter, the strongest published method for exactly this
> problem, is implemented in our setting and **does not converge** — not because
> it is mistuned, but because the route planned to avoid a belief removes it
> from sensor coverage, so the missed-detection evidence the filter needs is
> never generated. We derive the observability condition, measure the residual
> error across clearance margin, FOV and range, and show the bound is a property
> of avoidance rather than of the update rule.

An existing technique that is *in the paper, running, and losing for a stated
reason* is not an objection. It is the result.

---

## Immediate next actions, in order

1. **Fly ACCEPTANCE § 4.** Everything is behind it. Expect defects.
2. **Read three papers** — persistence filter, FreMEn, learned VoI (arXiv
   2403.03269). In parallel. Two weeks. They can end the project cheaply, which
   is why they come first.
3. **Measure throughput and SITL non-determinism** (`P1-E00`). Every budget
   number is currently an assumption.
4. **Fix F8 and F9** behind feature flags, with tests.
5. **Rewrite the gap paragraph** against what was actually read, then re-run the
   go/no-go.

**Do not** start the operator, the scenario campaign, or any RL work until Gate
A (`09_timeline_and_gates.md`) passes.

---

## Honest assessment of the whole thing

**What is good.** The governance instincts are better than most postgraduate
projects: freeze a baseline, pre-register, separate engineering from research,
gate before RL, safety outside the learned policy, hostile review before
submission, reproducibility as an artefact. The 2026-08-23 folder's rule —
*nothing gets a ✅ without evidence* — is exactly right and this audit is an
application of it to the folder itself.

**What is risky.** The engineering foundation is unflown, the fleet is size 1,
and the literature has more prior art than the review found. Two of the six
findings above would have been discovered by a reviewer rather than by you.

**What it is worth.** Even in the failure branches — persistence filter
converges, VoI wins, effect is small — this produces a submittable paper and a
reusable platform, because the gates are designed so that every failure has a
publishable form. That is unusual, and it is what makes the project safe to
commit to full time.

**Expected outcome:** Paper 1 submitted around month 7–8 and Paper 2 around
month 13–15, with a real chance that Paper 2 becomes a classical
value-of-information paper rather than a learning one. That would not be a
failure. It would be the plan working.
