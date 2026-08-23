# Research questions

Status: **provisional** — pending the week-5 literature gate in
`selected-topic.md`. Selected direction: **D1, disconfirmation-aware shared
obstacle memory** (see `novelty-analysis.md`).

Each question states what would count as a **positive answer**, a **negative
answer**, and — critically — **what would invalidate the question itself**.
A question with no falsifying outcome is not a research question.

---

## RQ1 — Does accumulate-only shared memory degrade with false or stale evidence?

> In a sparse geo-referenced obstacle belief store with monotone confidence
> accumulation and wall-clock decay, how does mission-level routing performance
> vary with the rate of false or stale obstacle reports?

- **Positive:** unnecessary-avoidance rate and excess path length rise
  monotonically and significantly with the false-report rate; the effect
  persists over ≥30 missions rather than washing out.
- **Negative:** performance is flat in the false-report rate — the wall-clock
  decay is already sufficient at realistic rates.
- **Invalidates the premise:** if realistic false-report rates in the deployed
  reactive avoider are ~0 (measurable from E1's logs), then the problem is
  hypothetical and the paper must be reframed around *stale* rather than
  *false* obstacles (E3), which remain unambiguously real.

**This is the load-bearing question.** If RQ1 is negative, there is no problem,
and the honest response is the characterisation-study fallback.

---

## RQ2 — Can free-space measurements restore performance, and at what safety cost?

> Can a disconfirmation operator that converts existing free-space measurements
> (`eff_front_m` along a known heading) into negative evidence recover the
> performance lost in RQ1, without increasing clearance violations?

- **Positive:** ≥50% of the gap to the oracle is recovered, with no
  statistically significant increase in minimum-clearance violations or
  near-misses.
- **Negative:** either the gap is not recovered, or it is recovered *only* at
  the cost of measurably worse clearance.
- **Note:** a negative answer of the second kind is still a result — it
  quantifies the stability–plasticity tradeoff, which is the paper's secondary
  contribution regardless.

---

## RQ3 — Does evidence-driven forgetting dominate wall-clock decay?

> Across environment change rates, does forgetting driven by contradicting
> observations outperform an exponential wall-clock decay — including a
> wall-clock decay whose time constant has been tuned per scenario?

- **Positive:** evidence-driven decay matches or beats *tuned* τ across the
  change-rate sweep, and strictly beats it where change is non-uniform in space.
- **Negative:** a well-tuned τ is sufficient.
- **Expected shape:** the interesting result is likely *conditional* — tuned τ
  competitive when obstacles change at a uniform rate, evidence-driven decay
  winning when change is spatially heterogeneous (some obstacles permanent,
  others transient), because a single τ cannot serve both. **State this
  hypothesis before running the experiment.**

**Beating tuned τ is the single most important control in the paper.** A
reviewer will assume the gain is just a better decay constant.

---

## RQ4 — Does source reliability contain a degraded reporter?

> In a heterogeneous fleet where one drone's range sensing is biased, does
> online per-source reliability estimation prevent that drone's reports from
> degrading fleet-wide routing — and how many healthy reporters are required?

- **Positive:** with reliability weighting, a fleet with one degraded drone
  performs within noise of an all-healthy fleet; without it, performance
  degrades measurably.
- **Negative:** the estimator cannot separate a degraded sensor from a drone
  that simply flies different routes and legitimately sees different obstacles.
- **Known hard case:** a drone flying a *disjoint* route has no overlap with
  fleet consensus, so agreement is undefined. The estimator must abstain rather
  than penalise. **This must be handled explicitly, not discovered in review.**

---

## RQ5 — Does cost-aware escalation improve delivery success? *(secondary)*

> Does replacing the fixed 12 s reactive-avoidance escalation timer with a
> futility estimate priced against the remaining energy budget improve delivery
> success rate, particularly under tight battery budgets?

- **Positive:** higher success rate under tight budgets, with no loss under
  loose budgets.
- **Negative:** the fixed timer is near-optimal for this vehicle and world.
- **Scope note:** this is *one* experiment (E8). It is cut first if the timeline
  slips. It requires a calibrated `BATTERY_ENERGY_PER_M_PCT` to be meaningful,
  which is itself a prerequisite task.

---

## Question hierarchy

```
RQ1 (is there a problem?)  ──►  RQ2 (does the fix work?)  ──►  RQ3 (is the fix
                                                                 better than
                                                                 tuning?)
                                        │
                                        └──►  RQ4 (does it survive a bad agent?)

RQ5 (secondary, separable — cut if the timeline slips)
```

If **RQ1** is negative → reframe as characterisation (fallback in
`selected-topic.md`).
If **RQ2** is negative → the stability–plasticity quantification becomes the
contribution.
If **RQ3** is negative → the paper is substantially weaker; consider merging
into a broader characterisation.
If **RQ4** is negative → drop it; the paper stands on RQ1–RQ3.
