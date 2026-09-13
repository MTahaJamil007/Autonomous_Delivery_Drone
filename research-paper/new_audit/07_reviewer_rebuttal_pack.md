# 07 — Reviewer rebuttal pack

Written before the experiments so the experiments can be designed to answer
these. An objection that can only be answered with words is an objection that
was not planned for.

Format: **objection → is it fair? → the answer → the evidence that makes the
answer stick.** If the evidence column is empty, the objection is currently
unanswerable and the experiment list is incomplete.

---

# Paper 1

## The three that can kill it

### 1. "This is a persistence filter."

**Fair? Yes — the strongest objection in the pack.** Rosen, Mason & Leonard
(ICRA 2016) model sparse feature persistence from negative evidence and provide
code.

**Answer:** We agree, and we implement it. Our claim is not a new update rule; it
is that in a *delivery* fleet the belief selects its own observation process, so
the missed-detection evidence any persistence model depends on is systematically
withheld for exactly the beliefs being avoided. We show the observability
condition, and we measure the persistence filter's residual error as a function
of the planner's clearance margin.

**Evidence:** `P1-E07` (persistence filter as `V3-pf`), `P1-E06` (margin/FOV/range
sweep with the predicted and measured non-convergence), § 3.3 of the paper (the
derivation).

**If the filter converges anyway:** say so, in the abstract. The paper becomes a
mission-level evaluation showing where the existing method holds and where it
begins to fail. Weaker, honest, still publishable. This is planned for.

### 2. "This is log-odds occupancy mapping with extra steps."

**Fair? Partly.** The update does reduce to a log-odds update in the
single-source, non-overlapping, fully-observable case.

**Answer:** Conceded explicitly in the method section — it costs nothing and buys
credibility. The differences that matter are that the observation is a sector
minimum rather than a beam, that evidence must be attributed across *merged,
overlapping* records with no cell geometry, and that the observation process is
endogenous to the belief. The last is the one that produces the result.

**Evidence:** `P1-E08` (dense grid `V7` as a baseline, with bytes/mission, rows
and query latency reported), `P1-E10` (attribution-rule ablation).

### 3. "Just use an occupancy grid, then."

**Fair? Yes, until measured.**

**Answer:** Then measure it. If the grid is affordable at this scale, we say so
and scope the claim to bandwidth-constrained deployments. The coupling result is
*representation-independent* — a dense grid routed around by the same planner
receives no ray through the obstacle either, so the free-space evidence is
equally absent. That is the strongest form of the answer, because it makes the
finding survive the objection entirely.

**Evidence:** `P1-E08`, and `P1-E06` run with `V7-grid` included.

## The rest

| Objection | Fair? | Answer | Evidence |
| --- | --- | --- | --- |
| "Trust-weighted fusion already exists." | **Yes** | Agreed; reliability is an ablation component, not a claim. The contribution is the abstention rule for non-overlapping routes and the mission-level containment measurement | `P1-E09`, `05` § 3 |
| "Your gain is just a better decay constant." | Yes | `V2-tau*` is grid-searched per scenario on the training split, with the search reported. The predicted and tested result is that tuned τ is *competitive* under uniform lifetimes and loses under heterogeneous ones | `P1-E05`, incl. the heterogeneous cell |
| "Are your missions statistically independent?" | **Yes** | No — and we say so. The run is the unit of analysis; missions within a run are a time series | `04` § 3.1; stated in the protocol section |
| "Is 30 enough?" | Yes | ≥30 independent runs per cell, sized by a power analysis from a 5-run pilot, with effect sizes and bootstrap CIs | `04` § 3.2, power analysis reported |
| "Does faster forgetting increase collisions?" | **Yes — the most important safety question** | Pre-specified non-inferiority test with a stated margin; every condition reports clearance distribution, near-misses and re-collision-after-forgetting | `04` § 3.4; safety table in the main body |
| "Is this an artefact of your planner?" | Yes | Second, structurally different planner | `P1-E11` |
| "Your simulator's LiDAR is unrealistically clean." | Yes | Noise, dropout and bias calibrated to a real 2-D LiDAR datasheet and swept | `P1-E12` |
| "Your abstract simulator is doing the work." | **Yes** | T1 reuses the real planner and the real belief store; T1↔T2 agreement is reported as a result, including any disagreement | `P1-E13` |
| "One world, one airframe." | Yes | Stated as an external-validity limitation; second world family; parameter sweeps rather than single settings | `P1-E11`, limitations section |
| "Simulation only." | Yes | Acknowledged. Mitigated by sensor realism, many independent seeds, strong baselines, released code and data. Venue chosen accordingly | Limitations section |
| "Your 'unnecessary-avoidance rate' is not new." | Yes | We do not claim it is. We define it precisely for this setting and release the computation | Metric definitions |
| "Energy results are not credible." | **Yes** | `BATTERY_ENERGY_PER_M_PCT` is a placeholder and payload mass is unsimulated. Energy is reported as a distance-based proxy, labelled as such at every occurrence | Metric definitions |
| "Why not learn the whole thing?" | Fair | Out of scope for Paper 1, and addressed in Paper 2 with a VoI baseline that must be beaten before learning is claimed to help | — |

---

# Paper 2

## The three that can kill it

### 1. "Why RL? A threshold would do."

**Fair? Yes.**

**Answer:** The threshold baseline is in the paper, tuned on the training split
with the same budget. So is a **myopic value-of-information policy**, which is
the principled version of the threshold and a much harder target. If VoI wins, we
say so — and we have committed in advance to not publishing a learning paper in
that case.

**Evidence:** `P2-E01` (K1), `P2-E02` (near-optimal reference bounding the
available headroom).

### 2. "Learned value of information already exists."

**Fair? Yes** — arXiv:2403.03269 and related work.

**Answer:** Agreed for the single-agent case. Our setting differs in structure:
the information-gathering action is a physical detour inside a deadline-bearing
delivery mission, and its benefit accrues to *other agents on future missions*.
That deferred, cross-agent credit assignment is the thing we claim, and we test
it directly by measuring whether the margin over VoI **grows** from the
single-agent to the multi-agent setting.

**Evidence:** `P2-E01` vs `P2-E03` (K2). If the margin does not grow, the framing
is not doing work and we say so.

### 3. "Is the gain just more parameters?"

**Fair? Yes.**

**Answer:** Ablations remove state features one at a time, so the gain is
attributed to *information*, not capacity. We additionally report a
capacity-matched control: the same architecture trained without the
consequence-related features.

**Evidence:** `P2-E07`.

## The rest

| Objection | Fair? | Answer | Evidence |
| --- | --- | --- | --- |
| "Why PPO / why MAPPO?" | Yes | PPO as a standard stable default; no novelty claimed. A centralised critic is introduced only if the deferred cross-agent credit structure demands it, and that is stated as the reason | Method section |
| "Does it generalise?" | **Yes** | Held-out layout families, change rates and **fault modes**, reported separately from in-distribution | `P2-E06` (K3) |
| "Can the policy be trusted?" | **Yes** | It never produces control. It emits belief-management recommendations; hard constraints (clearance floor, geofence, battery reserve) sit outside it and can override. Overrides are counted and reported | `02` § 5.1, `P2-E08` |
| "What about out-of-distribution inputs?" | Yes | Inputs clamped, out-of-envelope recommendations discarded and counted, count reported per condition | `P2-E08` |
| "Does verification pay for itself?" | **Yes — central** | Verification spend and its return are a reported endpoint, including a static-world control where the correct behaviour is to verify almost nothing | `P2-E10` |
| "What happens with sensor faults?" | Yes | Degraded-reporter study, with fault modes held out of training | `P2-E04`, `P2-E06` |
| "Only simulation." | Yes | Same answer as Paper 1, plus the T1↔T2 confirmation | `P2-E09` (K5) |
| "This is Paper 1 again." | **Yes, if handled badly** | Distinct contributions, Paper 1 cited not re-reported, shared text rewritten, related unpublished work declared to the editor | `02` § 9 |
| "Single training seed." | Yes | ≥5 training seeds, reported individually and aggregated, with learning-curve variance shown | Method section |

---

# The 60-second test

> *"What exactly is new here?"*

**Paper 1:** "Everyone studies map memory on robots that are trying to observe.
Delivery drones are trying to *avoid* — so the belief removes the evidence that
would correct it. We show when that makes forgetting impossible, including for
the best existing method, and we measure what it costs a delivery fleet."

**Paper 2:** "If avoiding a belief prevents correcting it, someone has to go and
look on purpose. Deciding who goes, for which belief, on which mission, is a
fleet allocation problem with deferred cross-agent benefit — and we show it is
worth more than the obvious one-step calculation."

Both are one breath. Both are falsifiable. Both survive "that already exists,"
because in both the existing thing is *in the paper, implemented, and losing for
a stated reason*.

---

# The pre-submission hostile review

Do this three weeks before submission, with someone who did **not** build the
experiments.

1. Give them the manuscript, the code, and this file with the answers removed.
2. Ask them to find the three weakest claims.
3. Ask them, after reading only the abstract: *"what is new?"* If they cannot
   answer in 60 seconds, the abstract is wrong.
4. Ask them to reproduce one figure from the released code, unaided.
5. Ask them to find one sentence the evidence does not support. There is usually
   one, and it is usually in the introduction.

Record the outcome in the decision log. Fix what they find before submitting,
not during review.
