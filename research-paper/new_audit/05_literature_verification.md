# 05 — Literature verification and citation integrity

Searched **2026-09-13**. This document does two jobs:

1. Records **prior art the 2026-08-23 review missed**, and what it does to the
   novelty claims.
2. Fixes a process problem: **zero of the ~70 entries in `references.md` have
   been read.** A manuscript built on unread search snippets is a retraction
   risk, not merely a weak paper.

---

## 1. The process problem, first

`references.md` states its own convention honestly:

> `[URL-2026-08-23]` = the URL was returned by a literature search ... It has
> **not** been read in full.

Every single entry carries that tag or `[STD]`. Nothing has been read.

A spot check performed today confirms the entries are not fabricated — for
example `Evidence-Landing-2026` resolves to a real paper, *"Evidence-Based
Landing Site Selection and Vision-Based Landing for UAVs in Unstructured
Environments"*, arXiv:2605.01432, submitted 2 May 2026 (`VERIFIED-WEB`). So the
bibliography is *real*. The danger is different and subtler:

> **The 2026-08-23 folder states what each paper does not solve, based on
> abstracts and search snippets.** Every "what it does NOT solve" column in
> `novelty-analysis.md` is an `UNVERIFIED` claim about a paper nobody read.

Those columns are precisely the load-bearing sentences of the novelty argument.
Section 3 below shows two of them are already known to be wrong.

### The rule from here

1. **No citation appears in a manuscript until the PDF is in
   `research-paper/papers/` and someone has read it.** Mark it `READ-<date>` in
   `references.md` with a one-paragraph note in `notes/`.
2. **No "X does not solve Y" sentence is written from an abstract.** Negative
   claims about prior work require reading the method and experiments sections.
3. **Every citation carries a DOI or arXiv ID, checked to resolve.** Run the
   check as a script; do not eyeball it.
4. **Re-run the searches at submission and at every revision.** This field moves
   fast; a paper published between drafting and review is the most common cause
   of a "this is already known" reviewer report.
5. **Record the search protocol in the manuscript** — databases, query strings,
   date, inclusion criteria, number screened. A reviewer who can reproduce the
   search cannot accuse you of missing the field selectively.

---

## 2. Missed subfield #1 — persistence modelling in sparse maps

**This is the most serious omission.** The 2026-08-23 review organises mapping
literature into "dense occupancy grids" and "multi-UAV learning", and concludes
that the *sparse* case has no disconfirmation mechanism. There is an entire
subfield between those two categories, and it does exactly that.

| Work | What it is | `VERIFIED-WEB` source |
| --- | --- | --- |
| **Rosen, Mason & Leonard, "Towards Lifelong Feature-Based Mapping in Semi-Static Environments", ICRA 2016 — the *persistence filter*** | A probabilistic generative model of **feature survival over time**, with a recursive Bayesian estimator giving an explicit belief over the persistence of **each sparse feature**, updated from *missed detections* (negative evidence). C++/Python implementation public. | [IEEE Xplore](https://ieeexplore.ieee.org/document/7487237/) · [code](https://github.com/david-m-rosen/Persistence-Filter) |
| **Krajník, Fentanes, Santos & Duckett, "FreMEn: Frequency Map Enhancement for Long-Term Mobile Robot Autonomy in Changing Environments", IEEE T-RO 33(4), 2017** | Models each environment state's occupancy probability as a **spectral (frequency-domain) function of time**, enabling *prediction* of future states; compression up to 1:100000. | [IEEE Xplore](https://ieeexplore.ieee.org/document/7878680/) |
| **"Better Together: Online Probabilistic Clique Change Detection in 3D Landmark-Based Maps", 2020** | Bayesian filtering over **landmark** persistence where landmarks are only partially observable due to sensor degradation, geometry and occlusion. | [arXiv:2008.00372](https://arxiv.org/pdf/2008.00372) |
| **"Perpetua: Multi-Hypothesis Persistence Modeling for Semi-Static Environments", 2025** | Multi-hypothesis persistence with different appearance/disappearance timescales. | [arXiv:2507.18808](https://arxiv.org/pdf/2507.18808) |
| Long-term LiDAR map maintenance / change detection (several 2025–2026 works) | Detecting spatial discrepancies across sessions and incrementally updating a prior map. | [arXiv:2606.29469](https://arxiv.org/pdf/2606.29469) and related |

### What this kills

> ~~"For a sparse, geo-referenced obstacle belief store there is no published
> model of disconfirmation, no evidence-driven forgetting rule..."~~

Delete it. A persistence filter **is** a published model of disconfirmation for
a sparse belief store, driven by negative evidence rather than the calendar. It
is twelve years old, it has open-source code, and the reviewer who knows it will
reject the paper in one paragraph.

### What survives, and is stronger

Persistence-modelling work is built for **mapping robots**, whose job is to
observe. Three things do not transfer, and all three are properties of a
**delivery fleet whose job is to avoid**:

1. **The observation process is chosen by the belief.** A mapping robot revisits
   features to update them. A delivery drone *routes around* what it believes,
   and therefore stops observing it. Persistence filters assume missed
   detections arrive when the feature is in view; in this setting the belief
   itself removes the feature from view. See `01_paper1_specification.md` § 3.
2. **The cost function is a delivery mission, not map accuracy.** Persistence
   filters are evaluated on localisation and map quality. Nobody evaluates
   persistence decisions by route length, escalation count, energy, and delivery
   success over repeated missions.
3. **The belief is shared across a heterogeneous fleet through a service**, with
   merged overlapping records and no per-observation ray. Attribution across
   merged records is a genuine modelling question the single-robot filters do
   not face.

**Therefore:** the persistence filter is not a competitor to dodge. It is the
**baseline you must implement and beat or match** (`V3-pf` in
`04_experiment_protocol.md`). A paper that implements it, reports it fairly, and
shows where the delivery setting breaks it is far stronger than one that never
mentions it.

> **Licensing warning.** The reference implementation is GPL-3. Linking it into
> this repository would make the repository GPL-3. **Re-implement from the
> paper** in the evaluation package, or keep it in a separate,
> clearly-licensed comparison repository. See `08_reproducibility_standard.md`
> § 5.

---

## 3. Missed subfield #2 — trust and reliability in multi-robot fusion

`novelty-analysis.md` § D1 states:

> ~~"No published merging rule for shared robot maps estimates and applies
> per-source reliability."~~

This is false.

| Work | What it is | `VERIFIED-WEB` source |
| --- | --- | --- |
| Pierson & Schwager, "Adaptive Inter-Robot Trust for Robust Multi-Robot Sensor Coverage", ISRR 2013 | Trust weightings that converge to values reflecting each robot's sensing performance, with no external authority. | [PDF](https://sites.bu.edu/pierson/files/2021/05/pierson2013isrr.pdf) |
| "Trust But Verify: A Distributed Algorithm for Multi-Robot ..." | Distributed verification of peer contributions. | [NSF PAR](https://par.nsf.gov/servlets/purl/10205901) |
| "Exploiting Trust for Resilient Hypothesis Testing with Malicious Robots" | Resilient fusion in the presence of adversarial agents. | [arXiv:2303.04075](https://arxiv.org/pdf/2303.04075) |
| "UDON: Uncertainty-weighted Distributed Optimization for Multi-Robot Neural Implicit Mapping" | Uncertainty-weighted fusion that selectively combines the most reliable portions of each agent's map. | [arXiv:2509.12702](https://arxiv.org/pdf/2509.12702) |
| "Trust Management Framework for Multi-Robot Systems" | Explicit reputation framework. | [Annals CSIS](https://annals-csis.org/Volume_39/drp/pdf/3165.pdf) |

Beyond robotics there are decades of trust/reputation work in distributed
sensing (WSN, VANET) that a reviewer may also invoke.

### What this means

Source-reliability estimation **cannot be a headline contribution**. Demote it:

- In **Paper 1**: an *ablation component* and a robustness result, described as
  "a standard Beta-reputation weighting, adapted to abstention on
  non-overlapping observations". Cite the trust literature as the origin.
- In **Paper 2**: useful as the state feature that lets a learned policy decide
  *whom to send to verify* — which is an allocation question the trust
  literature does not ask.

Keep the genuinely awkward case, because it is specific and real and the
existing docs already spotted it: a drone flying a **disjoint route** has no
overlap with fleet consensus, so agreement is undefined and the estimator must
abstain rather than penalise. That is a concrete engineering result, not a
novelty claim.

---

## 4. Missed subfield #3 — learned value of information (hits Paper 2)

| Work | Why it matters | `VERIFIED-WEB` source |
| --- | --- | --- |
| **"Active Information Gathering for Long-Horizon Navigation Under Uncertainty by Learning the Value of Information", 2024** | Learns the **value of information** of exploratory actions for navigation under an incomplete map: exactly the "should I go and look?" decision Paper 2 proposes to learn. | [arXiv:2403.03269](https://arxiv.org/pdf/2403.03269) |
| "Where to Look Next: Learning Viewpoint Recommendations for Informative Trajectory Planning", ICRA 2022 | Learned viewpoint selection for information gain. | [arXiv:2203.02381](https://arxiv.org/pdf/2203.02381) |
| "Active Robotic Mapping through Deep Reinforcement Learning", 2017 | RL agent acting on a belief state to improve a map. | [arXiv:1712.10069](https://arxiv.org/pdf/1712.10069) |
| "Reinforcement Learning for Active Perception in Autonomous Navigation", 2026 | Recent RL active perception. | [arXiv:2602.01266](https://arxiv.org/html/2602.01266) |

### What this means for Paper 2

"An RL agent decides when to gather information" is **not** novel. Paper 2's
framing has to move to what these works do not do:

- The information-gathering action has a **physical route cost inside a
  deadline-bearing delivery mission**, not an exploration budget.
- The decision is over a **shared, persistent, multi-source belief** — the
  benefit of verifying accrues to *other* agents on *future* missions.
- It is therefore a **fleet allocation problem**: not only *whether* to verify,
  but *which drone, on which mission, at what detour cost, for whose benefit*.
- Safety is a **hard constraint outside the policy**, not a reward term.

See `02_paper2_specification.md` § 1.

---

## 5. The claims that survive, stated precisely

Use these sentences; they are defensible as of 2026-09-13. Every one of them is
a claim about the **setting and the evaluation**, not about the mechanism.

> **S1.** Persistence and forgetting in sparse maps have been studied for
> mapping robots, whose observation process is independent of their belief. In a
> delivery fleet the two are coupled: a route planned to *avoid* a believed
> obstacle removes that obstacle from sensor coverage, so the belief suppresses
> its own refutation. We characterise this feedback and show the conditions
> under which passive disconfirmation cannot converge.

> **S2.** Shared environmental belief in multi-UAV systems is evaluated at the
> map level (accuracy, IoU, localisation error). We evaluate it at the
> **mission level** over repeated deliveries — route excess, escalation rate,
> energy proxy, delivery success and clearance — and release the protocol.

> **S3.** The decision to spend flight distance verifying a stale belief, in a
> fleet where the benefit accrues to other agents and later missions, is an
> allocation problem that neither the persistence-modelling nor the
> active-perception literature formulates.

**Every one of these is falsifiable by a search.** Run that search before
committing, and again before submitting.

---

## 6. The six papers that must be read in full before any implementation

Reading order matters; the first two decide whether the project exists.

| # | Paper | Decides |
| --- | --- | --- |
| 1 | Rosen, Mason & Leonard 2016 — persistence filter | Whether S1 survives, and what the baseline is |
| 2 | Krajník et al. 2017 — FreMEn | Whether periodic/spectral modelling makes evidence-driven decay unnecessary |
| 3 | "Active Information Gathering ... Value of Information" 2024 | Whether Paper 2 has a problem left |
| 4 | Pierson & Schwager 2013 — inter-robot trust | How to frame the reliability component honestly |
| 5 | `When2Replan-2023` | The escalation contribution (RQ5) — likely to kill it |
| 6 | "Perpetua" 2025 + one 2026 map-maintenance paper | Whether the field has closed the gap in the last twelve months |

**Gate:** if papers 1–3 already contain the delivery-coupled formulation of S1,
the direction must change. That is a two-week check, not a two-month one, and it
must happen before any code is written.

---

## 7. Searches still owed

The 2026-09-13 pass was bounded. Before the gate in § 6 closes, run and record:

- `"semi-static" OR "long-term" mapping + "multi-robot" + persistence`
- `"map maintenance" + UAV + "change detection" + fleet`
- `"stale map" OR "outdated map" + planning + "information gathering"`
- `"negative information" OR "negative evidence" + landmark + filter`
- `"reputation" OR "trust" + "collaborative mapping" + UAV`
- `"active verification" + belief + multi-robot`
- `UAV delivery + "shared knowledge" OR "collective memory" + repeated missions`
- The same set restricted to 2026 only, repeated at submission time.

Record each query, the date, the number of hits screened, and what was kept.
That table goes in the manuscript's related-work section or its appendix.
