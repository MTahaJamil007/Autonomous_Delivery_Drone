# 06 — Risk register

Every risk has a **trigger** (the observable event that says it has happened)
and a **response** (what to do, decided now, while it is cheap to think
clearly). A risk without a trigger is a worry; a risk without a response is a
surprise.

Severity: **B** = blocking (project stops), **H** = high (paper materially
weakened), **M** = medium (scope cut), **L** = low.

---

## 1. Research-validity risks

| ID | Risk | Sev | Trigger | Response |
| --- | --- | --- | --- | --- |
| R01 | **The phenomenon does not exist**: memory never affects routing (F8) | **B** | Already true — `10_codebase_evidence.md` § 1 | Fix F8 as *platform*. Redefine `V1` as memory-conditioned pre-planning. Never claim the fix as contribution |
| R02 | **The mechanism is prior art**: persistence filters already do evidence-driven forgetting in sparse maps | **B** | Already true — `05` § 2 | Move the claim to the coupling result + mission-level evaluation. Implement the persistence filter as `V3-pf` |
| R03 | **The persistence filter converges here** and the coupling does not bite | H | `P1-E07`: `V3-pf` reaches oracle-level belief across margin settings | Reframe: "the existing method works; here is the first mission-level evaluation, and the margin regime where it starts to fail." Weaker but submittable. `P1-E06` still carries a result |
| R04 | **Reliability claim is prior art** (inter-robot trust) | H | Already true — `05` § 3 | Demote to ablation component; cite trust literature as origin |
| R05 | **Paper 2's VoI baseline wins** | H | `P2-E01` K1 fails | Stop the RL paper. Publish the VoI formulation as a classical cost-aware verification paper. **Cheap, early, and planned for** |
| R06 | **The effect is small** | M | Cliff's δ < 0.2 with CI spanning 0 on the primary endpoint | Report honestly. The margin/FOV sweep (`P1-E06`) and the characterisation carry the paper |
| R07 | **Real false-report rate is ~0**, so the phantom scenario is hypothetical | M | Pilot logs show the deployed reporter almost never errs | Pivot the headline to the *removal* scenario, which is unambiguously real. Keep phantom as a stress test and say so |
| R08 | **Dense grid is affordable**, undermining "why sparse" | M | `P1-E08` shows bytes/mission within an order of magnitude | Scope the claim to bandwidth-constrained deployments, **report the numbers**, do not hide it |
| R09 | **Coupling is an artefact of `plan_detour`** | H | `P1-E11` second planner shows no coupling | The result becomes planner-specific; the paper's generality claim is cut. Report it — this is why `P1-E11` exists |
| R10 | A 2026 paper appears that closes the gap | M | Re-run searches at submission and at each revision | Cite it, re-position against it, or withdraw the novelty claim and strengthen the evaluation contribution |

## 2. Statistical and methodological risks

| ID | Risk | Sev | Trigger | Response |
| --- | --- | --- | --- | --- |
| R11 | **Missions counted as independent replicates** | **B** (to credibility) | Any analysis with n = missions | `04` § 3.1 — the run is the unit. Enforce in the analysis code, not by discipline |
| R12 | **Underpowered campaign discovered after it ran** | H | Post-hoc CIs too wide to support the claim | Power analysis from a 5-run pilot *before* committing. Cut cells, never runs |
| R13 | **Safety concluded from a non-significant difference** | H | Any sentence of the form "no significant increase in collisions" | Pre-specified non-inferiority margin + TOST (`04` § 3.4) |
| R14 | **Multiple comparisons** inflate false positives | M | >1 primary endpoint per experiment | One primary endpoint per experiment; Holm across primaries; everything else labelled exploratory |
| R15 | **Baselines under-tuned**, making the method look good | H | Baseline hyper-parameters not searched, or searched on test | Equal tuning budget on train/val; record and report the search |
| R16 | **Test split leakage** | H | Test data touched before the method is frozen | Open once; if reopened, regenerate and report the first result as development data |
| R17 | **T1 surrogate is not valid** | H | `P1-E13` shows ordering reversals between T1 and T2 | Restrict claims to cells confirmed in T2; report the disagreement as a finding |
| R18 | **Metric defined after seeing data** | H | Any metric that first appears in the results section | Metric set frozen in the pre-registration |

## 3. Engineering and schedule risks

| ID | Risk | Sev | Trigger | Response |
| --- | --- | --- | --- | --- |
| R19 | **ACCEPTANCE § 4 does not pass** | **B** | First three-leg flight fails repeatedly | `STATUS.md` predicts 2–3 defects; budget 3 weeks. If unresolved at 6 weeks, escalate: reduce mission to two legs and state the simplification in the paper |
| R20 | **Three-drone fleet (§ 11) fails** | H | Concurrency defects that resist fixing | **Both titles say "Multi-UAV."** Fall back to "repeated missions by a small fleet (K ≤ 2)" or a single UAV with a persistent shared store, and retitle honestly. Do not claim three drones from two |
| R21 | **Throughput far worse than assumed** | H | `P1-E00` shows ≫3 min/mission or poor parallel scaling | Shift more weight to T1; cut cells; consider sim-time acceleration only after verifying it does not corrupt lockstep timing |
| R22 | **Batch corruption / interrupted campaigns** | M | Resumed batch produces a biased subset | Checkpoint + resume mandatory; randomise cell execution order; verify completion counts per cell before analysis |
| R23 | **Scope creep into landing, energy, escalation** | M | Work begins on anything in `01` § 5.4 | Those are cut. Re-adding one requires a decision-log entry with what is being dropped to pay for it |
| R24 | **Research code contaminates flight code** | M | Flight modules importing `research/` | Repo gate forbidding the import direction, like the existing gates |
| R25 | **PX4 alpha build drifts** | M | Results not reproducible across a rebuild | Pin a stable tag or archive the exact commit SHA + build flags; record in every mission record |

## 4. Publication, ethics and legal risks

| ID | Risk | Sev | Trigger | Response |
| --- | --- | --- | --- | --- |
| R26 | **Citing unread papers** | **B** (to integrity) | Any citation without a `READ-<date>` tag | No citation ships unread. `05` § 1 |
| R27 | **Incorrect claims about prior work** | H | "X does not solve Y" written from an abstract | Negative claims require the full text |
| R28 | **Redundant publication / salami-slicing** across the two papers | H | Overlapping contributions or reused text | Paper 2 cites Paper 1; declare unpublished related work to the editor and supply the manuscript; rewrite shared sections |
| R29 | **GPL contamination** from the persistence-filter reference implementation | M | Linking GPL-3 code into the repository | Re-implement from the paper, or isolate in a separately-licensed comparison repo. `08` § 5 |
| R30 | **Sim asset licensing** — vendored PX4-derived models | M | Release without attribution or licence review | Audit `sim/models/` provenance and licences before release; preserve upstream notices |
| R31 | **Generative-AI disclosure** requirements not met | M | Submission without a disclosure statement | Major publishers require disclosure of AI assistance in preparation and forbid AI authorship. Check the live guidelines and include the statement |
| R32 | **Authorship dispute** | M | Contribution or order contested late | Settle authorship, order and corresponding author **now**, in writing, with the supervisor; record in the decision log |
| R33 | **Venue mismatch** (simulation-only rejected on scope) | M | Desk rejection | Verify simulation-only acceptability *before* submitting; keep a ranked list of three venues |
| R34 | **Dataset called a benchmark prematurely** | L | The word appears without splits + baselines + scripts + datasheet | Complete the package first, or call it a dataset |

## 5. The three that will most likely actually happen

Ranked by probability × cost. Plan concretely for these.

1. **R19 — the three-leg mission takes longer than budgeted.** It has never been
   flown, and the repository predicts defects. Everything is behind it.
   *Mitigation:* run it in week 1; do literature reading in parallel so a
   simulator week is never a wasted week.
2. **R03 — the persistence filter works better than expected.** Twelve years of
   use is a lot of evidence. *Mitigation:* the paper is already written to
   survive this (`01` § 7). Implementing `V3-pf` early converts the project's
   biggest unknown into a measurement.
3. **R05 — myopic VoI is good enough.** Small interpretable state spaces are
   where classical decision theory is strongest. *Mitigation:* K1 is deliberately
   cheap and early; failing it in week two is a success, not a loss.
