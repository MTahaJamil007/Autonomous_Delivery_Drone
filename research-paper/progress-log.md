# Progress log

Reverse-chronological. Every substantive decision, finding, or change of
direction gets an entry. This is the audit trail for the research process.

---

## 2026-08-23 — Research workspace established; direction selected

**What happened.** Full audit of `DroneProgram` @ `91abd5e` (16,552 lines, 64
files). Literature search across 14 queries covering precision landing,
shared/collaborative mapping, reactive-deliberative arbitration,
perception-aware planning, runtime assurance and fault injection, energy and
payload planning, GNSS-denied navigation, learning-based navigation, and
empirical UAV software studies. Ten research directions identified, scored and
classified.

**Key findings from the audit** (full detail in `notes/codebase-audit.md`):

- The system is substantially more complete and more honestly documented than a
  typical FYP. 126 automated tests, 7 repository gates, and a `STATUS.md` whose
  stated rule is *"nothing gets ✅ without a call site and a named
  verification."*
- **Flight-verified so far:** model-scoped sensor topics, live ArUco marker-0
  acquisition at 5.51 m (88.6 px measured vs 80.1 px predicted), preflight 6/6,
  no LiDAR self-returns at 10 Hz, `udp://` connectivity, async pad spawning.
- **Not flight-verified:** ten procedures including the full three-leg mission
  (§ 4), the 20-landing statistical gate (§ 7), camera sign confirmation (§ 8),
  the detour+memory two-run experiment (§ 10), and the three-drone fleet (§ 11).
- **Three architectural gaps stand out as research opportunities:**
  1. The obstacle memory has **no disconfirmation channel** — `ObstacleReporter`
     writes only on `entering_dodge`, while the avoider's continuous
     `eff_front_m` free-space evidence is discarded. A false obstacle persists
     for ~months (τ = 14 d) no matter how many drones fly through it.
  2. A **calibrated perception forward model exists but is never used online** —
     `decodable_px_at_altitude()` (quiet-zone factor 0.796, measured) is
     consulted once at design time to pick a constant, while the landing
     controller gates descent on an unrelated geometric heuristic.
  3. **Reactive→deliberative handoff is a fixed 12 s timer** with no prediction
     and no accounting for the detour's energy cost.

**Key findings from the literature:**

- Fiducial precision landing is **saturated** — `Springer-JIRS-2025` reviews 143
  papers (2018–2025); centimetre accuracy is routine.
- Energy/payload-aware routing, GNSS-denied navigation, learning-based UAV
  navigation and multi-UAV deconfliction are all **saturated**.
- Dense probabilistic mapping solved disconfirmation decades ago — but **for
  grids**. The sparse, geo-referenced, service-backed, cross-mission,
  multi-source case has no equivalent free-space channel, no evidence-driven
  decay, no per-source reliability, and **no mission-level evaluation anywhere
  in the literature.** This is the least crowded intersection found.
- `When2Replan-2023` (OMRON SINIC X) is close prior work for the arbitration
  idea (D3) — ground robots, dense costmap, DRL. It makes D3 too risky as a
  standalone topic.
- `Evidence-Landing-2026` is close prior work for the landing idea (D2) —
  decision-theoretic, but for *terrain safety in unstructured environments*, not
  fiducial decodability.

**Decision.** Pursue **D1 — disconfirmation-aware shared obstacle memory** as
the primary direction, with **D3 (cost-aware escalation)** as a secondary
contribution and **D4 (degradation stress)** as the robustness section.
Rationale in `selected-topic.md`.

**Rejected, with reasons recorded:** D5 energy/payload (saturated; payload mass
is not even simulated), D6 dynamic obstacles on 2D LiDAR (weak sensor, weak
novelty), D7 GNSS-denied (saturated + no VIO stack), D8 DRL navigation
(saturated + no infrastructure), D9 deconfliction (saturated), D11 bug taxonomy
(n = 1; but F5 and the unreachable-RTL-fallback finding are excellent paper
motivation).

**Next actions, in order:**

1. **Run ACCEPTANCE § 4 (three-leg mission).** Everything is blocked on this.
   Expect 2–3 first-flight defects.
2. Run § 8 (camera signs) — no landing number is trustworthy until it passes.
3. Run § 10 (detour + memory, two runs) — this is the experiment E1 scales up.
4. Read in full and record notes on: `Evidence-Landing-2026`,
   `When2Replan-2023`, `Bosch-Evidential-2024`, `OGM-Merging`, `Thrun-2005`
   ch. 9, `ARCog-NET-2025`. **Week-5 gate: confirm no closer prior work exists.**
5. Formalise the disconfirmation operator on paper before writing code
   (`methodology.md` § 2 has the current draft).
6. Build `scripts/batch_missions.py` and `sim/scenarios/`.

**Open questions flagged for resolution:**

- Should disconfirmation be altitude-gated? Provisionally **yes**, ±2 m, because
  a single-plane LiDAR cannot speak for obstacles outside its plane
  (`methodology.md` § 2.3). Needs confirming against the world design.
- What is the *actual* false-report rate of the deployed reporter? Measurable
  from E1's logs, and it determines whether RQ1's premise holds. **If it is ~0,
  the paper must pivot from "false" to "stale" obstacles.**
- Is the sparse representation genuinely bandwidth-motivated at this scale?
  Must be measured (bytes/mission, sparse vs dense) rather than asserted.
