# Methodology

Status: **draft v0.1** (2026-08-23). For D1, disconfirmation-aware shared
obstacle memory. This file is a living document — every methodological change
gets recorded here with a dated entry in `progress-log.md`.

---

## 1. System under study

`DroneProgram` @ `91abd5e`. PX4 v1.17 SITL + Gazebo Harmonic + ROS 2 Humble +
MAVSDK 2.12.10. See `notes/codebase-audit.md` for the complete inventory and,
importantly, for **what already exists and therefore cannot be claimed**.

The method is a small, feature-flagged change to three modules
(`obstacle_memory_service/db.py`, `drone_agent/navigation.py`,
`global_planner/detour.py`). Everything else is held constant. **Baseline and
treatment are the same binary with different flags** — this eliminates a whole
class of confound and should be stated in the paper.

---

## 2. Formal setup

### 2.1 Belief state

The shared store holds records
`oᵢ = (pᵢ, rᵢ, cᵢ, tᵢ, sᵢ)` — position (lat, lon), radius, confidence ∈ [0,1],
last-confirmed time, source drone. This is the existing schema.

### 2.2 Positive evidence (existing, unchanged)

On entering a dodge, the reporter places an obstacle at the drone's position
projected along its heading by the measured clear distance, and the store merges
it with any record within `OBSTACLE_MERGE_RADIUS_M = 5 m` using a probabilistic
OR, `c ← c + c'(1−c)`, with confidence-weighted position averaging.

### 2.3 Negative evidence (new)

A free-space observation is `f = (q, θ, d, τ, s)` — observer position, heading,
measured clear range `eff_front_m`, time, source. It asserts that the sector of
half-width `FRONT_HALF_DEG` from `q` along `θ` is clear out to `d`.

For each stored obstacle `oᵢ` whose disc intersects that swept sector, define
the **contradiction depth**

```
δᵢ = clamp( d − ‖q − pᵢ‖ + rᵢ , 0 , 2rᵢ )
```

— how far the clear reading penetrates past the near edge of the obstacle's
disc. `δᵢ = 0` means the reading stops short and contradicts nothing;
`δᵢ = 2rᵢ` means the reading passes clean through the whole disc.

Normalised contradiction `κᵢ = δᵢ / (2rᵢ) ∈ [0,1]`, and the update

```
cᵢ ← cᵢ · (1 − λ · κᵢ · ρ(s))
```

where `λ` is the disconfirmation gain and `ρ(s) ∈ [0,1]` the observer's
reliability. **Multiplicative, not subtractive**, so confidence approaches zero
asymptotically and cannot go negative — mirroring the probabilistic-OR used for
positive evidence.

**Attribution across overlapping records.** Because records are merged and their
discs can overlap, a single free-space observation may contradict several. The
default rule is *independent application to each intersecting record*; the
alternative — normalising `λ` by the number of intersecting records — is an
ablation, not an assumption. **Both must be tested (E7).**

**Open design question, to resolve before implementation:** should `κ` account
for the *vertical* dimension? The LiDAR is a single horizontal plane at cruise
altitude; a clear reading says nothing about an obstacle below or above that
plane. `great_wall` is 20 m tall so this does not bite in the current world, but
a low obstacle overflown at 10 m would be wrongly disconfirmed. **Decision:
restrict disconfirmation to observations taken within ±2 m of the altitude at
which the obstacle was reported, and record this as an explicit limitation.**

### 2.4 Evidence-driven forgetting (new)

Replace `c_query = c · exp(−Δt/τ)` with a rule in which the effective age is
measured in *contradicting-observation opportunities* rather than seconds: an
obstacle repeatedly flown past without contradiction retains confidence; one
whose region is repeatedly traversed clear loses it. Wall-clock decay is
retained as a slow floor for regions nobody visits, since absence of evidence
there is genuinely uninformative.

### 2.5 Source reliability (new)

`ρ(s)` is a Beta posterior over each drone's agreement with fleet consensus on
*overlapping* observations only. **Abstention is mandatory** where a drone's
observations do not overlap anyone else's: `ρ` stays at its prior rather than
being penalised. This is the failure case identified in RQ4 and must be handled
by construction.

### 2.6 Confidence-aware clearance (new)

`plan_detour` currently uses a constant `DETOUR_MARGIN_M`. Replace with
`margin(c) = m_min + (m_max − m_min)·c`, so low-confidence obstacles are given
less berth. Also introduce a *routing threshold*: obstacles below `c_min` are
not routed around at all — the reactive avoider remains the safety net.

---

## 3. Experimental design principles

1. **Same binary, different flags.** Baseline and treatment differ only by
   configuration.
2. **Fixed seeds, ≥30 repetitions per cell.** SITL is non-deterministic;
   report distributions and non-parametric tests (Mann–Whitney U, Cliff's δ),
   not means alone.
3. **Effect sizes with confidence intervals**, always. A p-value alone is not a
   result.
4. **Pre-register the hypothesis for each experiment** in `experiments.md`
   *before running it*, including the expected direction. Record surprises as
   surprises.
5. **Safety is a guardrail, not an outcome.** Minimum clearance and near-miss
   counts are reported for every condition, prominently. A method that improves
   routing by flying closer to obstacles has not improved anything.
6. **Negative results are written up.** The fallback in `selected-topic.md`
   depends on this being a genuine commitment.
7. **The oracle and the floor are always reported.** Without both, a percentage
   improvement is uninterpretable.

---

## 4. Instrumentation

Per mission, one JSON record:

```
mission_id, scenario_id, policy_variant, seed, drone_id,
path: [(t, lat, lon, alt, v_n, v_e, v_d)],
dodge_episodes: [(t_start, t_end, direction, min_eff_front_m)],
escalations: [(t, dodge_age_s, detour_planned, n_waypoints)],
detour_waypoints, legs: [(role, marker_id, arrived, landed, t_land, error_m)],
energy_proxy_m, energy_pct_used, battery_trace,
min_clearance_m, near_misses, collisions,
obstacle_db_before, obstacle_db_after, freespace_observations,
fsm_history, outcome, failure_stage
```

Ground truth (true obstacle positions and lifetimes) comes from the scenario
generator, so precision/recall and unnecessary-avoidance are computable exactly.

---

## 5. Threats to validity, stated up front

| Threat | Type | Mitigation |
| --- | --- | --- |
| Single simulator, single airframe, single world family | External | Vary world parameters widely; run the optional physical validation; state the limitation plainly |
| SITL LiDAR is cleaner than a real RPLIDAR | Construct | Inject range noise and dropout (E9); calibrate against real sensor specs |
| Scenarios designed by the same person who designed the method | Internal | Pre-register scenarios *before* implementing the operator; include scenarios where the method is expected to lose (E5 fast-change) |
| Baseline under-tuned | Internal | Grid-search τ for baseline 3; report the search |
| Effect driven by one scenario | Conclusion | Report per-scenario breakdowns, never only the aggregate |
| Metrics chosen after seeing results | Conclusion | Fix the metric set in this file before Phase D |
| Payload mass and wind unmodelled | Construct | State as a limitation; the energy metric is a proxy until calibrated |
| Single-plane LiDAR cannot disconfirm out-of-plane obstacles | Construct | Altitude-gated disconfirmation (§ 2.3); stated as a limitation |

---

## 6. Reproducibility commitments

- Every experiment runs from a committed config; the commit hash goes in the
  mission record.
- The scenario generator is seeded and deterministic.
- Raw mission JSON is archived under `research-paper/results/`.
- Analysis is scripted, not manual.
- The existing repo gates (`make check`) must pass on every commit that touches
  flight code.
