# 10 — What the code actually does

Audited **2026-09-13** against branch `remediation` @ `003f0a5`, 12,237 lines of
Python across the flight stack. Every row is `VERIFIED-CODE`: it was read, and
the `file:line` is given so it can be checked in thirty seconds.

The 2026-08-23 `notes/codebase-audit.md` remains accurate about *what modules
exist*. This document corrects it about **how they are wired**, which is where
the research premise lives.

---

## 1. F8 — the finding that blocks the research programme

> **The shared obstacle memory has no influence on any route until a reactive
> avoidance episode has already failed for 12 seconds.**

The chain, in full:

| Step | Evidence |
| --- | --- |
| Memory is fetched once per mission, before leg 1 | `drone_agent/mission.py:386` `_prefetch_obstacles()` |
| ...into `self.known_obstacles` | `drone_agent/mission.py:403` |
| ...and handed to the navigator | `drone_agent/mission.py:556` |
| The navigator initialises the route to **the straight line** | `drone_agent/navigation.py:247` — `waypoints = [(target_lat, target_lon)]` |
| `known_obstacles` is read in exactly one place | `drone_agent/navigation.py:383` |
| ...which is inside `if action == "ESCALATE":` | `drone_agent/navigation.py:352` |
| ...which the avoider emits only after 12 s of continuous dodging | `config.py:142` — `ESCALATION_LOCK_S = 12.0` |

`plan_detour()` is never called at leg start. There is no code path from a
remembered obstacle to a pre-emptive route.

### Why this is fatal to the plan as written

Every headline quantity in the 2026-08-23 experiment set is defined on behaviour
that cannot occur:

| Planned quantity | What actually happens today |
| --- | --- |
| *Unnecessary-avoidance rate* — detours around obstacles that are not there | **Identically zero.** A phantom in the database never produces a detour, because detours are triggered only by a live LiDAR return |
| *Time-to-forget* — missions until a removed obstacle stops affecting routing | **Undefined.** A removed obstacle stops affecting routing on the next mission, because it never affected routing to begin with |
| E2 phantom-obstacle cost | **No cost exists to measure** |
| E4 false-report dose-response | **Flat by construction** |
| ACCEPTANCE § 10 run-2 criterion — *"the flown path deviates around the wall from departure"* | **Cannot happen.** Run 2 dodges exactly as run 1 did |

`tests/test_detour_memory.py::test_a_known_obstacle_changes_the_planned_route`
passes — but it calls `plan_detour()` directly. It proves the *planner* can use
memory. It does not prove, and no test covers, that the *mission* ever asks it
to.

### What has to happen

A **memory-conditioned pre-plan** at leg start: call `plan_detour(start, goal,
known_obstacles, ...)` before entering the navigation loop and fly the result.
Roughly 20 lines, additive, feature-flagged.

Two consequences that must be accepted explicitly:

1. **It is platform, not contribution.** Pre-emptive routing from a shared
   obstacle store is what every fleet-memory system does. It belongs in the
   paper's *system description*, never in its contribution list.
2. **It redefines the baseline.** "V1 = the current system" is wrong. For
   routing purposes the current system is closer to *no memory at all*. The
   paper's primary baseline must be `V1 = memory-conditioned pre-planning with
   probabilistic-OR accumulation and wall-clock decay` — a baseline that has to
   be **built** before it can be beaten.

---

## 2. F9 — the disconfirmation signal does not reach the consumer

The LiDAR is a 360-sample planar scan at 10 Hz with 12 m range
(`config.py:151-156`). `avoider_node.py` reduces it to three scalars before it
crosses the UDP boundary (`avoider_node.py:421-429`):

```
{"action", "eff_front_m", "med_left_m", "med_right_m", "dodge_age_s", ...}
```

- `eff_front_m` is `min(median(front), min(front))` over ±35°
  (`avoider_node.py:274`, `config.py:172`).
- `med_left_m` / `med_right_m` are **medians** over the 40°–100° side sectors
  (`config.py:173-174`).

`methodology.md` § 2.3 proposes deriving negative evidence from `eff_front_m`.
That will not work, for a geometric reason:

- `DETOUR_MARGIN_M = SAFE_DIST = 6.5 m` (`config.py:446`, `config.py:138`)
- `OBSTACLE_DEFAULT_RADIUS_M = 3.0 m` (`config.py:439`)
- so a detour passes ≈ 9.5 m from a believed obstacle's centre — **abeam**, not
  ahead. It never enters the ±35° front cone.

A drone that successfully routes around a belief produces **no front-cone
evidence about it at all**. The only sectors that see it are the side sectors,
and those are reported as a *median over 60°*, which a 3 m disc at 9.5 m barely
moves.

### What has to happen

A dedicated free-space channel: a sectorised minimum-range vector (12–36 bins is
ample; 36 bins × 2 bytes ≈ 72 B per scan) emitted alongside the control scalars,
or as a separate lower-rate message. Additive to `avoider_node.py`; no change to
`decide_action()`.

**This decision determines whether Paper 1 has an effect to measure.** It is not
an implementation detail — it is the observation model, and it belongs in the
methodology section of the manuscript.

---

## 3. Confirmed from the 2026-08-23 audit (spot re-verified)

| Claim | Status | Evidence |
| --- | --- | --- |
| Confidence fuses by probabilistic OR | Confirmed | `obstacle_memory_service/db.py:88` |
| Confidence decays as `c·exp(−age_days/τ)`, τ = 14 d | Confirmed | `db.py:72-85`, `config.py:431` |
| Decay is applied **at query time**, not stored | Confirmed | `db.py:209-220` |
| Nothing ever writes negative evidence | Confirmed | no writer other than `add_or_merge_obstacle` |
| Obstacles are reported **only** on `entering_dodge` | Confirmed | `navigation.py:344-349` |
| Reports are deduplicated within 10 m, per mission | Confirmed | `navigation.py:92-103`, `config.py:433` |
| `plan_detour` never reads `confidence` | Confirmed | `global_planner/detour.py` — the field does not appear |
| Detour margin is a constant | Confirmed | `config.py:446` |
| `BATTERY_ENERGY_PER_M_PCT` is a placeholder | Confirmed | `config.py:398` = 0.01 |
| `FLEET_SIZE = 1` | Confirmed | `config.py:25` |
| ~126 automated tests | Confirmed (122 top-level + `tests/sitl_scenarios/`) | `tests/` |
| No tags in the repository | Confirmed | `git tag` is empty |

---

## 4. Things the research plan assumes that are not true yet

| Assumption | Reality | Consequence |
| --- | --- | --- |
| "Three-drone concurrency is structurally tested" | `FLEET_SIZE = 1`; ACCEPTANCE § 11 never run | **Both paper titles say "Multi-UAV."** If § 11 fails, the titles are unsupportable |
| "Run 2 records zero dodges" is the system's success metric | Cannot occur — see § 1 | The project's own acceptance criterion is currently unachievable |
| Energy is measurable | Placeholder coefficient; payload mass unsimulated (`drone_agent/payload.py` is kinematic) | Any energy claim is a proxy claim and must be labelled as one |
| The world supports scenario variation | One static obstacle (`great_wall`) in `sim/worlds/delivery.sdf`; `world/build_world.py` is not on the mission path | The scenario generator is net-new work, not a modification |
| ≈ 3 minutes per headless mission | Never measured | The 4,250-mission budget rests on an unmeasured constant — see `04_experiment_protocol.md` § 6 |

---

## 5. What still must not be claimed as novelty

The 2026-08-23 list stands unchanged and is repeated here because it is the most
useful page in the old folder. None of the following is a contribution:

ArUco precision landing · three-leg delivery missions · reactive 2-D LiDAR
dodging with hysteresis · escalation from reactive to geometric detour ·
persisting obstacles in a database with confidence and decay · fleet dispatch
with a job queue · PX4 + MAVSDK + ROS 2 + Gazebo + web UI integration · a safety
supervisor with heartbeat, geofence and battery checks · kinematic payload
attachment.

**Add to that list, as of this audit:**

- **Pre-emptive routing from a shared obstacle store** (the F8 fix). Standard.
- **Negative evidence and forgetting in a sparse map.** Prior art — see
  `05_literature_verification.md` § 2.
- **Per-source reliability weighting in multi-robot fusion.** Prior art — see
  `05_literature_verification.md` § 3.
- **Using PPO.** A method, not a result.

---

## 6. Where the remaining engineering risk sits

None of the following is research, and all of it is on the critical path.

| Item | Why it is on the critical path | Rough size |
| --- | --- | --- |
| ACCEPTANCE § 4 (three-leg mission) | Every experiment is a mission | Unknown — `STATUS.md` predicts 2–3 first-flight defects |
| ACCEPTANCE § 11 (three drones) | The word "Multi-UAV" in both titles | Unknown |
| F8 fix (pre-emptive memory routing) | The phenomenon under study | ~20 lines + tests |
| F9 free-space channel | The observation model | ~60 lines + tests |
| Scenario generator | Every experiment beyond E1 | ~300 lines |
| Headless batch runner with resume | 10³–10⁴ missions | ~250 lines |
| Structured per-mission logging | Every metric | ~80 lines |
| Throughput measurement | The entire experiment budget | 1 day |

Total ≈ 700 lines of new infrastructure **before** a single line of the method
is written. Plan for it as such.
