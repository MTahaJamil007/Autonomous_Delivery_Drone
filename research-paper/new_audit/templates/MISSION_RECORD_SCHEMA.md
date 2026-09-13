# Mission record schema

One JSON object per mission. This is the atom of every result in both papers and
the unit of the released dataset.

**A record missing any `REQUIRED` field is discarded, not repaired.** Repairing
provenance after the fact is how irreproducible results happen.

---

## Top level

```jsonc
{
  // ── identity ──────────────────────────────────────────── REQUIRED
  "schema_version": "1.0",
  "experiment_id":  "P1-E03",
  "cell_id":        "P1-E03/V4-disc/removal@11/margin=6.5",
  "run_id":         "P1-E03-V4-s0041",      // one RUN = the unit of analysis
  "mission_index":  11,                      // 0-based within the run
  "policy_variant": "V4-disc",
  "tier":           "T1",                    // "T1" | "T2"

  // ── provenance ────────────────────────────────────────── REQUIRED
  "git_commit":       "003f0a5…",
  "git_dirty":        false,                 // true INVALIDATES the record
  "config_hash":      "sha256:…",
  "container_digest": "sha256:…",            // null for T1-only runs
  "seeds": { "scenario": 41, "world": 41, "rng": 41, "torch": 41 },
  "software": {
    "px4": "v1.17.0-…", "gazebo": "harmonic", "ros2": "humble",
    "mavsdk": "2.12.10", "python": "3.10.12",
    "numpy": "…", "torch": "…"
  },
  "host": { "cpu": "…", "cores": 16, "ram_gb": 64, "os": "…",
            "parallel_instances": 4 },
  "started_at": "2026-09-13T14:02:11Z",
  "finished_at": "2026-09-13T14:05:07Z",
  "wall_clock_s": 176.4,

  // ── configuration under test ──────────────────────────── REQUIRED
  "fleet_size": 3,
  "drone_id": "drone-0",
  "flags": {
    "memory_conditioned_planning": true,     // the F8 fix
    "freespace_channel": true,               // the F9 channel
    "disconfirmation": true,
    "evidence_decay": true,
    "source_reliability": false,
    "confidence_aware_clearance": false,
    "attribution_rule": "independent",       // | "normalised"
    "decay_tau_days": 14,
    "clearance_margin_m": 6.5,
    "sensor": { "fov_half_deg": 35, "range_m": 12.0,
                "noise_sigma_m": 0.0, "dropout_rate": 0.0, "bias_m": 0.0 }
  },

  // ── scenario and ground truth ─────────────────────────── REQUIRED
  "scenario": {
    "family": "S3-removed",
    "world_family": "A",
    "true_obstacles": [
      { "id": "gt-0", "lat": 30.0318, "lon": 72.3140, "radius_m": 3.0,
        "present_from_mission": 0, "present_to_mission": 10 }
    ],
    "injected_false_reports": [],
    "degraded_reporter": null,               // e.g. {"drone_id":"drone-2","bias_pct":-40}
    "comms": { "delay_ms": 0, "loss_rate": 0.0 }
  },

  // ── belief store ──────────────────────────────────────── REQUIRED
  "belief_before": [
    { "id": "obs-7f3a", "lat": …, "lon": …, "radius_m": 3.0,
      "confidence_stored": 0.87, "confidence_effective": 0.83,
      "first_seen": "…", "last_confirmed": "…",
      "sources": ["drone-0","drone-1"], "confirmations": 6, "contradictions": 0 }
  ],
  "belief_after": [ /* same shape */ ],
  "belief_updates": [
    { "t": 41.2, "obstacle_id": "obs-7f3a", "kind": "disconfirm",
      "kappa": 0.62, "observer": "drone-0", "reliability": 1.0,
      "confidence_before": 0.83, "confidence_after": 0.61 }
  ],

  // ── what the drone did ────────────────────────────────── REQUIRED
  "planned_route": [[lat,lon], …],           // the route BEFORE flying: F8's output
  "flown_path": [ { "t": 0.1, "lat": …, "lon": …, "alt": …,
                    "v_n": …, "v_e": …, "v_d": … } ],
  "legs": [ { "role": "pickup", "marker_id": 0, "arrived": true,
              "landed": true, "t_land": 63.1, "touchdown_error_m": 0.31 } ],
  "dodge_episodes": [ { "t_start": …, "t_end": …, "direction": "DODGE_LEFT",
                        "min_eff_front_m": 4.9 } ],
  "escalations": [ { "t": …, "dodge_age_s": 12.1,
                     "detour_planned": true, "n_waypoints": 2 } ],
  "freespace_observations": [
    { "t": …, "lat": …, "lon": …, "heading_deg": …, "alt_m": …,
      "sector_min_m": [ /* one min-range per angular bin */ ] }
  ],
  "fsm_history": [ { "t": …, "from": "…", "to": "…", "event": "…" } ],

  // ── outcomes ──────────────────────────────────────────── REQUIRED
  "outcome": "SUCCESS",                      // SUCCESS | FAILED | ABORTED | EXCLUDED
  "failure_stage": null,                     // FSM state, when not SUCCESS
  "exclusion_reason": null,                  // infra | provenance | safety_violation

  "metrics": {
    "duration_s": 176.4,
    "path_length_m": 421.3,
    "geodesic_length_m": 388.0,
    "excess_distance_m": 33.3,
    "energy_proxy_m_s": 391.4,               // ∫|v| dt — A PROXY. Label it so.
    "energy_pct_used": 18.2,
    "dodges": 0,
    "escalations": 0,
    "unnecessary_avoidances": 1,             // detours around beliefs with no ground truth
    "verification_detours": 0,               // Paper 2
    "verification_distance_m": 0.0,          // Paper 2
    "bytes_exchanged": 3120,
    "belief_rows": 4,
    "query_latency_ms_p50": 3.1
  },

  "safety": {
    "collisions": 0,
    "min_clearance_m": 8.7,
    "near_misses": 0,
    "near_miss_threshold_m": 3.0,            // fixed BEFORE the campaign
    "recollision_after_forgetting": 0,
    "supervisor_interventions": [],
    "geofence_breaches": 0,
    "battery_reserve_violations": 0
  }
}
```

---

## Field rules

| Rule | Why |
| --- | --- |
| `git_dirty: true` ⇒ record invalid | Uncommitted changes cannot be reproduced |
| `energy_proxy_m_s` is never called "energy" in any figure or table | `BATTERY_ENERGY_PER_M_PCT` is a placeholder and payload mass is unsimulated |
| `planned_route` is recorded **before** flying | It is the only direct evidence that memory influenced routing (F8) |
| `true_obstacles` comes from the generator, never from the belief store | Otherwise belief-quality metrics are circular |
| `near_miss_threshold_m` is written into every record | Proves the threshold was not chosen after seeing data |
| `sector_min_m` bin edges are in `flags.sensor` | The observation model must be reconstructable from the record alone |
| `run_id` is stable across all missions in a run | The run is the unit of analysis (`04` § 3.1) |
| Timestamps are UTC, ISO-8601 | Batch campaigns cross timezone and DST boundaries |

## Storage

- One file per mission: `results/<experiment_id>/<run_id>/<mission_index>.json`
- One `run.json` per run with the run-level roll-up and the exit status
- Compressed per experiment for archival; the DOI release ships the compressed
  form plus the split files
- Never edited in place. A correction is a new record with
  `supersedes: "<record id>"` and a reason.

## Versioning

`schema_version` is mandatory. When the schema changes, increment it and write a
migration in `research/evaluation/`. Analysis code reads the version and refuses
records it was not written for — silently mixing schema versions across a
campaign is a quiet way to corrupt a result.
