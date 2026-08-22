<!--
  The remediation plan this release implements, recovered from the published
  artifact and converted to Markdown so it lives with the code it describes.

  Original: https://claude.ai/code/artifact/39484417-1062-4ae5-8ad8-6df3dd6a0d8d
  Baseline verified 15 Aug 2026 on this host.

  KEPT AS WRITTEN, including the parts implementation proved wrong. The
  corrections are recorded in CHANGELOG.md under "Corrections to the plan" -
  MAVSDK's version and URL scheme, PX4_GZ_MODEL_NAME's actual behaviour, PX4's
  gz_env.sh clobbering the asset paths, and the confidence-merge semantics.
  Editing the plan to match the outcome would destroy the record of what was
  assumed versus what was measured, which is the more useful artefact.

  For current state, read STATUS.md. For the contracts, docs/ARCHITECTURE.md.
-->

# DroneProgram Remediation Plan (as written, 15 Aug 2026)

> **Historical document.** See [../STATUS.md](../STATUS.md) for what is actually
> true now, and [../CHANGELOG.md](../CHANGELOG.md) for where implementation
> contradicted this plan.

DroneProgram Remediation Plan

    Engineering remediation plan · autonomous delivery drone

# Making DroneProgram do what its documentation says

    Eight verification-gated phases that close the gap between STATUS.md and the code that
      actually flies. Every fix below is anchored to a measured fact about this machine, not an assumption
      about PX4 in general.

      Repository~/DroneProgram · 6,799 LOC

      AutopilotPX4 v1.17.0-alpha1-1225

      StackPython 3.10 · ROS 2 Humble · gz Harmonic

      Estimated effort≈12.5 engineer-days

          - §1Target

          - §2Baseline

          - §3New findings

          - §4Architecture

          - Phases

          - P0Freeze

          - P1Dispatch

          - P2Perception

          - P3Mission core

          - P4Landing

          - P5Detour

          - P6Fleet

          - P7Docs & CI

          - Reference

          - §5Standards

          - §6Risk

          - §7Sequencing

          §1 — Definition of done

## What "works as described in the docs" means

        STATUS.md and RUN_GUIDE.md describe a specific system. Taken literally, the
          documentation commits to nine behaviours. The plan is complete when each one is demonstrable and covered
          by a named test or logged procedure — not before.

            Documented behaviour | Source | State today | Closed by |

              3-leg delivery mission ends in FSM DONE, job COMPLETED | STATUS.md L110–117 | Crashes at dispatch | P1, P3 |

              Precision landing on ArUco IDs 0 / 1 / 2 with disambiguation | STATUS.md L12 | Pad undetectable | P2, P4 |

              LiDAR reactive dodging with time hysteresis | STATUS.md L14, L23 | Works | P2 (hardening) |

              Obstacle escalation after 12 s triggers detour planning | config.py L42 | Never emitted | P2, P5 |

              Battery monitoring gates each leg | STATUS.md L16 | Dead code | P3 |

              Safety supervisor: heartbeat + geofence with direct authority | STATUS.md L80 | Never instantiated | P3 |

              Payload bay kinematically follows the airframe | STATUS.md L26 | Never instantiated | P3 |

              Fleet-wide obstacle sharing, persisted between missions | RUN_GUIDE.md L71 | In-memory only | P5 |

              Multi-drone fleet (3 drones, queued jobs) | STATUS.md L17, L38 | Single-drone globals | P3, P6 |

          Governing decision
          drone_agent/ becomes the mission core; drone_web/drone_logic.py becomes a thin
            per-drone orchestrator. Two parallel implementations exist. The documented feature set — FSM,
            marker disambiguation, payload, supervisor, battery gating, escalation — lives only in
            drone_agent/, which nothing imports. The code that flies is the older, simpler
            drone_logic.py.

          Every phase below assumes that direction. The alternative (port features into drone_logic.py)
            discards ~1,800 lines of working, unit-tested modules and re-earns their bugs.

          §2 — Measured baseline

## Facts this plan rests on

        All values below were read off this machine, not inferred. Several contradict what a generic PX4 plan
          would assume, and three of them change the fixes materially.

            Subject | Measured | Consequence |

                Simulation assets |
                ~/PX4-Autopilot/Tools/simulation/gz is a dirty git submodule (git status → m) |
                The drone's entire sensor suite is an uncommitted local edit inside a third-party repo. See F2. |

                Downward camera |
                x500_base/model.sdf:315 — downward_camera, 320×240, hfov 1.047 rad, pose 0 0 -0.05 0 1.5708 0, topic /camera/image, 10 Hz |
                vision_bridge.py's hardcoded centre 160/120 is correct. Derived fx = 277.1 px. |

                LiDAR |
                x500_base/model.sdf:333 — rplidar_a1, gpu_lidar, 360 samples over −π…+π, range 0.15–12.0 m, topic /lidar/scan, 10 Hz |
                1°/sample; INF_REPLACE=15 > CLEAR_DIST=9 ✓. Self-return risk at prop radius ≈0.29 m. See F5. |

                World |
                worlds/default.sdf, origin 30.0314722 °N, 72.3140833 °E; great_wall box 30×1×20 m at (0, 15, 10); landing_pad = model://arucotag at (5, 0) |
                Leaflet map centre [30.0315, 72.3140] matches ✓. A ready-made obstacle exists for P5 validation. |

                Marker models |
                arucotag_0/1/2 exist in PX4 and decode as DICT_4X4_50 IDs 0, 1, 2; 0.5 × 0.5 m plane; quiet zone present |
                Disambiguation is achievable exactly as documented — world/marker_models.py is right. |

                Generic arucotag |
                354×354 texture; fails to decode in all 27 predefined dictionaries as rendered. Decodes as 4×4 ID 0 only after adding a white border. |
                The pad drone_logic.py:135 spawns cannot be seen by the detector. See F1. |

                MAVLink ports |
                px4-rc.mavlink:5 → offboard remote port = 14540 + instance |
                config.MAVSDK_PORT_BASE = 14540 ✓. Instances >9 collapse to 14549 — cap the fleet at 10. |

                MAVSDK |
                2.8.4. Battery.remaining_percent documented 0–100; Health exposes is_global_position_ok, is_home_position_ok, is_armable; LandedState, param, geofence plugins present |
                Battery units are fine on this version; the range changed from 0–1 in 1.x. Pin it. LandedState gives a proper touchdown signal. |

                Test suite |
                12 pass. test_avoider_node.py → collection ImportError. test_marker_disambiguation.py:80 → blocks on input() indefinitely. |
                One import error aborts the whole run, so pytest tests/ reports nothing. |

                pytest environment |
                pytest 9.1.1 + ROS Humble's launch_testing entrypoints → PluginValidationError before collection |
                Any run inside a ROS-sourced shell fails. Needs PYTEST_DISABLE_PLUGIN_AUTOLOAD=1. |

                Version control |
                git ls-files → 7 files. config.py, drone_agent/, fleet_dispatch/, tests/, world/ all untracked. |
                No rollback, no bisect, no safety net for any of the work below. |

          §3 — Root causes found during planning

## Seven findings that reshape the work

        These are additions to the code-review findings, uncovered by inspecting the simulation assets and doing
          the sensor arithmetic. Each one would have silently defeated a fix applied only at the Python layer.

              F1

### The landing pad is invisible to the ArUco detector

              Blocker

            drone_logic.py:135 spawns model://arucotag for all three pads. That model's
              texture fills the plane edge-to-edge with no white quiet zone, and ArUco's contour stage cannot find a
              marker without one. Decoding the texture directly fails across every predefined dictionary; adding a
              40 px white border makes it decode as 4×4 ID 0.

            So vision_data["locked"] never becomes True, landing falls into the "rotate
              slowly and search" branch, times out after 45 s, re-takes off, and the mission continues as if nothing
              happened. This is the root cause of precision landing never working — independent of
              every other bug. arucotag_0/1/2 are correctly formed and must be used instead.

            arucotag     354×354  NOT DECODABLE (all 27 dicts)
arucotag_0   250×250  DICT_4X4_50 → [0]
arucotag_1   250×250  DICT_4X4_50 → [1]
arucotag_2   250×250  DICT_4X4_50 → [2]
arucotag + 40px white border → DICT_4X4_50 → [0]

              F2

### The sensor suite lives in an uncommitted third-party submodule

              Blocker

            The downward camera and the 360° LiDAR were hand-added to
              ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf, and the wall plus a landing
              pad were hand-added to worlds/default.sdf. Tools/simulation/gz is a git
              submodule and shows as modified.

            A single git submodule update --force, a PX4 rebase, or a fresh clone silently removes
              every sensor. The drone would then arm, take off, and fly blind with /camera/image and
              /lidar/scan simply absent — and today nothing checks for them. These assets are
              project assets and belong in DroneProgram/sim/.

              F3

### A 0.5 m marker is 14 pixels wide at cruise altitude

              Blocker

            With fx = (w/2)/tan(hfov/2) = 160/tan(0.5235) = 277.1 px, a marker of side
              s at altitude h spans fx·s/h pixels. A 4×4 ArUco tag
              needs roughly 25–30 px to decode reliably.

            px = fx · s / h  =  277.1 × 0.5 / h  =  138.6 / h

h = 10 m  →  13.9 px   undecodable — the drone searches at cruise altitude
h =  5 m  →  27.7 px   marginal
h =  3 m  →  46.2 px   reliable

            The documented flow — arrive at TARGET_ALT = 10 m, then search — therefore cannot lock even
              with correct pads. Two levers fix it and the plan uses both: 2.0 m pads (55 px at 10 m, and realistic for
              a delivery pad) plus an explicit acquisition altitude with margin.

              F4

### The landing controller's loop gain scales with 1/altitude

              Instability

            Both landing implementations command velocity proportional to pixel error
              (K_p = 0.015 m/s per px). Pixel error for a fixed ground offset grows as the drone
              descends, so the effective gain in the physical loop is K_p·fx/h = 4.16/h s⁻¹.

            h = 10 m  →  K_eff = 0.42 s⁻¹   sluggish but stable
h =  1 m  →  K_eff = 4.16 s⁻¹   overshoot and oscillation, exactly at touchdown

            Converting pixels to metres with the altitude first, then applying a fixed metric gain, makes the loop
              altitude-invariant. It also delivers most of what solvePnP was deferred for
              (STATUS.md S1.5) without needing camera calibration.

              F5

### Blocking calls inside the control loop will drop OFFBOARD mode

              Flight safety

            payload.py:149 runs subprocess.run("gz service …") synchronously from an
              async tick scheduled at NAV_HZ = 10. Each invocation blocks the whole event loop for tens to
              hundreds of milliseconds, which starves the offboard setpoint stream. PX4 requires setpoints faster than
              2 Hz and reverts to a failsafe mode when the stream gaps.

            So enabling the payload feature as documented would, by itself, cause mid-flight mode loss. The same
              pattern appears in spawn_gazebo_marker (drone_logic.py:139) and in
              vision_bridge.py's cv2.imshow inside a transport callback. The structural fix
              is a dedicated setpoint publisher task (P3.2) plus async subprocess I/O.

              F6

### The camera sign convention is correct — and now derived

              Confirmed

            Worth recording, because it is the one thing easiest to "fix" into a crash. The mount is
              pose 0 0 -0.05 0 1.5708 0: a +90° pitch about Y. Gazebo cameras look down +X with image
              right = −Y and image down = −Z.

            R_y(90°): X_cam → (0,0,−1) down ✓   Y_cam → (0,1,0) body-left   Z_cam → (1,0,0) body-fwd
image u (right) = −Y_cam → body right      image v (down) = −Z_cam → body aft
⇒ FRD:  v_forward = −K·err_y      v_right = +K·err_x      matches existing code

            Keep the signs. P4.7 adds a one-time empirical confirmation and writes it into
              docs/CALIBRATION.md so nobody re-derives it under pressure.

              F7

### Sensor topics are global, so a fleet cannot work

              Blocks P6

            The hand-added sensors hardcode <topic>/camera/image</topic> and
              <topic>/lidar/scan</topic>. Those are absolute gz topic names, so every spawned
              drone publishes onto the same two topics. With FLEET_SIZE = 3, all three vision bridges see
              a blend of three cameras and all three avoiders see a blend of three LiDARs.

            Per-drone ports in config.py (PORT_STRIDE) solve the UDP half of this but not
              the gz half. The model must use model-scoped topics, and the topic name must become a CLI argument to
              both the vision bridge and ros_gz_bridge.

          §4 — Target architecture

## One contract per hop

        Most integration bugs here are producer/consumer disagreements that no single file reveals. The table
          below is the contract the phases implement; it becomes ARCHITECTURE.md in P7 and is the
          reference for every schema change.

            Producer | Transport | Payload | Consumer |

                Gazebo downward_camera |
                gz topic (model-scoped) |
                gz.msgs.Image 320×240 RGB |
                perception/vision_bridge.py |

                vision_bridge.py |
                UDP 5005 + 10·i |
                JSON {t, seq, w, h, fx, detections:[{id, err_x, err_y, corners, area_px}]} |
                DroneMission.vision_data |

                Gazebo rplidar_a1 |
                gz → ros_gz_bridge |
                sensor_msgs/LaserScan 360 pts |
                avoider_node.py |

                avoider_node.py |
                UDP 5006 + 10·i |
                JSON {t, seq, action, eff_front_m, dodge_age_s}; action ∈ CLEAR · DODGE_LEFT · DODGE_RIGHT · ESCALATE |
                DroneMission.lidar_data |

                PX4 SITL |
                MAVLink UDP 14540 + i |
                MAVSDK telemetry / offboard |
                DroneMission + setpoint publisher |

                DroneMission |
                HTTP :5050 |
                Obstacle report / bbox query |
                obstacle_memory_service/ (aiosqlite) |

                DroneMission |
                in-process registry |
                get_state_snapshot(drone_id) |
                fleet_dispatch/app.py → SQLite → WebSocket → Leaflet |

### Files that stop existing

        Duplication is the mechanism by which fixes get applied to the wrong copy. Four deletions, each with its
          survivor named:

            - drone_web/app.py → survivor fleet_dispatch/app.py (its template already has the
            dispatch-result error handling)

            - obstacle_memory_service.py (in-memory, own constants) → survivor
            obstacle_memory_service/ (persistent, reads config)

            - working code reference/ → byte-identical to the active avoider_node.py; git
            history replaces it

            - perception/ as an empty directory → becomes the real home of vision_bridge.py

        Plus one consolidation: get_distance_m is defined six times with identical bodies. One copy in
          drone_agent/geo.py.

          § Phases — each gated on the previous

## P0 → P7

        Phases are ordered by dependency, not by size. Do not start a phase until the previous phase's
          acceptance criteria pass. That rule is what prevents the situation the project is in now, where
          nine features are marked complete and none of them run.

            P0

### Freeze the baseline

              Create the safety net and make the environment honest. No behaviour changes — this phase
                only makes later phases reversible and verifiable.

            0.5 dNo flight

                - P0.1
                Commit everything. git add -A on all 1,517 files minus ignores. Add
                  .gitignore entries for *.db, *.log, __pycache__/,
                  vision_env/, .pytest_cache/. Delete working code reference/ and
                  the empty perception/ placeholder. Commit the deleted commands.md.

                - P0.2
                Vendor the simulation assets out of PX4 and into the project (fixes F2).
                  Create sim/models/x500_delivery/ that merge-includes stock x500 and adds the
                  camera and LiDAR with model-scoped topics (pre-empting F7), plus
                  sim/worlds/delivery.sdf carrying the world origin and great_wall but not the
                  undecodable landing_pad. Add sim/env.sh:

                # sim/env.sh — source before launching anything
export GZ_SIM_RESOURCE_PATH=$PROJECT/sim/models:$GZ_SIM_RESOURCE_PATH
export PX4_GZ_WORLDS=$PROJECT/sim/worlds
# launch: PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_delivery \
#         PX4_GZ_WORLD=delivery make px4_sitl

                Record the original PX4 edits as a patch in sim/patches/ so the current setup stays
                  reproducible while the migration is validated.

                - P0.3
                scripts/preflight.py — fail fast, fail loud. Refuses to dispatch unless
                  every check passes: gz topic -l advertises the camera and LiDAR topics; the camera's
                  actual width/height match config; the LiDAR's angle_min,
                  angle_increment and range_max match config; MAVSDK connects on
                  14540+i; the obstacle service answers on 5050. This is the check whose absence let
                  F2 and F1 stay hidden.

                - P0.4
                Un-break the test harness. Add pytest.ini with
                  asyncio_mode = auto and markers sitl / hitl; default
                  addopts = -m "not sitl and not hitl". Add scripts/test.sh exporting
                  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 and loading -p asyncio (the ROS Humble
                  launch_testing plugins are incompatible with pytest 9 and abort collection). Remove
                  input() from test_marker_disambiguation.py:80 and mark that test
                  @pytest.mark.sitl. Move from pathlib import Path below the docstring in the
                  four test files where it silently demotes the docstring.

                - P0.5
                Make it a real package. Add pyproject.toml, install with
                  pip install -e ., and delete all nine sys.path.insert(...) blocks. They are
                  why fleet_dispatch/app.py and obstacle_memory_service/app.py only import from
                  one specific working directory. Pin mavsdk>=2.0,<3 —
                  Battery.remaining_percent changed from 0–1 to 0–100 between 1.x and 2.x and the current
                  >=1.4.0 spans that break.

            Acceptance

                - git status clean; git ls-files | wc -l > 60

                - scripts/test.sh → 12 passed, 0 errors, 0 hangs, exits in <30 s

                - scripts/preflight.py passes with the sim up; with vision_bridge killed it
                fails naming the missing topic

                - python -c "import drone_agent, fleet_dispatch, global_planner, perception" works from
                /tmp

            P1

### Dispatch path integrity

              A dispatch either flies or reports why, and never wedges the fleet. Smallest phase that
                converts the system from "silently broken" to "diagnosable".

            1 dNo flight

                - P1.1
                Declare the contract once. In drone_web/drone_logic.py:
                  async def execute_delivery(job: DeliveryJob, drone_id: str) -> MissionResult and
                  def get_state_snapshot(drone_id: str) -> dict. A dataclass argument ends the 4-vs-5
                  positional-argument class of bug permanently.

                - P1.2
                Fix the import that wedges the fleet. fleet_dispatch/app.py:93 imports a
                  non-existent get_state_snapshot from inside run_drone_task but above its
                  try:, so the finally: complete_job() never runs and the drone stays
                  BUSY forever. Move both imports to module scope so a broken build fails at startup, and
                  delete the stub fallback at lines 26–36 — a dispatch that cannot fly must raise, not log and return
                  success.

                - P1.3
                Honest job lifecycle. QUEUED → ASSIGNED → IN_PROGRESS → COMPLETED | FAILED |
                  ABORTED. Replace complete_job(job_id) with
                  finalize_job(job_id, status, detail). Today every outcome, including a crash, is recorded
                  as COMPLETED.

                - P1.4
                Atomic drone claim. dispatch() currently does create → get_available →
                  assign across three separate connections, so two concurrent requests can claim the same drone. Use one
                  WAL-mode connection and a conditional update, then check rowcount:

                UPDATE drones SET status='BUSY', current_job_id=?
 WHERE id=? AND status='AVAILABLE'   -- rowcount 0 ⇒ lost the race, re-queue

                - P1.5
                Drain the queue. Nothing currently starts a QUEUED job. On every
                  finalize_job, pop the oldest queued job and dispatch it. Without this, the documented
                  "job queueing" behaviour is a one-way write.

                - P1.6
                Housekeeping. datetime.now(timezone.utc) in place of the deprecated
                  utcnow() (4 sites in fleet_dispatch/db.py). One heartbeat writer task per
                  mission for lat/lon/alt/battery, and surface FAILED reasons in the existing status line
                  in templates/index.html.

            Acceptance

                - Dispatch with PX4 offline → job FAILED with a reason, drone back to
                AVAILABLE within 5 s, reason visible in the browser

                - Two simultaneous dispatches at FLEET_SIZE=1 → exactly one ASSIGNED, one
                QUEUED; the queued one starts automatically when the first finalizes

                - 50 rapid-fire dispatches leave no drone stuck BUSY with no live mission

            P2

### Perception contracts

              Make the two sensor feeds say what the consumers expect, timestamp them, and give the
                existing fail-closed checks something real to read.

            1.5 dBench + hover

                - P2.1
                perception/vision_bridge.py. CLI --drone-id,
                  --topic, --headless. Build the dictionary and detector once
                  (currently rebuilt every frame). Derive the image centre from msg.width/height rather than
                  hardcoding 160/120 — correct today, silently wrong the moment the model changes. Emit all
                  detections, not corners[0][0]; disambiguation is impossible while the producer discards
                  every marker but the first. Publish the JSON schema from §4, including empty-detection frames so that
                  "no marker" and "bridge dead" become distinguishable. Send to
                  VISION_PORT_BASE + PORT_STRIDE·i.

                - P2.2
                avoider_node.py: extract the decision function and emit ESCALATE. Pull
                  the state machine out of _cb into a pure function with exactly the signature
                  tests/test_avoider_node.py already expects — the test was written against an API that was
                  never built:

                def decide_action(eff_front, med_left, med_right, now,
                  dodge_dir, dodge_start_t, clear_first_seen,
                  SAFE_DIST, CLEAR_DIST, CLEAR_CONFIRM_S,
                  MIN_LOCK_S, ESCALATION_LOCK_S)
    → (action, dodge_dir, dodge_start_t, clear_first_seen)

                Emit ESCALATE once now − dodge_start_t ≥ ESCALATION_LOCK_S (12 s) — the
                  action navigation.py:293 and config.py:42 both already anticipate and which
                  nothing has ever sent. Import config instead of re-declaring the five thresholds as class
                  constants. Add MIN_VALID_RANGE_M = 0.5 to mask self-returns (props sit ≈0.29 m out, above
                  the 0.15 m sensor minimum, so a prop hit currently reads as a permanent 0.29 m wall). Validate scan
                  geometry against config on the first message. Publish JSON with monotonic t
                  and seq on every scan, so silence means failure.

                - P2.3
                drone_agent/udp_receiver.py — the missing half of fail-closed.
                  navigation.py:210 and safety_supervisor.py:144 both read
                  last_lidar_ts, and safety_supervisor.py:103 reads
                  last_telemetry_ts. Nothing anywhere writes either. So the nav guard holds
                  position forever and the heartbeat check computes time.time() − 0 and triggers an
                  emergency RTL on its first tick. One shared reader fixes both: drain the socket to the newest datagram
                  (never process a backlog of stale actions), parse, and stamp
                  last_*_ts = time.monotonic().

                - P2.4
                One time base. Replace time.time() with time.monotonic()
                  for every duration, timeout and staleness check across the avoider, navigation, landing and
                  supervisor. Wall clock is subject to NTP steps; a backwards step in flight makes a timeout fire
                  instantly or never.

            Acceptance

                - tests/test_avoider_node.py passes unmodified, including the ESCALATE case

                - New replay test: a recorded LaserScan sequence drives CLEAR → DODGE → ESCALATE → CLEAR
                with no wall-clock dependency

                - Hovering in an empty world, logged eff_front ≈ 15.0 m for 60 s — no self-returns

                - Killing avoider_node.py mid-flight trips the nav hold within 1 s; restarting it resumes
                within 1 s

                - Two markers in frame → both appear in one datagram with correct distinct IDs

            P3

### Mission core

              Adopt drone_agent/, isolate per-drone state, and guarantee the offboard
                setpoint stream. The largest phase, and the one that turns nine "dead" features live.

            3 dFlight

                - P3.1
                DroneMission class (the documented R1.3). Owns
                  drone_state, vision_data, lidar_data, its ports, its FSM,
                  supervisor and payload bay. Module-level globals at drone_logic.py:54–56 are what limit
                  the system to one drone. A module-level registry {drone_id: DroneMission} backs
                  get_state_snapshot.

                - P3.2
                A single setpoint publisher — the key structural fix. One task per mission at 20 Hz
                  that unconditionally sends whatever is in self._setpoint; mission code only ever mutates
                  that object. Enter offboard once per leg and never stop mid-leg.

                This removes an entire bug class at once: the start/stop thrashing the V5 docstring itself warns
                  about, the mixing of set_velocity_body and set_velocity_ned in one session
                  (landing.py:432), and the F5 stalls. PX4 needs setpoints above 2 Hz;
                  making that a structural invariant rather than a property of every code path is the difference between
                  "usually works" and "works".

                - P3.3
                Rewrite execute_delivery as an FSM-driven orchestrator.
                  IDLE → TAKEOFF → ENROUTE → SEARCHING → APPROACH → DESCEND → PAYLOAD_OP → NEXT_LEG ×3
                  → DONE, driving the existing MissionFSM whose transition table is already
                  correct and unit-tested. Legs come from world/marker_models.py, which finally makes marker
                  IDs 0 / 1 / 2 real instead of the hardcoded 0, 0, 0 at
                  drone_logic.py:493/509/525:

                LEGS = [(pickup, 0, payload.attach),
        (dropoff, 1, payload.release),
        (home,    2, None)]

                A failed leg must route to HOVER_AND_ALERT or ABORT. Today
                  if landed: at line 494 has no else, so a failed landing silently proceeds to
                  the next leg from an unknown altitude.

                - P3.4
                Wire the four documented-but-dead features: pre-leg
                  battery.check_leg_battery; SafetySupervisor as a task for the mission's
                  lifetime; PayloadBay.attach/release at the two PAYLOAD_OP entries; and a
                  single fetch_known_obstacles_for_mission(bbox) before leg 1, cached for the mission as
                  its docstring specifies.

                - P3.5
                Fix the supervisor before trusting it with authority. Three defects, each of which
                  makes it dangerous rather than merely inert:

                    - safety_supervisor.py:192 — the RTL→land fallback is unreachable, because
                    _trigger_emergency_rtl sets _emergency_triggered = True at line 178 and the
                    fallback returns immediately at 201–202. Pass an override, or set the flag only after a command is
                    accepted. As written, an RTL failure means no action at all.

                    - :103 — require last_telemetry_ts > 0 before arming the heartbeat
                    check, so a cold start cannot fire an emergency RTL at t=0.

                    - :114–122 — enable the commented-out battery check now that battery telemetry is
                    plumbed, and additionally upload a real PX4 geofence via MAVSDK's geofence plugin.
                    Autopilot-enforced limits survive a Python crash; a supervisor task does not.

                - P3.6
                Unreachable obstacle reporting. navigation.py:270–291 assigns
                  last_action = action in the first if and then tests
                  action != last_action in the second — always false, so a dodge is never reported to fleet
                  memory. Capture prev_action before reassigning. Also hold a reference to the
                  fire-and-forget asyncio.create_task(report_obstacle(...)) and log its exceptions;
                  unreferenced tasks can be garbage-collected mid-flight and swallow their errors.

                - P3.7
                Acceleration limiting instead of tick blending. Replace the
                  blend_velocity counter (navigation.py:313–318) with a slew limit of ≤2 m/s²
                  per axis. It is the physically meaningful constraint, it matches MPC_ACC_HOR, and it
                  behaves correctly when an action changes mid-blend — which the counter does not.

                - P3.8
                Async Gazebo I/O and a cargo model. asyncio.create_subprocess_exec for
                  spawn and pose-set (fixes F5); cargo following in its own 10 Hz task, never inline in
                  the nav tick. Create sim/models/cargo_box/ — no cargo model exists anywhere on this
                  machine, so PayloadBay.update() has been addressing a non-existent entity. Spawn pads via
                  PAD_MODELS (arucotag_0/1/2), never model://arucotag
                  (F1).

                - P3.9
                Apply the PX4 tuning parameters in code. config.py:82–111 documents
                  seven MPC_* values as a comment block asking the operator to type them into a PX4 shell.
                  Set them at mission start with drone.param.set_param_float and read them back to confirm.
                  Manual steps do not survive a reboot and cannot be reviewed.

                - P3.10
                Delete the duplicates per §4, and make drone_logic.py import
                  config. Right now it re-declares TARGET_ALT, CRUISE_SPEED and
                  eight more at lines 41–50, so tuning config.py has no effect on flight — the single most
                  confusing property of the current codebase.

            Acceptance

                - POST /dispatch → 3-leg mission reaches FSM DONE, job
                COMPLETED, drone AVAILABLE

                - Zero OFFBOARD-rejected or setpoint-timeout messages in the PX4 console for the whole
                mission

                - Cargo box visibly tracks the airframe between pickup and drop, and stays put after release

                - Two DroneMission instances in one process keep independent state (unit test, no sim)

                - Killing the mission task mid-flight → supervisor RTLs the drone; job ABORTED

                - grep -c "TARGET_ALT\s*=" drone_web/drone_logic.py → 0

            P4

### Precision landing that actually locks

              Close F1, F3 and F4 — the three
                reasons vision-guided landing has never worked, in the order the drone encounters them.

            2 dFlight

                - P4.1
                Control in metres, not pixels (F4). Derive
                  fx once from the reported image width and hfov, convert pixel error to a
                  ground offset using AGL altitude, then run a fixed-gain controller:

                fx        = (w/2) / tan(hfov/2)                # 277.1 px measured
offset_m  = err_px * alt_agl_m / fx
v_cmd     = clamp(K * offset_m, ±1.5)          # K ≈ 0.8 s⁻¹, deadband 0.05 m

                Apply the EMA to the metric offset, not to the output velocity as
                  landing.py:329 does — filtering the command adds lag to the actuator instead of
                  removing noise from the measurement. This also delivers most of the value that
                  solvePnP was deferred for, with no calibration rig.

                - P4.2
                Gate the descent. Descend only while the offset stays inside a cone that narrows
                  with altitude — |offset| < 0.35·h + 0.15 m — and climb to re-centre if it leaves.
                  Schedule v_z 0.8 → 0.35 → 0.15 m/s below 1.5 m. Currently the drone descends at 0.4 m/s
                  even when badly off-centre (landing.py:339), which converts a lateral error at 8 m into
                  a missed pad at 0 m.

                - P4.3
                Acquisition altitude and pad size (F3). Add
                  SEARCH_ALT_M and PAD_SIZE_M to config.py with the
                  px = fx·s/h budget written next to them. Ship 2.0 m pads
                  (sim/models/pad_0|1|2, the arucotag_N textures on a 2×2 plane → 55 px at
                  10 m) and descend to SEARCH_ALT_M = 6 before searching, for margin at both ends. Either
                  lever alone is fragile; together they give roughly 4× headroom.

                - P4.4
                Real touchdown detection. Use LandedState.ON_GROUND plus
                  armed == False, keeping alt < 0.30 m only as a fallback. And delete the
                  per-tick telemetry subscription at drone_logic.py:386 — async for … break
                  inside a 10 Hz loop opens a fresh MAVLink stream every 100 ms. drone_state["alt"] is
                  already maintained by the telemetry task.

                - P4.5
                Make the spiral search a real search. Hold altitude during it — the search setpoint
                  at landing.py:433 sends v_z = 0 with no altitude correction, so the drone
                  drifts vertically for the whole pattern. Set ring spacing to 0.6 × ground footprint
                  (2h·tan(hfov/2)) so coverage overlaps instead of striping. Cap
                  effective_timeout_s at 240 s; the current
                  max(LANDING_TIMEOUT_S, dynamic) over 100+ waypoints at 1 m/s yields a multi-minute hover.
                  On exhaustion fire search_exhausted into HOVER_AND_ALERT, not a bare
                  return False.

                - P4.6
                One offboard session. Route search and descent setpoints through the P3.2 publisher
                  rather than calling offboard.start()/stop() and switching setpoint types
                  mid-loop.

                - P4.7
                Record the calibration. One scripted ground test that commands a known lateral
                  offset and logs the observed pixel motion, confirming F6's derived signs. Write the
                  result, the derivation and fx into docs/CALIBRATION.md.

            Acceptance

                - 20 consecutive scripted landings on IDs 0 / 1 / 2 from 8 m altitude and ±5 m lateral offset:
                ≥19 touchdowns within 0.5 m of pad centre

                - Zero landings on the wrong marker ID, with all three pads within 15 m of each other

                - Zero oscillation events below 3 m altitude (offset sign changes >3 times)

                - Marker lock acquired within 20 s of arriving at SEARCH_ALT_M in ≥18 of 20 runs

            P5

### Escalation → detour → memory

              Close the loop the docs describe but never connect: a wall that defeats reactive dodging
                becomes a planned route, and a second mission benefits from the first.

            1.5 dFlight

                - P5.1
                Replace the TODO at navigation.py:293–302. On ESCALATE
                  (now actually emitted, per P2.2), pass the mission's cached obstacles plus the live dodge position to
                  plan_detour(current, target, obstacles, margin_m=SAFE_DIST), fly the returned waypoints
                  through the same nav tick, then resume the leg. Bound to max_iterations=3, then
                  HOVER_AND_ALERT. The current handler logs a warning and continues flying at the wall,
                  which is the worst of the available behaviours.

                - P5.2
                Report obstacles once per wall. With P3.6's fix the report finally fires; add a 10 m
                  dedupe so a 30 m wall is not reported fifty times during one dodge, and seed the reported position
                  from the drone's position projected eff_front_m along its bearing rather than the drone's
                  own coordinates.

                - P5.3
                Switch to the persistent service. Run obstacle_memory_service/
                  (aiosqlite, reads config) and delete the root obstacle_memory_service.py
                  that RUN_GUIDE.md step 3 currently launches — it keeps obstacles in a dict, so
                  "fleet-wide obstacle sharing" resets on every restart and
                  config.OBSTACLE_MERGE_RADIUS_M is ignored. Fix its from db import … to a
                  package-relative import so it runs from any directory.

                - P5.4
                Make prefetch change behaviour. With the wall in memory, the pre-leg
                  plan_detour should route around it before departure, so run 2 records zero
                  DODGE events. That is the observable difference between an obstacle database and an
                  obstacle log.

            Acceptance

                - Run 1 across great_wall: dodge → ESCALATE at ≈12 s → detour → arrival, no
                collision, no timeout

                - obstacles.db holds exactly one row for the wall with confidence >0.5 and the correct
                reporter

                - Run 2 on the same route: zero DODGE_* actions, path deviates around the wall from
                departure

                - Restarting the obstacle service preserves the row

            P6

### Fleet

              Three drones flying three jobs concurrently, which the UI has always claimed and the
                architecture has never allowed.

            2 dFlight

                - P6.1
                Model-scoped sensor topics (F7). Absolute
                  /camera/image and /lidar/scan in x500_base mean N drones share
                  two topics. Use the default scoped topic names in sim/models/x500_delivery and pass the
                  topic to vision_bridge.py and ros_gz_bridge as an argument — one instance of
                  each per drone.

                - P6.2
                Rewrite spawn_fleet.sh to the recipe. Build once with
                  make px4_sitl, then launch build/px4_sitl_default/bin/px4 -i $i per instance
                  with PX4_GZ_STANDALONE=1 for i>0, plus
                  PX4_GZ_MODEL_NAME=drone_$i and PX4_GZ_MODEL_POSE, against one
                  gz sim server. The current script calls make inside the loop, which
                  serialises builds and races on the build directory. Cap the fleet at 10:
                  px4-rc.mavlink:6 collapses instances above 9 onto port 14549.

                - P6.3
                Per-drone everything. FLEET_SIZE = 3; ports from
                  config.drone_index (14540+i, 5005+10i, 5006+10i); one DroneMission, FSM,
                  supervisor and payload bay per drone. Remove the hardcoded "drone-0" defaults at
                  navigation.py:283 and in PayloadBay.

                - P6.4
                Fleet view. Three live markers on the existing Leaflet map — one WebSocket per
                  drone, which /ws/{drone_id} already supports — plus a per-drone status row driven by
                  /fleet/status.

            Acceptance

                - Three concurrent dispatches → three airborne drones → three COMPLETED jobs

                - No cross-talk: each drone lands only on its own leg's marker; each avoider reacts only to its own
                LiDAR

                - A fourth dispatch queues, then starts automatically when the first drone frees

                - Killing one drone's PX4 instance fails only that job; the other two complete

            P7

### Docs and guardrails

              Make the documentation a description rather than an aspiration, and make regressions
                loud. The phase that prevents this plan from being needed again.

            1 dNo flight

                - P7.1
                Resolve HOW_TO_RUN.md. Seven references point at a file that does not
                  exist, two of them inside runtime error strings the operator sees mid-incident
                  (navigation.py:222, safety_supervisor.py:153). Pick one filename and grep
                  to zero.

                - P7.2
                Rewrite STATUS.md from the code. Rule: nothing gets ✅ without a call
                  site, and every claim carries a "Verified by" column naming a test ID or a logged procedure. The
                  current table marks the FSM, payload and supervisor "Working" while nothing imports them, and asserts
                  that pytest tests/ succeeds when it aborts at collection.

                - P7.3
                Add ARCHITECTURE.md containing §4's contract table, the FSM diagram,
                  the offboard-stream invariant, and the marker/altitude budget from F3. The
                  producer/consumer schema mismatch between vision_bridge.py's CSV and
                  landing.py's JSON expectation existed because no document owned that boundary.

                - P7.4
                make check and CI. ruff, scripts/test.sh, and an import
                  smoke test from a neutral working directory — the exact check that would have caught the
                  get_state_snapshot blocker at commit time. Run it on push.

                - P7.5
                One history. A single CHANGELOG.md replaces the R1–R6 fix-round
                  narrative scattered through docstrings and status files; git carries the rest.

            Acceptance

                - grep -rn HOW_TO_RUN . resolves to a file that exists

                - Every ✅ in STATUS.md maps to a test ID or a dated procedure log

                - make check green from a clean clone plus pip install -e .

                - A new operator completes a mission using only RUN_GUIDE.md, with no verbal help

          §5 — Cross-cutting

## Standards that apply to every phase

        Each of these generalises a specific bug found in this codebase. They are cheap as conventions and expensive
          as one-off fixes.

            Rule | Why, here |

              Monotonic time for durations, UTC ISO-8601 only for records | Every timeout and staleness check currently uses time.time(); an NTP step in flight fires or defeats them. |

              Never block the control loop — no sync subprocess, no cv2.imshow, no unbounded I/O in a tick | F5. PX4 drops OFFBOARD when setpoints gap; the mission code cannot be the thing that gaps them. |

              Fail closed on sensor loss — hold, alert, never assume CLEAR | Already the intent at navigation.py:213; P2.3 supplies the timestamp that makes it real. |

              Units in every name — _m, _m_s, _pct, _px, _deg, _rad | The pixel/metre confusion in F4 and the 0–1 vs 0–100 battery ambiguity are both unit bugs that a name would have caught. |

              config.py is the only source of tunables | drone_logic.py:41–50 and avoider_node.py:18–25 shadow it today, so the file that claims to centralise tuning changes nothing. |

              Structured logging with drone_id; no print() in flight code | drone_logic.py prints ~40 times; a 3-drone fleet's output becomes unreadable, and nothing is machine-parseable post-flight. |

              Every safety threshold gets a unit test | Geofence radius, battery reserve, escalation delay and stale timeout are all single numbers with no coverage. |

              One schema change ⇒ update §4's table in the same commit | The CSV-vs-JSON mismatch shipped because producer and consumer were edited in different weeks. |

          §6 — Risk register

## What can still go wrong

            Risk | Likelihood | Impact | Mitigation |

              A PX4 submodule reset silently removes camera and LiDAR | High | Total — drone flies blind | P0.2 vendoring + P0.3 topic preflight |

              0.5 m pads remain undetectable at cruise altitude | Certain | Landing never locks | P4.3 — 2 m pads and SEARCH_ALT_M |

              A blocking call drops OFFBOARD mid-flight | Medium | Failsafe / uncommanded descent | P3.2 publisher + P3.8 async I/O |

              LiDAR self-returns from props read as a wall | Medium | Permanent dodge, mission stalls | P2.2 range mask + hover check |

              Detour recursion produces long paths in clutter | Medium | Battery abort mid-leg | max_iterations=3 → HOVER_AND_ALERT; battery gate |

              Multi-drone gz topic collision | Certain at N>1 | Wrong drone reacts to wrong sensor | P6.1 scoped topics; P6 gated on it |

              Flat-earth geodesy error at range | Low | ~2 m at 10 km | Documented limit; 500 m geofence keeps it irrelevant |

              SITL behaviour does not transfer to hardware | Medium | Rework at flight test | Keep every threshold in config; recalibrate fx and pad size per P4.7 |

          §7 — Sequencing

## Order, effort, and the gates between

            Phase | Effort | Depends on | Gate to pass before moving on |

              P0 Freeze | 0.5 d | — | Tests run clean; preflight fails loudly when it should |

              P1 Dispatch | 1.0 d | P0 | Failure path returns the drone to AVAILABLE |

              P2 Perception | 1.5 d | P0 | ESCALATE emitted; staleness detected within 1 s |

              P3 Mission core | 3.0 d | P1, P2 | 3-leg mission reaches DONE with no OFFBOARD loss |

              P4 Landing | 2.0 d | P3 | 19 of 20 landings within 0.5 m, correct IDs |

              P5 Detour | 1.5 d | P3, P2 | Run 2 avoids the wall with zero dodges |

              P6 Fleet | 2.0 d | P3, P4, P5 | 3 concurrent missions, no cross-talk |

              P7 Docs & CI | 1.0 d | All | New operator flies a mission from the guide alone |

        P0 and P1 together are one day and unblock everything. They convert the system from
          "dispatch does nothing and wedges the fleet" to "dispatch flies or tells you why", which is the difference
          between debugging and guessing. P2 can proceed in parallel with P1 — they touch disjoint files.

          Two things to accept before starting
          The docs oversold completion, and that is the real defect to fix. Nine features were
            marked complete on the strength of the code existing rather than running. The per-phase acceptance gates
            above exist specifically to make that failure mode impossible to repeat — a feature is done when a named
            test or a logged procedure demonstrates it, not when the module compiles.

          Some of this work is re-verification, not new code. Roughly a third of
            drone_agent/ is sound and unit-tested; it has simply never been executed against a running
            autopilot. Expect the first flight after P3 to surface two or three defects of the same kind as
            F5 — bugs that only exist once the code actually runs. That is planned for, not a
            setback.

        DroneProgram remediation plan
        Baseline verified 15 Aug 2026 on this host
        PX4 v1.17.0-alpha1-1225-g4a48525e45 · MAVSDK 2.8.4 · ROS 2 Humble

