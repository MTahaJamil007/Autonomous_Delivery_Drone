# Run guide

Everything needed to fly a delivery mission, in order. If you have to ask
someone a question to get through this, that is a defect in this guide — please
fix the guide.

---

## Quick start

Already set up? This is the whole thing:

```bash
cd ~/DroneProgram
source sim/env.sh          # 1. point Gazebo + PX4 at the vendored assets
world/spawn_fleet.sh       # 2. Gazebo server + PX4 instance(s)
scripts/run_system.sh      # 3. services, bridges, avoiders, vision
python3 scripts/preflight.py   # 4. MUST print PREFLIGHT PASS
```

Then open <http://localhost:5000>, click the map twice, press **Dispatch**.

Stop with `scripts/stop_system.sh`. First time on this machine? Start at
[Prerequisites](#prerequisites) instead — and read
[step 4](#then-verify-before-dispatching-anything), which is the one step people
skip and should not.

---

## Prerequisites

| Requirement | Check |
| --- | --- |
| PX4-Autopilot at `~/PX4-Autopilot` | `ls ~/PX4-Autopilot/Makefile` |
| Gazebo Harmonic | `gz sim --versions` → 8.x or later |
| ROS 2 Humble, sourced | `ros2 --help` |
| Python 3.10+ | `python3 --version` |

Set `PX4_DIR` if PX4 lives elsewhere.

**Setting up a new machine from scratch?** Follow
[docs/MIGRATION.md](docs/MIGRATION.md) first. It installs every row of this
table at the reference machine's exact versions, and `scripts/verify_env.sh`
confirms the result.

A `vision_env/` virtualenv may exist in the repo root on the original
development machine (it is git-ignored, so a fresh clone has none). It is
optional — it was created with `include-system-site-packages = true`, so it
sees the same packages the bare system `python3` does, and adds no isolation
on this host.
Activate it (`source vision_env/bin/activate`) if you want a dedicated shell
for this project; everything below works identically with or without it.

### One-time setup

```bash
cd ~/DroneProgram
make install
make check          # every gate must be green before you fly anything
```

`make install` does an editable install with `--no-build-isolation`. That flag is
needed on this host: the system `setuptools` (59.6) predates PEP 660, and pip's
isolated build picks it up and refuses the editable install with a confusing
message about a missing `build_editable` hook.

It also upgrades `setuptools` first, and skips `--user` for that upgrade when
run inside a virtualenv (`vision_env` included) — a venv's own site-packages
already wins the path race, so pip refuses `--user` there with *"Will not
install to the user site because it will lack sys.path precedence to
setuptools in .../site-packages"*. If you see that error, you are on an older
checkout; pull the latest `Makefile` or drop `--user` from that one line
yourself.

---

## Starting the system

Three commands, in this order. **The order matters** — the sensor topics do not
exist until the drones have spawned, so a bridge started earlier subscribes to
nothing and stays silent.

```bash
cd ~/DroneProgram
source sim/env.sh        # 1. point Gazebo and PX4 at the vendored assets
world/spawn_fleet.sh     # 2. Gazebo server + PX4 instance(s)
scripts/run_system.sh    # 3. services, bridges, avoiders, vision
```

**`scripts/run_system.sh` refuses to start on top of a previous run.** It
records every pid it starts in `/tmp/droneprogram/run_system.pids`, and if any
of those processes (checked against `/proc/<pid>/cmdline`, so a reused pid from
something unrelated cannot trip this) is still alive, it stops instead of
stacking a second avoider and a second vision bridge onto the same drone —
that is finding F7 all over again, and it would also overwrite the pid file so
`stop_system.sh` could no longer clean up the first stack. The fix is almost
always:

```bash
scripts/stop_system.sh
scripts/run_system.sh
```

If you are certain the recorded pids are stale (e.g. the machine rebooted and
they were reassigned to something else), `scripts/run_system.sh --force` skips
the check.

### Then verify, before dispatching anything

```bash
python3 scripts/preflight.py
```

**If preflight fails, do not dispatch.** It checks the things whose absence is
invisible from the Python side: that every pad's markers actually decode and stay
readable across the whole descent (not just at one altitude), that the
model-scoped camera and LiDAR topics are actually advertised, that the live image
dimensions match the intrinsics the landing controller is derived from, that the
scan geometry matches the avoider's sector maths, and that MAVSDK, the obstacle
service and the fleet dispatcher itself all answer.

Two of the seven root causes behind the P0–P7 remediation — an undecodable
landing pad, and a sensor suite living in an uncommitted third-party submodule —
stayed hidden for months precisely because nothing ran this check. The drone
armed, took off and flew blind, and the only symptom was a mission that "didn't
quite work". A later one — `fleet_dispatch` crashing during startup while
preflight still reported every check green — is why "fleet dispatch service" is
its own check now, hitting `/fleet/status` rather than a bare socket.

Then open <http://localhost:5000>.

### Flying a mission

1. Search for a location, or accept the default view (the world origin is
   30.0315 N, 72.3141 E).
2. **Click the map twice.** First click sets the pickup pad, second sets the
   drop-off.
3. Press **Dispatch**.

The drone takes off to 10 m, flies to the pickup, descends to 6 m, searches for
ArUco marker **0**, lands on it, picks up the parcel, then repeats for marker
**1** at the drop-off and marker **2** back home.

The fleet table shows live status per drone. The **Reason / detail** column is
where a failure explains itself — that column is the point. A failed job that
says only "FAILED" is barely more useful than the silent false "COMPLETED" this
system used to report.

### What success looks like

So you can tell a working system from a broken one without asking anybody.

**After `world/spawn_fleet.sh`** — the drone exists and carries its sensors:

```bash
$ gz topic -l | grep -c x500_delivery_0
20                       # non-zero is what matters
$ python3 -m sim_topics --drone-id drone-0
camera   /world/delivery/model/x500_delivery_0/link/camera_link/sensor/downward_camera/image
lidar    /world/delivery/model/x500_delivery_0/link/lidar_link/sensor/rplidar_a1/scan
ros-scan /drone_0/scan
```

**After `scripts/run_system.sh`** — preflight passes, all ten checks (one drone;
each extra drone adds five more: its own camera/lidar topic, camera/lidar
geometry, and mavsdk checks):

```
[  ok  ] landing pads decode          pad_0=[0, 10, 11, 12, 13], pad_1=[1, 20, 21, 22, 23], pad_2=[2, 30, 31, 32, 33]
[  ok  ] landing altitude budget      pad visible 0.69-6.43 m; search 6.0 m, commit 1.2 m, no-climb 3.0 m
[  ok  ] gazebo reachable             48 topics advertised
[  ok  ] drone-0: camera topic        /world/delivery/.../downward_camera/image
[  ok  ] drone-0: lidar topic         /world/delivery/.../rplidar_a1/scan
[  ok  ] drone-0: camera geometry     320x240, fx=277.2 px, 2.0 m pad spans 92 px at 6 m
[  ok  ] drone-0: lidar geometry      360 samples, angle_min=-3.1416 rad, range_max=12.0 m
[  ok  ] drone-0: mavsdk              udp://0.0.0.0:14540
[  ok  ] obstacle memory service      http://127.0.0.1:5050/ -> Obstacle Memory Service
[  ok  ] fleet dispatch service       http://127.0.0.1:5000/fleet/status -> 1 drone(s) registered

PREFLIGHT PASS - 10 checks, 0 failures
```

Each pad now carries five ArUco markers (a small centre one plus four larger
ones around it, per `world/pad_layout.py`) rather than one — a single large
marker decodes fine on the bench but goes out of frame below ~2.5 m in flight,
which is what produced the original "hovers and never lands" report. `landing
altitude budget` is the check that would have caught it: it fails if
`SEARCH_ALT_M`, `COMMIT_ALT_M` or `NO_CLIMB_ALT_M` in `config.py` fall outside
the altitude range the shipped pad geometry can actually see.

**The avoider is alive and seeing nothing** (the drone is on the ground in an
empty area, so `eff_front_m` should be the 15.0 m infinity substitute — a
*sustained* reading near 0.3 m means propeller returns are getting through):

```bash
$ tail -2 /tmp/droneprogram/avoider_0.log
scan geometry verified: 360 samples, angle_min=-3.1416 rad, range_max=12.0 m
CLEAR -> CLEAR | front=15.0m left=15.0m right=15.0m dodge_age=0.0s
```

**The camera is alive** and reports the geometry the landing controller assumes:

```bash
$ tail -1 /tmp/droneprogram/vision_0.log
first frame: 320x240, fx=277.2 px, 2.0 m pad spans 92 px at 6 m
```

**During a mission**, the FSM narrates itself:

```bash
$ grep -- '--\[' /tmp/droneprogram/fleet_dispatch.log | tail -5
[drone-0] IDLE --[start]--> TAKEOFF
[drone-0] TAKEOFF --[altitude_reached]--> ENROUTE
[drone-0] ENROUTE --[arrived]--> SEARCHING
[drone-0] SEARCHING --[marker_locked]--> APPROACH
[drone-0] APPROACH --[centered_stable]--> DESCEND
```

A finished mission ends `NEXT_LEG --[mission_complete]--> DONE` and the job shows
`COMPLETED`. **Anything else carries a reason** — in the browser's *Reason /
detail* column, and in `curl -s localhost:5000/jobs`.

**One number worth checking after every flight.** PX4 drops OFFBOARD if the
setpoint stream gaps for about half a second, so the publisher reports its worst
gap on shutdown:

```bash
$ grep 'setpoint stats' /tmp/droneprogram/fleet_dispatch.log
[drone-0] setpoint stats: {'publishes': 8412, 'send_failures': 0, 'max_gap_s': 0.0621, ...}
```

`max_gap_s` well under 0.4 and `send_failures: 0` is healthy. A large gap means
something blocked the event loop — that is finding F5, and `make blocking-check`
is the gate meant to prevent it.

### Stopping

```bash
scripts/stop_system.sh
```

Leaves `obstacle_memory_service/obstacles.db` in place, deliberately: that file
is what makes a second run of a route avoid a wall the first run discovered.

---

## What is running, and why

| Process | Port | Purpose | If it dies |
| --- | --- | --- | --- |
| `gz sim` | — | Physics and sensors | Everything stops |
| `px4` × N | MAVLink `14540+i` | Autopilot | That drone's mission fails; others continue |
| `obstacle_memory_service` | HTTP `5050` | Persistent fleet obstacle memory | Prefetch returns nothing; detours still work reactively |
| `fleet_dispatch` | HTTP `5000` | Job intake, assignment, UI | No new dispatches; live missions continue |
| `ros_gz_bridge` × N | — | Gazebo LiDAR → ROS `LaserScan` | **That drone holds position** (fail-closed) |
| `avoider_node` × N | UDP `5006+10i` | Avoidance decisions | **That drone holds position** (fail-closed) |
| `vision_bridge` × N | UDP `5005+10i` | ArUco detection | That drone cannot land; the search times out |

**One of each per drone.** That is the whole difference between this and a
single-drone launcher: the sensor topics are model-scoped, so drone-1's avoider
subscribes to drone-1's LiDAR and nothing else.

Logs are in `/tmp/droneprogram/`:

```bash
tail -f /tmp/droneprogram/avoider_0.log        # avoidance decisions
tail -f /tmp/droneprogram/fleet_dispatch.log   # mission progress
tail -f /tmp/droneprogram/px4_0.log            # autopilot
```

---

## Multi-drone

```bash
sed -i 's/^FLEET_SIZE = 1$/FLEET_SIZE = 3/' config.py
scripts/stop_system.sh
world/spawn_fleet.sh 3
scripts/run_system.sh 3
python3 scripts/preflight.py      # must pass for all three
```

Hard limit of **10 drones**: PX4's `px4-rc.mavlink` collapses every instance
above 9 onto port 14549, so instances 10+ would share one MAVLink port and fight
over it. `spawn_fleet.sh` refuses past that.

---

## Troubleshooting

### Preflight says a topic is not advertised

The drone has not spawned, or it spawned from a model without the sensors.

```bash
gz topic -l | grep x500_delivery      # nothing? the drone did not spawn
echo $GZ_SIM_RESOURCE_PATH            # must include ~/DroneProgram/sim/models
grep 'Spawning Gazebo model' /tmp/droneprogram/px4_0.log
```

Almost always: `sim/env.sh` was not sourced, so PX4 spawned the stock `x500`
(which has no camera and no LiDAR) instead of `x500_delivery`.

If `world/spawn_fleet.sh` printed `world 'delivery' already running, reusing
it`, you are launching a new PX4 instance into a Gazebo world left over from
an earlier session — run `scripts/stop_system.sh` first (it also sweeps stray
`gz sim`/`px4` processes), then `world/spawn_fleet.sh` again for a clean
world. Also give it a few seconds: model spawning happens after PX4 reports a
MAVLink heartbeat, so a `preflight.py` run immediately after
`world/spawn_fleet.sh` prints "Fleet launched" can catch the drone mid-spawn.
`scripts/run_system.sh` already accounts for this with its own delay.

### The drone takes off and then hovers, going nowhere

Working as designed. Navigation is **fail-closed**: with no obstacle data it
holds position rather than flying blind.

```bash
grep 'obstacle feed' /tmp/droneprogram/fleet_dispatch.log
ros2 topic hz /drone_0/scan          # should be ~10 Hz
```

Usually the `ros_gz_bridge` is not running, or ROS 2 was not sourced before
`run_system.sh` started it.

### The drone reaches the pad but never lands

```bash
grep -E 'marker|locked|spiral' /tmp/droneprogram/fleet_dispatch.log
tail -20 /tmp/droneprogram/vision_0.log
```

- *"no markers in view"* — the pads did not spawn. Check
  `grep 'spawn' /tmp/droneprogram/fleet_dispatch.log`.
- *"marginal size"* — the marker is too small in frame to decode. Check
  `config.PAD_SIZE_M` against `sim/models/pad_N/model.sdf`; `make schema-check`
  catches a mismatch.
- Lock acquired but the descent never starts — the drone is outside the descent
  cone. It will climb and re-centre; if it never converges, run the sign
  confirmation in [docs/CALIBRATION.md](docs/CALIBRATION.md) § 2.

### Dispatch says a job was assigned, then nothing happens

```bash
curl -s localhost:5000/jobs | python3 -m json.tool | head -30
```

The `detail` field carries the reason. A drone showing `BUSY` with
`mission_live: false` means it is marked busy with nothing running — the failure
mode the last remediation removed. Report it; it should not be possible.

### Drones are stuck BUSY after a crash

The dispatcher recovers these automatically at startup. To force it:

```bash
python3 reset_fleet_database.py
```

### `make test` / `make check` fails two `test_dispatch_endtoend.py` tests

```
a dispatch that cannot fly must be FAILED, got IN_PROGRESS
job ... ended as QUEUED; the queued job must be started by the drain step
```

These two tests assert what happens when a dispatch has **no autopilot to fly
against** — that is the scenario they are testing. If a real fleet (PX4 SITL)
is up in another terminal, the dispatcher finds a live drone instead, the
mission actually proceeds, and the job never reaches `FAILED` on the test's
short timeout, so it fails for the opposite of the right reason. Stop the
fleet before running the suite:

```bash
scripts/stop_system.sh
pkill -f 'gz sim'; pkill -f 'px4_sitl_default/bin/px4'
make test
```

If nothing else is running and this still fails, it is a real regression —
not this.

### A bridge log balloons to hundreds of MB and the rest of `run_system.sh` never comes up

```bash
tail -c 400 /tmp/droneprogram/ros_gz_bridge_0.log
# Exception sending a multicast message:Network is unreachable  (repeated forever)
```

`ros_gz_bridge` (a ROS 2 / DDS process) tried to discover peers over multicast
and every network interface was down. `sim/env.sh` sets `ROS_LOCALHOST_ONLY=1`
to stop it from trying — everything in this system talks over `127.0.0.1`
anyway — but that only takes effect for processes started **after** it is
sourced. Kill the runaway process, confirm `echo $ROS_LOCALHOST_ONLY` prints
`1`, and restart.

### Everything is wedged

```bash
scripts/stop_system.sh
pkill -9 -f 'gz sim'; pkill -9 -f px4
rm -f fleet_dispatch/fleet.db
```

`obstacles.db` is worth keeping — it is what makes the second run of a route
avoid a wall the first run discovered.

---

## Configuration

`config.py` is the **only** place tunables live. `make no-shadowed-config` fails
the build if one is re-declared elsewhere, because that is precisely what used to
make editing `config.py` have no effect on what flew.

Most likely to want changing:

| Constant | Default | Effect |
| --- | --- | --- |
| `FLEET_SIZE` | 1 | Number of drones |
| `TARGET_ALT_M` | 10.0 | Cruise altitude |
| `CRUISE_SPEED_M_S` | 3.5 | Maximum forward speed |
| `SEARCH_ALT_M` | 6.0 | Altitude for marker search — see CALIBRATION § 3 before changing |
| `PAD_SIZE_M` | 2.0 | Must match `sim/models/pad_N/model.sdf` |
| `COMMIT_ALT_M` | derived | Altitude below which the descent goes open-loop; derived from pad geometry, not chosen |
| `NO_CLIMB_ALT_M` | 3.0 | Floor below which the controller may not climb back away from the pad |
| `SAFE_DIST` | 6.5 | Frontal distance that triggers a dodge |
| `ESCALATION_LOCK_S` | 12.0 | Dodge duration before a detour is planned |
| `GEOFENCE_RADIUS_M` | 500.0 | Also uploaded to PX4 as a real fence |

After changing anything in the camera or pad group, run `make schema-check` —
and note `python3 scripts/preflight.py`'s **landing altitude budget** check now
also fails if `SEARCH_ALT_M`, `COMMIT_ALT_M` or `NO_CLIMB_ALT_M` fall outside
what the shipped pad markers can actually be seen across.

---

## Reference

| Document | Contents |
| --- | --- |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Contract per hop, message schemas, FSM diagram, the offboard invariant |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Camera intrinsics, sign derivation, marker budget, battery model |
| [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md) | Every acceptance criterion and how it is verified |
| [STATUS.md](STATUS.md) | What works, with evidence for each claim |
| [CHANGELOG.md](CHANGELOG.md) | History |
| [docs/LANDING_REWRITE_LOG.md](docs/LANDING_REWRITE_LOG.md) | Flight-by-flight log of the Sep 2026 precision-landing rewrite |
| [docs/IMPLEMENTATION_REPORT.md](docs/IMPLEMENTATION_REPORT.md) | What changed, what is left, and why |
| [docs/MIGRATION.md](docs/MIGRATION.md) | Rebuilding the whole environment on a new machine, with expected output per step |

Shareable web versions of this guide (*Dispatch Runbook*) and the handover
report (*Remediation Handover*) are linked at the top of
[docs/IMPLEMENTATION_REPORT.md](docs/IMPLEMENTATION_REPORT.md). This file is
the source of truth; update it first, then republish.
