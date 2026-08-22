# Run guide

Everything needed to fly a delivery mission, in order. If you have to ask
someone a question to get through this, that is a defect in this guide — please
fix the guide.

---

## Prerequisites

| Requirement | Check |
| --- | --- |
| PX4-Autopilot at `~/PX4-Autopilot` | `ls ~/PX4-Autopilot/Makefile` |
| Gazebo Harmonic | `gz sim --versions` → 8.x or later |
| ROS 2 Humble, sourced | `ros2 --help` |
| Python 3.10+ | `python3 --version` |

Set `PX4_DIR` if PX4 lives elsewhere.

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

### Then verify, before dispatching anything

```bash
python3 scripts/preflight.py
```

**If preflight fails, do not dispatch.** It checks the things whose absence is
invisible from the Python side: that the model-scoped camera and LiDAR topics are
actually advertised, that the live image dimensions match the intrinsics the
landing controller is derived from, that the scan geometry matches the avoider's
sector maths, that every pad texture really decodes to its expected ArUco ID, and
that MAVSDK and the obstacle service answer.

Two of the seven root causes behind the last remediation — an undecodable landing
pad, and a sensor suite living in an uncommitted third-party submodule — stayed
hidden for months precisely because nothing ran this check. The drone armed, took
off and flew blind, and the only symptom was a mission that "didn't quite work".

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

### Stopping

```bash
scripts/stop_system.sh
```

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
| `SAFE_DIST` | 6.5 | Frontal distance that triggers a dodge |
| `ESCALATION_LOCK_S` | 12.0 | Dodge duration before a detour is planned |
| `GEOFENCE_RADIUS_M` | 500.0 | Also uploaded to PX4 as a real fence |

After changing anything in the camera or pad group, run `make schema-check`.

---

## Reference

| Document | Contents |
| --- | --- |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Contract per hop, message schemas, FSM diagram, the offboard invariant |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Camera intrinsics, sign derivation, marker budget, battery model |
| [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md) | Every acceptance criterion and how it is verified |
| [STATUS.md](STATUS.md) | What works, with evidence for each claim |
| [CHANGELOG.md](CHANGELOG.md) | History |
