# Migration: setting this project up on a new machine

How to rebuild the complete development environment on a fresh Ubuntu 22.04
install, so that it matches the machine the project was developed and verified
on (the *reference machine*) exactly. Every step lists what you should see; if
you see something else, stop there, because every later step assumes the
earlier ones worked.

Everything below was read from the reference machine on 28 Sep 2026, from
PX4's own `Tools/setup/ubuntu.sh`, or from the official ROS 2 and Gazebo
installation docs. For day-to-day running once set up, see
[RUN_GUIDE.md](../RUN_GUIDE.md).

---

## What the reference machine has

| Component | Version | Installed by |
| --- | --- | --- |
| Ubuntu | 22.04.5 LTS | — |
| Python | 3.10.12 (`/usr/bin/python3`), pip 22.0.2 | Ubuntu |
| ROS 2 | Humble, `ros-humble-desktop` 0.10.0 | step B |
| PX4-Autopilot | `main` @ `4a48525e4505d74970388476c24cc0cc642bdc5a` (`v1.17.0-alpha1-1225-g4a48525e45`) | step D |
| Gazebo | Harmonic, gz-sim 8.11.0 | step D (PX4's `ubuntu.sh`) |
| Gazebo ↔ ROS bridge | `ros-humble-ros-gzharmonic` 0.244.12 | step E |
| Gazebo Python bindings | `python3-gz-transport13` 13.5.0, `python3-gz-msgs10` 10.3.2 | step E |
| Python packages | 77 exact versions in `constraints/py310-lock.txt` | steps D and F |

`scripts/verify_env.sh` checks every row of this table.

### Four traps this procedure avoids

1. **The wrong bridge.** Humble's default `ros-humble-ros-gz` is built for
   Gazebo *Fortress*. Gazebo's own docs say the Harmonic packages "conflict
   with `ros-humble-ros-gz*` packages". With the wrong bridge the drone gets no
   LiDAR feed, and because navigation is fail-closed it then holds position
   forever. **Never install `ros-humble-ros-gz`.**
2. **protobuf.** It must stay at 3.20.x or `gz.msgs10` fails to import. Without
   the lock file it stays there only because mavsdk 2.8.4 happens to cap it.
3. **Upgrade before ROS.** The ROS docs warn that on 22.04, installing ROS
   before `systemd`/`udev` are updated "can trigger the removal of critical
   system packages".
4. **Order.** ROS goes before PX4, because PX4's setup reuses the `empy` that
   ROS installs. PX4 must be built before `sim/env.sh` works, because the build
   generates the `gz_env.sh` it sources.

---

## Before you start

- About **30 GB free** on the Ubuntu partition. PX4 alone is 10 GB once built.
- If the machine has an **NVIDIA GPU**, install its driver first from
  *Software & Updates → Additional Drivers*. With Secure Boot on, you will be
  asked to enroll a key on the next reboot.
- Keep the folder names `~/DroneProgram` and `~/PX4-Autopilot`. The scripts
  default to them (`sim/env.sh` honours `PX4_DIR` if you must move PX4).
- A working internet connection for the whole of steps A–F. Step D downloads
  several GB.

---

## A. Base system: upgrade first

```bash
sudo apt update && sudo apt upgrade -y && sudo reboot
```

After the reboot:

```bash
sudo apt install -y git curl software-properties-common
sudo add-apt-repository -y universe
```

**Expect:**

```
$ lsb_release -ds
Ubuntu 22.04.5 LTS
$ python3 --version
Python 3.10.12
$ locale | head -1
LANG=en_US.UTF-8            # any *.UTF-8 locale is fine
```

## B. ROS 2 Humble

These are the official commands from the ROS 2 Humble install page, in order.
Run them one at a time. The first line repeats step A's `curl` install on
purpose: without `curl`, every line after it fails, and the error you finally
see is a misleading `Unable to locate package ros-humble-desktop`.

```bash
sudo apt install -y curl software-properties-common
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
echo "$ROS_APT_SOURCE_VERSION"
```

**Expect** a version number (`1.3.0` on 28 Sep 2026, possibly newer). If it
prints a blank line, stop: `curl` is missing or GitHub was unreachable.

```bash
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

The `grep ... ||` guard adds the line to `~/.bashrc` only if it is not already
there, so re-running this step after a failure is safe.

**Expect:**

```
$ echo $ROS_DISTRO
humble
$ ros2 --help | head -1
usage: ros2 [-h] [--use-python-default-buffering]
```

## C. The project

The repository is public, so cloning over HTTPS needs no key.

```bash
git clone -b remediation https://github.com/MTahaJamil007/Autonomous_Delivery_Drone.git ~/DroneProgram
```

**Expect** the local commit to be the one on GitHub. Both commands print the
same 40-character ID:

```bash
git -C ~/DroneProgram rev-parse HEAD
git ls-remote https://github.com/MTahaJamil007/Autonomous_Delivery_Drone.git refs/heads/remediation
```

## D. PX4 at the exact commit, plus Gazebo Harmonic

```bash
git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
git checkout 4a48525e4505d74970388476c24cc0cc642bdc5a
git submodule update --init --recursive
PIP_CONSTRAINT=$HOME/DroneProgram/constraints/py310-lock.txt bash Tools/setup/ubuntu.sh
```

`ubuntu.sh` asks for your sudo password. It installs PX4's build tools, the
NuttX toolchain, **Gazebo Harmonic** (it adds the OSRF apt repository itself),
and PX4's Python packages. `PIP_CONSTRAINT` makes its unpinned
`requirements.txt` resolve to the reference machine's versions instead of
whatever is newest today.

**Expect:** it ends without any line starting `E:` (apt) or `ERROR:` (pip).
The script runs under `set -e`, so it stops at the first failure; `echo $?`
straight afterwards must print `0`.

It also adds you to the `dialout` group, so **reboot** now. Then build:

```bash
cd ~/PX4-Autopilot && make px4_sitl
```

**Expect:**

```
$ git -C ~/PX4-Autopilot describe --tags
v1.17.0-alpha1-1225-g4a48525e45
$ gz sim --versions
8.11.0                       # a later 8.x is fine; verify_env.sh will WARN
$ ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
/home/<you>/PX4-Autopilot/build/px4_sitl_default/bin/px4
/home/<you>/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
```

## E. The Gazebo ↔ ROS bridge and Gazebo's Python bindings

```bash
sudo apt install -y ros-humble-ros-gzharmonic python3-gz-transport13 python3-gz-msgs10
```

**Never `ros-humble-ros-gz`** (trap 1 above).

**Expect:**

```
$ dpkg-query -W -f='${Package} ${Version}\n' ros-humble-ros-gzharmonic python3-gz-transport13 python3-gz-msgs10
ros-humble-ros-gzharmonic 0.244.12-3jammy
python3-gz-transport13 13.5.0-1~jammy
python3-gz-msgs10 10.3.2-1~jammy
```

Newer versions than these are expected if the repositories have moved on;
`verify_env.sh` reports them as WARN, not FAIL.

## F. The project's Python packages, at exact versions

```bash
cd ~/DroneProgram
PIP_CONSTRAINT=$PWD/constraints/py310-lock.txt make install
```

**Expect:** two pip runs, the second ending with a
`Successfully installed ...` line that includes `droneprogram-0.8.0`, and no
line starting `ERROR:`.

---

## Verifying the result

Do these in order. Stop at the first one that does not match.

### 1. The environment

```bash
cd ~/DroneProgram && scripts/verify_env.sh
```

**Expect** (this is the reference machine's output; paths show your own home):

```
System
  OK    Ubuntu                     22.04.5 LTS
  OK    python3                    3.10.12 (/usr/bin/python3)
  OK    pip                        22.0.2

ROS 2 and Gazebo
  OK    ros-humble-desktop         0.10.0
  OK    ROS environment            sourced (ROS_DISTRO=humble)
  OK    ros-humble-ros-gzharmonic  0.244.12
  OK    no Fortress ros-gz bridge  ros-humble-ros-gz* absent
  OK    gz sim                     8.11.0
  OK    python3-gz-transport13     13.5.0
  OK    python3-gz-msgs10          10.3.2

PX4 (/home/<you>/PX4-Autopilot)
  OK    PX4 commit                 4a48525e45
  OK    PX4 SITL build             build/px4_sitl_default/bin/px4
  OK    PX4 gz_env.sh              present

Python packages
  OK    pip packages (lock file)   all 77 match
  OK    apt Python packages        all 11 match
  OK    Gazebo Python bindings     gz.transport13, gz.msgs10 import
  OK    OpenCV                     cv2 4.13.0 with aruco
  OK    MAVSDK                     mavsdk_server binary present
  OK    ROS 2 Python               rclpy, sensor_msgs import
  OK    project                    editable install points at this checkout

verify_env: 20 OK, 0 WARN, 0 FAIL
verify_env: environment matches the reference machine
```

A WARN is a version that differs from the reference in a way expected to be
harmless. It does not fail the run. Every FAIL line says what to do.

### 2. The test suite

Run it with the simulator **stopped**. Two dispatch tests fail on purpose if a
live PX4 is up (see [RUN_GUIDE.md](../RUN_GUIDE.md), *Troubleshooting*).

```bash
make check
```

**Expect** the same result as the reference machine. Only the timing may
differ:

```
  lint: clean
  import-check: every module imports from /tmp
  ...
====================== 199 passed, 3 deselected in 40.36s ======================

make check: all gates green
```

### 3. The live system

```bash
source sim/env.sh
world/spawn_fleet.sh
scripts/run_system.sh
python3 scripts/preflight.py
```

**Expect** the last line to be:

```
PREFLIGHT PASS - 10 checks, 0 failures
```

RUN_GUIDE.md, *What success looks like*, shows all ten lines.

### 4. A delivery

Open <http://localhost:5000>, click the map twice (pickup, then drop-off) and
press **Dispatch**. **Expect** the job to reach `COMPLETED`.

### 5. Real landings

```bash
python3 scripts/sitl_landing_trial.py --missions 1
```

**Expect** three touchdowns (pickup, drop-off, home), and:

```
  -> mission COMPLETED, 3 touchdown(s)
```

This step matters. During the September landing rewrite the test suite was
green while the real drone still failed to land (docs/LANDING_REWRITE_LOG.md).
Only a live flight proves landing works. The machine should be otherwise idle:
PX4 runs in lockstep with Gazebo, so competing CPU load slows simulated time
and can look like a hang.

Stop everything with `scripts/stop_system.sh`.

---

## The one thing not proved in advance

The reference machine still carries old local edits inside PX4's
`Tools/simulation/gz` submodule (a camera and LiDAR added to the stock model),
so it has never run this project against **unmodified** PX4. Those edits were
superseded by `sim/models/` in this repository, nothing reads the topics they
created, and every simulation asset loads without them. Even so, the new
machine's first live run is the real test.

`scripts/preflight.py` is built to catch exactly this. If it reports a missing
camera or LiDAR topic:

```bash
echo $GZ_SIM_RESOURCE_PATH    # must include ~/DroneProgram/sim/models
```

If that is wrong, `sim/env.sh` was not sourced in that terminal.

---

## Not migrated, on purpose

| Item | Why it is not needed |
| --- | --- |
| PX4's local edits in `Tools/simulation/gz` | Replaced by `sim/models/` (see `sim/patches/README.md`) |
| `~/ros2_ws` | The old avoider, replaced by `avoider_node.py`; nothing references it |
| `run_drone.sh`, `start_px4.sh`, `kill_drone.sh` in the old home folder | Replaced by `world/spawn_fleet.sh`, `scripts/run_system.sh`, `scripts/stop_system.sh` |
| `vision_env/` | Optional (RUN_GUIDE.md, *Prerequisites*), git-ignored, and adds no isolation |
| `fleet_dispatch/fleet.db` | Runtime state; recreated on first start |
| Python packages for other projects | Not in the lock file; install them separately if you need them |

## Optional: copy by USB

| Item | Why |
| --- | --- |
| `~/DroneProgram/obstacle_memory_service/obstacles.db` (20 KB) | Walls the drones have already learned. Copy to the same path; without it the map starts empty and is relearned |
| `QGroundControl-x86_64.AppImage` (180 MB) | Ground-station UI. The stack does not use it (it talks MAVSDK) |

## Pushing from the new machine

Make a **new** SSH key there rather than copying the old one, add it to GitHub,
and switch the clone from HTTPS to SSH:

```bash
ssh-keygen -t ed25519 -C "<your GitHub username>"
cat ~/.ssh/id_ed25519.pub     # paste at https://github.com/settings/ssh/new
ssh -T git@github.com         # expect: Hi <username>! You've successfully authenticated
git -C ~/DroneProgram remote set-url origin git@github.com:MTahaJamil007/Autonomous_Delivery_Drone.git
```
