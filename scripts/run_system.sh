#!/usr/bin/env bash
# run_system.sh - bring up every support process for a fleet of N drones.
#
#   scripts/run_system.sh          # config.FLEET_SIZE drones
#   scripts/run_system.sh 3        # override
#   scripts/run_system.sh --force  # start even if the pid file looks live
#
# It refuses to start on top of a previous run: a second avoider on one drone is
# finding F7 all over again. Run scripts/stop_system.sh first.
#
# Run world/spawn_fleet.sh FIRST (it starts Gazebo and the PX4 instances), then
# this. Order matters: the sensor topics do not exist until the drones have
# spawned, so a bridge started earlier subscribes to nothing and stays silent.
#
# WHY THIS REPLACES RUN_SYSTEM.sh
# ------------------------------
# The old script opened six gnome-terminal windows with one hardcoded process
# each: ONE vision bridge, ONE ros_gz_bridge on the absolute topic /lidar/scan,
# ONE avoider. That is a single-drone launcher. With FLEET_SIZE=3 all three
# drones published onto the same two topics and one bridge and one avoider tried
# to serve all of them - so every drone reacted to a blend of three LiDARs
# (finding F7).
#
# Here, each drone gets its own ros_gz_bridge (on its own model-scoped topic,
# remapped to a short per-drone ROS name), its own vision bridge, and its own
# avoider on its own UDP port. It also does not require a desktop: everything
# runs headless with logs on disk, which is what makes it usable over SSH and in
# CI.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# shellcheck disable=SC1091
source "$PROJECT_ROOT/sim/env.sh"

FORCE=0
POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        *)       POSITIONAL+=("$arg") ;;
    esac
done
set -- ${POSITIONAL[@]+"${POSITIONAL[@]}"}

FLEET_SIZE="${1:-$(python3 -c 'import config; print(config.FLEET_SIZE)')}"
LOG_DIR="${LOG_DIR:-/tmp/droneprogram}"
PID_FILE="$LOG_DIR/run_system.pids"
mkdir -p "$LOG_DIR"

# ── Refuse to double-start ───────────────────────────────────────────────────
# A second run of this script used to stack a whole extra set of processes on
# top of the live one. The shared services survive it (the second obstacle_memory
# and fleet_dispatch die on EADDRINUSE, which is loud but harmless), but the
# per-drone processes bind no ports: a second vision bridge and a SECOND AVOIDER
# come up alongside the first and both then drive the same drone off the same
# LiDAR. That is finding F7 -- the exact condition this script's header explains
# it was written to prevent -- reintroduced by running it twice.
#
# Worse, the run below overwrites PID_FILE, so stop_system.sh loses every pid
# from the first stack and cannot clean up what it can no longer see.
#
# So: if the recorded pids are still alive, stop and say so. `--force` skips the
# check for the case where the pid file is stale in a way this cannot detect.
live_recorded_processes() {
    [ -f "$PID_FILE" ] || return 0

    local pid name cmdline
    while read -r pid name; do
        [ -n "${pid:-}" ] || continue
        kill -0 "$pid" 2>/dev/null || continue

        # A dead pid can be reused by something unrelated, and refusing to start
        # because of a stranger's pid would be its own bug. Only count it if the
        # command line still looks like one of ours.
        cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
        case "$cmdline" in
            *"$PROJECT_ROOT"* | *obstacle_memory* | *fleet_dispatch* \
            | *perception.vision_bridge* | *avoider_node* \
            | *ros_gz_bridge* | *parameter_bridge*)
                printf '  %-26s pid %s\n' "${name:-?}" "$pid"
                ;;
        esac
    done < "$PID_FILE"
}

RUNNING="$(live_recorded_processes)"
if [ -n "$RUNNING" ] && [ "$FORCE" -eq 0 ]; then
    cat >&2 <<EOF
ERROR: support processes from a previous run are still alive:

$RUNNING

Starting again would run a second avoider and a second vision bridge against
the same drone (finding F7), and would overwrite $PID_FILE so
stop_system.sh could no longer stop the processes above.

  Stop them first:  scripts/stop_system.sh
  Then:             scripts/run_system.sh${1:+ $1}

If you are certain those pids are stale, re-run with --force.
EOF
    exit 1
fi

: > "$PID_FILE"

start() {
    # start <name> <command...>
    local name="$1"; shift
    "$@" >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!
    echo "$pid $name" >> "$PID_FILE"
    printf '  %-26s pid %-7s log %s\n' "$name" "$pid" "$LOG_DIR/$name.log"
}

echo "=============================================="
echo " DroneProgram support processes ($FLEET_SIZE drone(s))"
echo "=============================================="

# ── Shared services ──────────────────────────────────────────────────────────
echo "[1/3] Shared services"
start obstacle_memory python3 -m obstacle_memory_service.app
sleep 1
start fleet_dispatch  python3 -m fleet_dispatch.app
sleep 1

# ── Per-drone sensor consumers ───────────────────────────────────────────────
echo "[2/3] Per-drone bridges and avoiders"
if ! command -v ros2 >/dev/null 2>&1; then
    echo "  WARNING: ros2 is not on PATH. Source your ROS 2 Humble setup first"
    echo "           (e.g. source /opt/ros/humble/setup.bash), or the LiDAR"
    echo "           bridge cannot start and every drone will hold position on a"
    echo "           dead obstacle feed."
fi

for i in $(seq 0 $((FLEET_SIZE - 1))); do
    drone_id="drone-$i"

    # One bridge per drone, on that drone's model-scoped Gazebo topic, remapped
    # to a short ROS name. sim_topics.py owns the derivation so nothing here has
    # to reconstruct the string.
    mapfile -t bridge_args < <(python3 -m sim_topics --drone-id "$drone_id" --what bridge-args | tr ' ' '\n')
    start "ros_gz_bridge_$i" "${bridge_args[@]}"

    start "vision_$i"  python3 -m perception.vision_bridge --drone-id "$drone_id" --headless
    start "avoider_$i" python3 avoider_node.py --drone-id "$drone_id"
done

# ── Verify ───────────────────────────────────────────────────────────────────
echo "[3/3] Preflight"
sleep 4
if python3 scripts/preflight.py; then
    echo
    echo "  System ready. Open http://localhost:$(python3 -c 'import config; print(config.FLEET_DISPATCH_PORT)')"
else
    echo
    echo "  PREFLIGHT FAILED - do not dispatch. The output above names what is"
    echo "  missing. Logs are in $LOG_DIR."
fi

cat <<EOF

Per-drone ports:
$(for i in $(seq 0 $((FLEET_SIZE - 1))); do
    python3 - "$i" <<'PY'
import sys, config
i = int(sys.argv[1]); d = f"drone-{i}"
print(f"  {d}: mavlink {config.mavsdk_port(d)}  vision {config.vision_port(d)}  lidar {config.lidar_port(d)}")
PY
done)

Stop everything:  scripts/stop_system.sh
Tail a log:       tail -f $LOG_DIR/avoider_0.log
EOF
