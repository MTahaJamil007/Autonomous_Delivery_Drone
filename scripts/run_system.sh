#!/usr/bin/env bash
# run_system.sh - bring up every support process for a fleet of N drones.
#
#   scripts/run_system.sh          # config.FLEET_SIZE drones
#   scripts/run_system.sh 3        # override
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

FLEET_SIZE="${1:-$(python3 -c 'import config; print(config.FLEET_SIZE)')}"
LOG_DIR="${LOG_DIR:-/tmp/droneprogram}"
PID_FILE="$LOG_DIR/run_system.pids"
mkdir -p "$LOG_DIR"
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
