#!/usr/bin/env bash
# spawn_fleet.sh - launch one Gazebo server and N PX4 SITL instances against it.
#
#   world/spawn_fleet.sh            # config.FLEET_SIZE drones
#   world/spawn_fleet.sh 3          # override the count
#   HEADLESS=1 world/spawn_fleet.sh # no Gazebo GUI
#
# WHY THIS IS A REWRITE, NOT A PATCH
# ----------------------------------
# The previous version called `make px4_sitl gz_x500` inside the per-drone loop.
# That serialised N builds and had them race on one build directory, so a fleet
# launch was slow, non-deterministic, and frequently produced a half-built
# binary. It also exported PX4_GZ_MODEL_NAME, which in this PX4 version does the
# opposite of what was intended: setting it makes PX4 SKIP spawning a model and
# instead attach to a pre-existing one (see px4-rc.gzsim), so the drones never
# appeared.
#
# The recipe here is: build once, start one Gazebo server ourselves, then launch
# each PX4 instance standalone against it.
#
# WHY WE START GAZEBO OURSELVES
# -----------------------------
# PX4's px4-rc.gzsim re-sources its own gz_env.sh on the non-standalone path,
# which unconditionally overwrites PX4_GZ_WORLDS and PX4_GZ_MODELS back to the
# PX4 source tree. Our vendored world and sensor-equipped model would be ignored
# and the drone would fly blind. Under PX4_GZ_STANDALONE=1, gz_env.sh is never
# sourced, so the values sim/env.sh exports survive. That is the whole reason
# for this ordering, and it is why the last drone is NOT launched non-standalone
# "for convenience".
#
# INSTANCE NAMING: PX4 spawns each drone as "${PX4_SIM_MODEL}_${instance}",
# e.g. x500_delivery_0. sim_topics.py derives the sensor topic names from that,
# which is how each drone gets its own camera and LiDAR topic (finding F7).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# shellcheck disable=SC1091
source "$PROJECT_ROOT/sim/env.sh"

FLEET_SIZE="${1:-$(python3 -c 'import config; print(config.FLEET_SIZE)')}"
MAX_FLEET_SIZE=$(python3 -c 'import config; print(config.MAX_FLEET_SIZE)')

if [ "$FLEET_SIZE" -lt 1 ]; then
    echo "spawn_fleet: FLEET_SIZE must be >= 1" >&2
    exit 1
fi

if [ "$FLEET_SIZE" -gt "$MAX_FLEET_SIZE" ]; then
    # PX4's px4-rc.mavlink collapses every instance above 9 onto port 14549, so
    # instances 10+ would share a MAVLink port and fight over it.
    echo "spawn_fleet: FLEET_SIZE $FLEET_SIZE exceeds the hard limit of $MAX_FLEET_SIZE." >&2
    echo "             PX4 maps instances >9 onto the same MAVLink port (14549)." >&2
    exit 1
fi

WORLD_FILE="$PX4_GZ_WORLDS/${PX4_GZ_WORLD}.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "spawn_fleet: world not found at $WORLD_FILE" >&2
    exit 1
fi

MODEL_DIR="$PX4_GZ_MODELS/$PX4_SIM_MODEL"
if [ ! -f "$MODEL_DIR/model.sdf" ]; then
    echo "spawn_fleet: model not found at $MODEL_DIR/model.sdf" >&2
    exit 1
fi

LOG_DIR="${LOG_DIR:-/tmp/droneprogram}"
mkdir -p "$LOG_DIR"

echo "=============================================="
echo " DroneProgram fleet launch"
echo "=============================================="
echo " drones      : $FLEET_SIZE"
echo " world       : $WORLD_FILE"
echo " model       : $PX4_SIM_MODEL"
echo " MAVLink     : 14540 .. $((14540 + FLEET_SIZE - 1))"
echo " logs        : $LOG_DIR"
echo

# ── 1. Build once ────────────────────────────────────────────────────────────
# Outside the loop, deliberately. N concurrent makes racing on one build
# directory was the previous script's central defect.
echo "[1/3] Building PX4 SITL (once)..."
make -C "$PX4_DIR" px4_sitl >"$LOG_DIR/px4_build.log" 2>&1 || {
    echo "spawn_fleet: PX4 build failed. See $LOG_DIR/px4_build.log" >&2
    tail -20 "$LOG_DIR/px4_build.log" >&2
    exit 1
}
echo "      build ok"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [ ! -x "$PX4_BIN" ]; then
    echo "spawn_fleet: $PX4_BIN missing after a successful build" >&2
    exit 1
fi

# ── 2. One Gazebo server for the whole fleet ─────────────────────────────────
echo "[2/3] Starting Gazebo server..."
if gz topic -l 2>/dev/null | grep -q "^/world/${PX4_GZ_WORLD}/clock$"; then
    echo "      world '${PX4_GZ_WORLD}' already running, reusing it"
else
    gz sim --verbose=1 -r -s "$WORLD_FILE" >"$LOG_DIR/gz_server.log" 2>&1 &
    GZ_PID=$!
    echo "      server pid $GZ_PID (log: $LOG_DIR/gz_server.log)"

    if [ -z "${HEADLESS:-}" ]; then
        gz sim -g >"$LOG_DIR/gz_gui.log" 2>&1 &
        echo "      gui pid $!"
    fi

    # Wait for the scene service rather than sleeping a guessed interval.
    echo -n "      waiting for world to come up"
    for _ in $(seq 30); do
        if gz service -i --service "/world/${PX4_GZ_WORLD}/scene/info" 2>&1 \
             | grep -q "Service providers"; then
            echo " ready"
            break
        fi
        echo -n "."
        sleep 1
    done
fi

# ── 3. One PX4 instance per drone, all standalone ────────────────────────────
echo "[3/3] Launching PX4 instances..."
SPAWN_SPACING_M="${SPAWN_SPACING_M:-5.0}"

for i in $(seq 0 $((FLEET_SIZE - 1))); do
    x_offset=$(python3 -c "print($i * $SPAWN_SPACING_M)")
    instance_dir="$PX4_DIR/build/px4_sitl_default/instance_$i"
    mkdir -p "$instance_dir"

    (
        cd "$instance_dir"
        PX4_GZ_STANDALONE=1 \
        PX4_SYS_AUTOSTART="$PX4_SYS_AUTOSTART" \
        PX4_SIM_MODEL="$PX4_SIM_MODEL" \
        PX4_GZ_WORLD="$PX4_GZ_WORLD" \
        PX4_GZ_MODELS="$PX4_GZ_MODELS" \
        PX4_GZ_MODEL_POSE="${x_offset},0,0,0,0,0" \
        GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
        exec "$PX4_BIN" -i "$i" -d "$PX4_DIR/build/px4_sitl_default/etc"
    ) >"$LOG_DIR/px4_$i.log" 2>&1 &

    echo "      drone-$i: instance $i, MAVLink udpin://0.0.0.0:$((14540 + i)), " \
         "model ${PX4_SIM_MODEL}_$i at x=${x_offset}m (log: $LOG_DIR/px4_$i.log)"

    # Stagger so instances do not contend on model spawning.
    sleep 3
done

echo
echo "=============================================="
echo " Fleet launched. Next:"
echo "   1. python3 scripts/preflight.py"
echo "      -> must PASS before dispatching anything"
echo "   2. scripts/run_system.sh   (services, bridges, avoiders, vision)"
echo
echo " Per-drone sensor topics:"
for i in $(seq 0 $((FLEET_SIZE - 1))); do
    echo "   drone-$i:"
    python3 -m sim_topics --drone-id "drone-$i" | sed 's/^/     /'
done
echo
echo " Stop everything: scripts/stop_system.sh"
echo "=============================================="
