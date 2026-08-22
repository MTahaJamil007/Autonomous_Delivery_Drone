#!/usr/bin/env bash
# stop_system.sh - stop everything run_system.sh and spawn_fleet.sh started.
#
# Kills by recorded PID first, then sweeps by pattern for anything started by
# hand or left over from a crash. The old teardown was a bare list of
# `pkill -f` patterns, which also matched an editor with the file open.

set -uo pipefail

LOG_DIR="${LOG_DIR:-/tmp/droneprogram}"
PID_FILE="$LOG_DIR/run_system.pids"

echo "Stopping DroneProgram..."

if [ -f "$PID_FILE" ]; then
    while read -r pid name; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null && echo "  stopped $name (pid $pid)"
        fi
    done < "$PID_FILE"
    sleep 2
    # Escalate only for what ignored SIGTERM.
    while read -r pid name; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null && echo "  killed $name (pid $pid)"
        fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
fi

# Sweep. Patterns are specific enough not to match an editor or a grep.
for pattern in \
    'perception.vision_bridge' \
    'avoider_node.py' \
    'ros_gz_bridge' \
    'fleet_dispatch.app' \
    'obstacle_memory_service.app' \
    'mavsdk_server' \
    'px4_sitl_default/bin/px4' \
    'gz sim'
do
    if pkill -f "$pattern" 2>/dev/null; then
        echo "  swept: $pattern"
    fi
done

echo "Done. Logs remain in $LOG_DIR."
