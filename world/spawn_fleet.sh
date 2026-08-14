#!/bin/bash
# spawn_fleet.sh - Launch multi-vehicle PX4 SITL fleet
# Each drone gets a unique instance ID, MAVSDK port, and spawn position

set -e

# Source configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# Import fleet size from Python config
FLEET_SIZE=$(python3 -c "import sys; sys.path.insert(0, '$PARENT_DIR'); import config; print(config.FLEET_SIZE)")
MAVSDK_PORT_BASE=$(python3 -c "import sys; sys.path.insert(0, '$PARENT_DIR'); import config; print(config.MAVSDK_PORT_BASE)")

# PX4 directory (adjust if needed)
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ PX4-Autopilot not found at $PX4_DIR"
    echo "   Set PX4_DIR environment variable or install PX4-Autopilot"
    exit 1
fi

echo "🚁 Starting $FLEET_SIZE drone fleet..."
echo "   MAVSDK ports: $MAVSDK_PORT_BASE - $((MAVSDK_PORT_BASE + FLEET_SIZE - 1))"

# Spawn offset (5 meters apart in X direction to avoid collisions)
SPAWN_OFFSET=5.0

# Launch each drone instance
for i in $(seq 0 $((FLEET_SIZE - 1))); do
    INSTANCE_ID=$i
    MAVSDK_PORT=$((MAVSDK_PORT_BASE + i))
    
    # Calculate spawn position (offset along X axis)
    X_OFFSET=$(python3 -c "print($i * $SPAWN_OFFSET)")
    
    echo ""
    echo "🚁 Launching drone-$i:"
    echo "   Instance: $INSTANCE_ID"
    echo "   MAVSDK Port: udpin://0.0.0.0:$MAVSDK_PORT"
    echo "   Spawn Position: X=$X_OFFSET, Y=0, Z=0"
    
    # Launch PX4 SITL instance with Gazebo
    # Each instance needs:
    # - Unique instance ID (-i flag)
    # - Unique spawn position (PX4_GZ_MODEL_POSE)
    # - Unique model name to avoid Gazebo entity conflicts
    
    cd "$PX4_DIR"
    
    # Set environment variables for this instance
    export PX4_SIM_MODEL="gz_x500"
    export PX4_GZ_MODEL_NAME="drone_${i}"
    export PX4_GZ_MODEL_POSE="${X_OFFSET},0,0,0,0,0"
    
    # Launch in background
    # Note: Output is redirected to prevent terminal clutter
    # Check logs in PX4-Autopilot/build/px4_sitl_default/instance_${i}/
    make px4_sitl gz_x500 -j1 PX4_SYS_AUTOSTART=4001 \
        PX4_SIM_MODEL=gz_x500 \
        PX4_INSTANCE=$INSTANCE_ID \
        > "/tmp/px4_drone_${i}.log" 2>&1 &
    
    PID=$!
    echo "   PID: $PID (logs: /tmp/px4_drone_${i}.log)"
    
    # Brief delay between launches to avoid resource contention
    sleep 2
done

echo ""
echo "✅ Fleet launch initiated!"
echo ""
echo "📋 Next steps:"
echo "   1. Wait ~30 seconds for all instances to initialize"
echo "   2. Check Gazebo window - you should see $FLEET_SIZE drones"
echo "   3. Verify MAVSDK connectivity on ports $MAVSDK_PORT_BASE-$((MAVSDK_PORT_BASE + FLEET_SIZE - 1))"
echo ""
echo "🛑 To stop all drones: pkill -f 'px4.*sitl'"
