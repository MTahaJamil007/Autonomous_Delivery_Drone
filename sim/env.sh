#!/usr/bin/env bash
# sim/env.sh - point Gazebo and PX4 at DroneProgram's vendored simulation assets.
#
# Source this before launching anything that touches the simulator:
#
#     source ~/DroneProgram/sim/env.sh
#
# WHY THIS FILE EXISTS (remediation plan P0.2, finding F2)
# -------------------------------------------------------
# The downward camera and the 360-degree LiDAR used to be a hand edit inside
# PX4-Autopilot/Tools/simulation/gz, which is a git submodule. A single
# `git submodule update --force`, a PX4 rebase, or a fresh clone would have
# deleted both sensors, and the drone would then have armed and flown with
# /camera/image and /lidar/scan simply absent. The sensors now live in
# sim/models/x500_delivery, under this project's version control, and
# scripts/preflight.py refuses to dispatch if their topics are missing.
#
# ORDERING IS LOAD-BEARING
# ------------------------
# PX4's own build/px4_sitl_default/rootfs/gz_env.sh must be sourced FIRST,
# because it sets GZ_SIM_SYSTEM_PLUGIN_PATH and GZ_SIM_SERVER_CONFIG_PATH,
# which the Gazebo server needs to load PX4's plugins, and it puts PX4's stock
# model tree on the resource path -- x500_delivery merge-includes the stock
# `x500`, so that path is required, not optional.
#
# Our directories are then PREPENDED so they win name collisions, and
# PX4_GZ_MODELS / PX4_GZ_WORLDS are overridden last. That override only sticks
# when PX4 is launched with PX4_GZ_STANDALONE=1, because px4-rc.gzsim re-sources
# gz_env.sh (clobbering both) on the non-standalone path. scripts/sim_launch.sh
# therefore always starts the Gazebo server itself and runs PX4 standalone.

# Resolve this file's directory even when sourced.
_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export DRONEPROGRAM_ROOT="$(dirname "$_SIM_DIR")"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "sim/env.sh: PX4-Autopilot not found at $PX4_DIR (set PX4_DIR)" >&2
    return 1 2>/dev/null || exit 1
fi

# 1. PX4's environment: plugins, server config, stock models.
_PX4_GZ_ENV="$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
if [ -f "$_PX4_GZ_ENV" ]; then
    # shellcheck disable=SC1090
    . "$_PX4_GZ_ENV"
else
    echo "sim/env.sh: $_PX4_GZ_ENV not found - run 'make px4_sitl' in $PX4_DIR first." >&2
    echo "sim/env.sh: continuing with best-effort defaults." >&2
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
    export GZ_SIM_SERVER_CONFIG_PATH="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"
fi

# 2. Our assets take precedence.
export GZ_SIM_RESOURCE_PATH="$DRONEPROGRAM_ROOT/sim/models:$DRONEPROGRAM_ROOT/sim/worlds:$GZ_SIM_RESOURCE_PATH"

# 3. Override PX4's spawn/world lookup (effective under PX4_GZ_STANDALONE=1).
export PX4_GZ_MODELS="$DRONEPROGRAM_ROOT/sim/models"
export PX4_GZ_WORLDS="$DRONEPROGRAM_ROOT/sim/worlds"

# 4. Mission defaults. PX4_SYS_AUTOSTART=4001 is the gz_x500 airframe; the
#    airframe file honours PX4_SIM_MODEL, so the same airframe flies our
#    sensor-equipped variant.
export PX4_SYS_AUTOSTART=4001
export PX4_SIM_MODEL=x500_delivery
export PX4_GZ_WORLD=delivery

unset _SIM_DIR _PX4_GZ_ENV
