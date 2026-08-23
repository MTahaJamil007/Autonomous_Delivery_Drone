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

# 0. Seed the two search paths we are about to extend.
#
# PX4's generated gz_env.sh appends to them with a bare `$GZ_SIM_RESOURCE_PATH`
# and `$GZ_SIM_SYSTEM_PLUGIN_PATH` -- no `:-` default. Under `set -u` that is a
# fatal "unbound variable" on a shell where they are not already exported, so
# any script that sources this file with `set -euo pipefail` died on the first
# clean shell: scripts/run_system.sh aborted before starting a single process,
# and the error named a file inside PX4-Autopilot rather than the missing step.
# It only appeared to work when the operator had already run `source sim/env.sh`
# by hand, which is what the RUN_GUIDE quick start does.
#
# The else-branch below and step 2 expand the same variables, so seeding here
# covers every path through this file.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

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

# 2b. Drop the empty entries the seeding above leaves behind. On a clean shell
#     the seed is "", so composing it produces "ours:...::px4:..." -- and Gazebo
#     reads an empty path entry as the current working directory, which would
#     make model lookup depend on where you happened to be standing.
_strip_empty_entries() {
    printf '%s' "$1" | sed -e 's/::*/:/g' -e 's/^://' -e 's/:$//'
}
GZ_SIM_RESOURCE_PATH="$(_strip_empty_entries "$GZ_SIM_RESOURCE_PATH")"
GZ_SIM_SYSTEM_PLUGIN_PATH="$(_strip_empty_entries "$GZ_SIM_SYSTEM_PLUGIN_PATH")"
export GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
unset -f _strip_empty_entries

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
