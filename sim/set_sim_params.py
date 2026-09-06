#!/usr/bin/env python3
"""Set the SITL-only parameters that make a full delivery mission flyable.

WHY THIS EXISTS
===============
PX4's simulated battery drains from 100% to SIM_BAT_MIN_PCT in SIM_BAT_DRAIN
seconds, and SIM_BAT_DRAIN DEFAULTS TO 60. Sixty seconds is shorter than a
single leg of a three-leg delivery, so the low-battery failsafe fires mid-flight
and takes the vehicle to RTL:

    INFO  [tone_alarm] battery warning (fast)
    WARN  [failsafe] Failsafe activated
    INFO  [navigator] RTL: start return at 31 m

That is what happened on the first end-to-end run of the rewritten landing
controller. It is a property of the simulator's battery model, not of the
aircraft and not of the code under test: a real delivery multirotor has twenty
to forty minutes of endurance, so a sixty-second battery makes any multi-leg
mission untestable and makes every such test a test of the failsafe instead.

Nothing here changes flight behaviour. The battery GATE in
drone_agent/battery.py still runs, still reads the same telemetry, and still
refuses a leg it cannot afford -- this only stops the simulated cell from
emptying faster than the vehicle can fly.

Run after world/spawn_fleet.sh, before dispatching. spawn_fleet.sh calls it.
"""

from __future__ import annotations

import asyncio
import sys

from mavsdk import System

ENDURANCE_S = 3000.0
"""Full-charge-to-SIM_BAT_MIN_PCT time, in seconds. 50 minutes, chosen to sit
comfortably beyond the longest three-leg mission plus its search patterns
rather than to model a particular airframe."""


async def _set_one(instance: int) -> bool:
    port = 14540 + instance
    drone = System(port=50100 + instance)
    await drone.connect(system_address=f"udp://0.0.0.0:{port}")

    try:
        await asyncio.wait_for(_wait_connected(drone), timeout=30.0)
    except asyncio.TimeoutError:
        print(f"  drone-{instance}: no MAVLink on {port} after 30 s - skipped")
        return False

    try:
        await drone.param.set_param_float("SIM_BAT_DRAIN", ENDURANCE_S)
    except Exception as exc:  # noqa: BLE001
        print(f"  drone-{instance}: could not set SIM_BAT_DRAIN ({exc})")
        return False

    print(f"  drone-{instance}: SIM_BAT_DRAIN = {ENDURANCE_S:.0f} s")
    return True


async def _wait_connected(drone: System) -> None:
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def main(fleet_size: int) -> int:
    print(f"Setting SITL battery endurance on {fleet_size} instance(s)...")
    results = [await _set_one(i) for i in range(fleet_size)]
    return 0 if all(results) else 1


if __name__ == "__main__":
    size = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    raise SystemExit(asyncio.run(main(size)))
