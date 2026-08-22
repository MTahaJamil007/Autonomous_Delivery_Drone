"""The per-drone entry point the dispatcher calls. Thin by design.

WHAT THIS FILE USED TO BE
-------------------------
528 lines that were the code that actually flew, in parallel with the 1,800
lines in drone_agent/ that nothing imported. Two implementations of the same
system, of which the richer one -- with the FSM, marker disambiguation, payload
bay, safety supervisor, battery gating and escalation -- was dead. Every
documented feature lived in the copy that never ran.

The governing decision in the remediation plan was to make drone_agent/ the
mission core and reduce this file to an orchestrator. Concretely, what left:

* THE DUPLICATE CONSTANTS. Lines 41-50 re-declared TARGET_ALT, CRUISE_SPEED,
  SLOW_RADIUS_M, MIN_SPEED, ARRIVAL_M, DODGE_SPEED, BACK_SPEED, ALT_GAIN,
  ALT_MAX_VEL and NAV_HZ at module scope. config.py claimed to centralise
  tuning; editing it changed nothing about what flew. This was the single most
  confusing property of the codebase. There are now zero tunables here --
  `make check` greps for their return.

* THE MODULE-LEVEL STATE. drone_state, vision_data and lidar_data as module
  globals are what actually limited the system to one drone: a second drone in
  the same process would overwrite the first's telemetry. They are now instance
  attributes on DroneMission.

* THE DUPLICATED GEOMETRY. get_distance_m, get_bearing and bearing_to_ned were
  three of the ten copies of those formulas. One copy now lives in
  drone_agent/geo.py.

* THE BLOCKING GAZEBO CALLS. spawn_gazebo_marker used subprocess.run from an
  async context, which stalls the offboard setpoint stream (finding F5), and
  spawned `model://arucotag` for all three pads -- a texture that decodes in
  none of OpenCV's 27 dictionaries (finding F1).

What remains is the contract, and only the contract.
"""

from __future__ import annotations

import logging
from typing import Any

from drone_agent import mission as mission_core
from drone_agent.contracts import DeliveryJob, MissionResult

logger = logging.getLogger(__name__)


async def execute_delivery(job: DeliveryJob, drone_id: str) -> MissionResult:
    """Fly one delivery job on one drone.

    Args:
        job: The delivery request. A dataclass, not four loose floats -- the
            dispatcher used to call this with five positional arguments against
            a four-parameter definition, the TypeError was swallowed by a broad
            `except`, and the job was recorded as COMPLETED. Four same-typed
            floats in a row is a signature no reviewer and no tool can check.
        drone_id: Which drone flies it. Determines the MAVLink port, both UDP
            sensor ports, the gRPC port and the Gazebo model name.

    Returns:
        A MissionResult carrying a terminal status, a human-readable reason and
        the per-leg outcomes. Does not raise for flight failures -- those are
        results, not exceptions -- but does propagate CancelledError so an
        operator abort stays distinguishable from a crash.
    """
    logger.info(
        "dispatch accepted: job %s on %s, pickup (%.6f, %.6f) -> drop (%.6f, %.6f)",
        job.job_id,
        drone_id,
        job.pickup_lat,
        job.pickup_lon,
        job.drop_lat,
        job.drop_lon,
    )
    return await mission_core.DroneMission(drone_id).run(job)


def get_state_snapshot(drone_id: str) -> dict[str, Any]:
    """Live telemetry for a drone, or a zeroed snapshot if it is not flying.

    THIS FUNCTION IS THE ORIGINAL DISPATCH BLOCKER. fleet_dispatch/app.py
    imported it from inside run_drone_task and above that function's `try:`; it
    did not exist, so every dispatch raised ImportError before the try block,
    the `finally: complete_job(...)` never ran, and the drone stayed BUSY
    forever. Three dispatches emptied a three-drone fleet, each reported to the
    operator as successfully assigned.

    Never raises. The dispatcher polls this every two seconds for drones that
    may well be idle, and a telemetry read must not be able to end a flight.
    """
    return mission_core.get_state_snapshot(drone_id)


def active_drones() -> list[str]:
    """Drone ids with a live mission in this process."""
    return mission_core.registered_drones()
