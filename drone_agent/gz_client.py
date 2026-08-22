"""Asynchronous Gazebo service calls.

WHY ASYNC MATTERS HERE (finding F5)
-----------------------------------
`payload.py` called

    subprocess.run("gz service ...", shell=True)

from an async tick scheduled at config.NAV_HZ. `subprocess.run` is synchronous:
it blocks the entire asyncio event loop -- including the offboard setpoint
publisher -- for as long as the `gz` binary takes to start, connect, transact
and exit. That is tens to hundreds of milliseconds, every tick, at 10 Hz.

PX4 reverts out of OFFBOARD when the setpoint stream gaps for more than about
half a second. So enabling the payload feature exactly as documented would, on
its own, have caused mid-flight mode loss and an uncommanded descent. The same
pattern was in drone_logic's marker spawning.

Every call here uses asyncio.create_subprocess_exec, so the loop keeps running
and the setpoint publisher keeps publishing while `gz` does its work.

WHY exec AND NOT shell
----------------------
The previous calls interpolated model names and float coordinates into a shell
string. Beyond the injection surface, a coordinate formatted in scientific
notation ('1e-05') or a negative number leading with '-' could be reparsed by
the shell as something other than intended. Passing an argument vector removes
the shell from the path entirely.
"""

from __future__ import annotations

import asyncio
import logging
import shutil

import config
from drone_agent import geo

logger = logging.getLogger(__name__)

GZ_BINARY = "gz"
DEFAULT_TIMEOUT_S = 3.0


class GzUnavailable(RuntimeError):
    """The `gz` CLI is not usable, so no simulator control is possible."""


def gz_available() -> bool:
    return shutil.which(GZ_BINARY) is not None


async def _call_service(
    service: str,
    req_type: str,
    rep_type: str,
    request: str,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> bool:
    """Invoke one Gazebo service. Returns True on an apparent success.

    Never raises on a service failure: simulator bookkeeping (a cargo box's
    pose, a pad's spawn) must not be able to end a flight. Failures are logged
    and reported by return value so the caller can decide.
    """
    if not gz_available():
        raise GzUnavailable(
            f"`{GZ_BINARY}` is not on PATH. Source sim/env.sh and ensure Gazebo "
            f"Harmonic is installed."
        )

    argv = [
        GZ_BINARY, "service", "-s", service,
        "--reqtype", req_type,
        "--reptype", rep_type,
        "--timeout", str(int(timeout_s * 1000)),
        "--req", request,
    ]

    try:
        process = await asyncio.create_subprocess_exec(
            *argv,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
    except OSError as exc:
        logger.error("gz service %s: could not start process: %s", service, exc)
        return False

    try:
        stdout, stderr = await asyncio.wait_for(
            process.communicate(), timeout=timeout_s + 2.0
        )
    except asyncio.TimeoutError:
        # Kill rather than leak: an abandoned `gz` process holds a transport
        # connection and, at 10 Hz, would accumulate hundreds of them.
        process.kill()
        await process.wait()
        logger.warning("gz service %s timed out after %.1fs", service, timeout_s)
        return False

    if process.returncode != 0:
        logger.warning(
            "gz service %s failed (rc=%s): %s",
            service, process.returncode,
            (stderr or b"").decode(errors="replace").strip()[:200],
        )
        return False

    # Gazebo replies with `data: true` on success and `data: false` on a
    # refused request, both with exit status 0 -- so the status alone is not
    # enough to tell whether a model actually spawned.
    reply = (stdout or b"").decode(errors="replace")
    if "data: false" in reply:
        logger.warning("gz service %s returned data: false (%s)", service, request[:120])
        return False

    return True


async def spawn_model(
    sdf_uri: str,
    name: str,
    x_east_m: float,
    y_north_m: float,
    z_up_m: float = 0.01,
    yaw_rad: float = 0.0,
    world: str = config.GZ_WORLD,
) -> bool:
    """Spawn a model instance into the running world.

    Args:
        sdf_uri: e.g. 'model://pad_0'. Resolved through GZ_SIM_RESOURCE_PATH, so
            sim/env.sh must have been sourced by whatever launched Gazebo.
        name: Unique entity name. A collision is refused by Gazebo, not merged,
            so callers spawning per-mission pads must namespace by drone.
        x_east_m, y_north_m, z_up_m: World ENU position. Note ENU, not NED:
            Gazebo's z is UP, so a positive z is above ground.
    """
    request = (
        f'sdf_filename: "{sdf_uri}", name: "{name}", '
        f"pose: {{position: {{x: {x_east_m:.4f}, y: {y_north_m:.4f}, z: {z_up_m:.4f}}}, "
        f"orientation: {{z: {(yaw_rad / 2.0):.6f}, w: 1.0}}}}"
    )
    ok = await _call_service(
        f"/world/{world}/create",
        "gz.msgs.EntityFactory", "gz.msgs.Boolean", request,
    )
    if ok:
        logger.info(
            "spawned %s as '%s' at E=%.2f N=%.2f U=%.2f",
            sdf_uri, name, x_east_m, y_north_m, z_up_m,
        )
    return ok


async def set_model_pose(
    name: str,
    x_east_m: float,
    y_north_m: float,
    z_up_m: float,
    world: str = config.GZ_WORLD,
    timeout_s: float = 0.5,
) -> bool:
    """Teleport a model. Used by the payload bay's kinematic following.

    Short timeout by design: this is called at config.PAYLOAD_HZ, and a cargo
    box one frame behind is invisible to the eye, whereas a call that waits
    seconds would queue up behind itself.
    """
    request = (
        f'name: "{name}", '
        f"position: {{x: {x_east_m:.4f}, y: {y_north_m:.4f}, z: {z_up_m:.4f}}}"
    )
    return await _call_service(
        f"/world/{world}/set_pose",
        "gz.msgs.Pose", "gz.msgs.Boolean", request,
        timeout_s=timeout_s,
    )


async def remove_model(name: str, world: str = config.GZ_WORLD) -> bool:
    """Remove a model. Lets a re-dispatch spawn pads under the same names."""
    return await _call_service(
        f"/world/{world}/remove",
        "gz.msgs.Entity", "gz.msgs.Boolean",
        f'name: "{name}", type: MODEL',
    )


async def spawn_pad_at_gps(
    role: str,
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
    name_prefix: str = "",
    world: str = config.GZ_WORLD,
) -> bool:
    """Spawn the correct pad model for a leg role at a GPS position.

    Uses world.marker_models.PAD_MODELS, so pickup gets ArUco ID 0, drop-off
    gets 1 and home gets 2. The previous code spawned `model://arucotag` for all
    three -- a texture that decodes in no dictionary at all (finding F1) -- and
    then asked the detector for ID 0 on every leg.
    """
    import world.marker_models as marker_models

    uri = marker_models.model_uri_for_role(role)
    x_east_m, y_north_m = geo.lat_lon_to_local_enu(lat, lon, origin_lat, origin_lon)
    name = f"{name_prefix}{role}" if name_prefix else role

    # Remove any pad left by an earlier mission: Gazebo refuses a duplicate
    # name rather than replacing it, so without this the second dispatch of a
    # session silently has no pads.
    await remove_model(name, world)

    return await spawn_model(uri, name, x_east_m, y_north_m, z_up_m=0.02, world=world)
