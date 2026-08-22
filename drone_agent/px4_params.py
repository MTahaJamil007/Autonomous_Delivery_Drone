"""Apply PX4 tuning parameters from code, and verify they took.

WHY THIS IS CODE AND NOT A COMMENT
----------------------------------
config.py used to carry seven MPC_* values inside a comment block that asked the
operator to type them into a PX4 shell and run `param save`. That approach has
three problems, in increasing order of severity:

1. A manual step does not survive a rebuild, a reboot, or a new machine.
2. It cannot be reviewed. Nothing in a diff shows whether the tuning changed.
3. Nothing in the logs distinguished "tuned as documented" from "never typed".
   Every flight was therefore of unknown configuration, which makes comparing
   two flights meaningless.

Setting them over MAVSDK at mission start fixes all three. Reading them back
matters as much as setting them: PX4 silently ignores a set for an unknown
parameter name, and clamps values outside a parameter's declared range. A
write-only "apply" would report success for a value the autopilot rejected.
"""

from __future__ import annotations

import asyncio
import logging

import config

logger = logging.getLogger(__name__)

SET_TIMEOUT_S = 5.0


async def apply_params(
    drone,
    params: dict[str, float] | None = None,
    tolerance: float = config.PX4_PARAM_TOLERANCE,
) -> tuple[dict[str, float], list[str]]:
    """Set each parameter and read it back.

    Args:
        drone: Connected MAVSDK System.
        params: name -> value. Defaults to config.PX4_PARAMS.
        tolerance: Absolute difference tolerated between what we wrote and what
            PX4 reports. Not zero: parameters are stored as float32, so a
            float64 like 0.95 does not round-trip bit-exactly.

    Returns:
        (applied, problems) -- the confirmed values, and a human-readable
        complaint per parameter that could not be set or did not read back.

    Never raises. Tuning is an optimisation, not a precondition: refusing to fly
    because MPC_JERK_MAX could not be set would ground the fleet over a comfort
    setting. Problems are returned so the caller can log them prominently.
    """
    if params is None:
        params = config.PX4_PARAMS

    applied: dict[str, float] = {}
    problems: list[str] = []

    for name, value in params.items():
        try:
            await asyncio.wait_for(
                drone.param.set_param_float(name, float(value)), timeout=SET_TIMEOUT_S
            )
        except asyncio.TimeoutError:
            problems.append(f"{name}: set timed out after {SET_TIMEOUT_S}s")
            continue
        except Exception as exc:  # noqa: BLE001 - MAVSDK raises plugin-specific errors
            problems.append(f"{name}: set failed ({type(exc).__name__}: {exc})")
            continue

        try:
            readback = await asyncio.wait_for(
                drone.param.get_param_float(name), timeout=SET_TIMEOUT_S
            )
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{name}: set ok but read-back failed ({exc})")
            continue

        if abs(readback - float(value)) > tolerance:
            # Usually means PX4 clamped the value to the parameter's allowed
            # range -- worth knowing, because the drone will not fly the way
            # config.py claims.
            problems.append(
                f"{name}: wrote {value} but PX4 reports {readback} "
                f"(likely clamped to its allowed range)"
            )
            applied[name] = readback
            continue

        applied[name] = readback

    if problems:
        logger.warning(
            "PX4 tuning applied with %d problem(s): %s",
            len(problems), "; ".join(problems),
        )
    logger.info(
        "PX4 tuning confirmed: %s",
        ", ".join(f"{k}={v:g}" for k, v in sorted(applied.items())) or "none",
    )
    return applied, problems


async def upload_geofence(
    drone,
    centre_lat: float,
    centre_lon: float,
    radius_m: float = config.GEOFENCE_RADIUS_M,
    points: int = 16,
) -> bool:
    """Upload a real circular geofence to the autopilot.

    WHY BOTHER, GIVEN SafetySupervisor ALREADY CHECKS THE RADIUS
    -----------------------------------------------------------
    Because the supervisor is a Python task in the same process as the mission.
    If that process crashes, is killed, or simply stops being scheduled, the
    supervisor stops checking -- and a drone in OFFBOARD with a stale setpoint
    keeps flying. An autopilot-enforced fence survives all of that: PX4 acts on
    a breach whether or not anything on the companion side is still alive.

    The supervisor's check stays as the fast, chatty layer that can react before
    the boundary and put a reason in the log. This is the backstop.

    Approximates the circle as a polygon because MAVLink geofences are
    polygonal. 16 points inscribed in the circle bound the error at about 2%
    of the radius (10 m at 500 m), always inward, which is the safe direction.
    """
    try:
        from mavsdk.geofence import FenceType, GeofenceData, Point, Polygon
    except ImportError as exc:
        logger.error("MAVSDK geofence plugin unavailable: %s", exc)
        return False

    import math

    from drone_agent import geo

    vertices = []
    for i in range(points):
        bearing_deg = 360.0 * i / points
        lat, lon = geo.offset_bearing(centre_lat, centre_lon, bearing_deg, radius_m)
        vertices.append(Point(lat, lon))

    polygon = Polygon(vertices, FenceType.INCLUSION)

    try:
        await asyncio.wait_for(
            drone.geofence.upload_geofence(
                GeofenceData([polygon], [])
            ),
            timeout=10.0,
        )
    except TypeError:
        # MAVSDK's geofence API changed shape across 2.x point releases: older
        # builds take a bare list of polygons. Try that before giving up.
        try:
            await asyncio.wait_for(
                drone.geofence.upload_geofence([polygon]), timeout=10.0
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("geofence upload failed on both API shapes: %s", exc)
            return False
    except Exception as exc:  # noqa: BLE001
        logger.error("geofence upload failed: %s", exc)
        return False

    inscribed_error_m = radius_m * (1.0 - math.cos(math.pi / points))
    logger.info(
        "geofence uploaded: %d-point inclusion polygon, r=%.0f m around "
        "(%.6f, %.6f); polygon sits up to %.1f m inside the circle",
        points, radius_m, centre_lat, centre_lon, inscribed_error_m,
    )
    return True
