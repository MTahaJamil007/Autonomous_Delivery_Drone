"""Precision landing on a specific ArUco pad.

THE THREE REASONS THIS NEVER WORKED, IN THE ORDER THE DRONE MEETS THEM
----------------------------------------------------------------------
F1  The pad was undecodable. drone_logic spawned `model://arucotag`, whose
    texture has no white quiet zone, so ArUco's contour stage cannot segment it
    -- it decodes in ZERO of OpenCV's 27 predefined dictionaries. `locked` never
    became True. Fixed outside this file, in world/marker_models.py.

F3  A 0.5 m marker spans 14 px at 10 m cruise altitude, and a 4x4 tag needs
    25-30 px. The documented flow -- arrive at TARGET_ALT, then search -- could
    not lock even with a decodable pad. Fixed with 2 m pads AND a descent to
    config.SEARCH_ALT_M before searching, because either lever alone is
    fragile.

F4  The controller commanded velocity proportional to PIXEL error. Pixel error
    for a fixed ground offset grows as 1/altitude, so the real loop gain is
    K_p * fx / h:

        h = 10 m -> 0.42 1/s   sluggish but stable
        h =  1 m -> 4.16 1/s   overshoot and oscillation, exactly at touchdown

    Fixed by converting pixels to metres using the current altitude and then
    applying a fixed metric gain, which makes the loop altitude-invariant. This
    also delivers most of what solvePnP was deferred for, with no calibration
    rig -- see docs/CALIBRATION.md.

OTHER FIXES
-----------
* The EMA now filters the MEASURED OFFSET, not the output velocity. Filtering
  the command adds lag to the actuator without removing noise from the sensor.
* Descent is gated by a cone that narrows with altitude. Descending at a fixed
  rate regardless of centring converts a tolerable error at 8 m into a missed
  pad at 0 m.
* Touchdown uses LandedState.ON_GROUND plus disarm, with the altitude threshold
  only as a fallback. And the per-tick `async for pos in drone.telemetry.
  position(): ... break` is gone -- that opened a fresh MAVLink subscription
  every 100 ms.
* The spiral holds altitude (it previously commanded vz=0 with no correction, so
  the drone drifted vertically for the whole pattern), spaces rings by 0.6 of
  the camera footprint so swaths overlap instead of striping, and the timeout is
  capped at config.LANDING_TIMEOUT_MAX_S instead of being computed from a
  100-waypoint path length, which yielded a multi-minute hover.
* No offboard start/stop. Search and descent both go through the mission's one
  setpoint publisher, so the frame never changes mid-loop.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

import config
from drone_agent import geo, udp_receiver
from drone_agent.setpoint import SetpointPublisher

logger = logging.getLogger(__name__)


class LandingOutcome:
    TOUCHDOWN = "touchdown"
    SEARCH_EXHAUSTED = "search_exhausted"
    TIMEOUT = "timeout"
    SENSOR_LOST = "sensor_lost"
    STALLED = "stalled"


# ═════════════════════════════════════════════════════════════════════════════
#  MARKER DISAMBIGUATION
# ═════════════════════════════════════════════════════════════════════════════


def select_target(detections: list[dict], expected_id: int) -> dict | None:
    """The detection matching expected_id, or None.

    Multiple markers in frame are a non-issue -- the filter belongs here, not in
    perception. But this only works if the producer sends every marker it saw:
    the old vision bridge sent only corners[0][0], so this function could never
    see the second pad even when it was plainly in view.
    """
    return next((d for d in detections if d.get("id") == expected_id), None)


# ═════════════════════════════════════════════════════════════════════════════
#  SMOOTHING
# ═════════════════════════════════════════════════════════════════════════════


class EMAFilter:
    """Exponential moving average over a 3-tuple.

    Applied to the MEASURED metric offset. Applying it to the output velocity --
    as the previous landing loop did -- filters the wrong signal: it delays
    every command the controller issues while leaving the measurement noise that
    caused the command fully intact.
    """

    def __init__(self, alpha: float = config.LANDING_EMA_ALPHA):
        """
        Args:
            alpha: Weight of the newest sample. Higher is more responsive and
                less smooth. At 10 Hz, 0.3 gives roughly a 0.3 s time constant.
        """
        self._alpha = alpha
        self._state: tuple[float, float, float] | None = None

    def update(self, sample: tuple[float, float, float]) -> tuple[float, float, float]:
        if self._state is None:
            self._state = sample
        else:
            self._state = tuple(
                self._alpha * new + (1.0 - self._alpha) * old
                # strict=True: a length mismatch means the caller changed the
                # tuple shape, which should fail loudly rather than silently
                # dropping an axis of the offset.
                for new, old in zip(sample, self._state, strict=True)
            )
        return self._state

    def reset(self) -> None:
        """Clear state. Called on lock loss, so a reacquired marker is not
        averaged with where it was several seconds ago."""
        self._state = None


# ═════════════════════════════════════════════════════════════════════════════
#  PIXEL -> METRE CONVERSION (the F4 fix)
# ═════════════════════════════════════════════════════════════════════════════


def pixel_to_ground_offset_m(
    err_px: float, altitude_m: float, fx_px: float = config.CAMERA_FX_PX
) -> float:
    """Convert a pixel error to a ground offset in metres.

    Similar triangles: a marker `offset_m` off the optical axis at height
    `altitude_m` projects to `offset_m * fx / altitude_m` pixels. Inverting:

        offset_m = err_px * altitude_m / fx

    Doing this BEFORE the gain is the whole of the F4 fix. It makes the control
    loop's gain a property of the controller instead of a property of how high
    the drone happens to be.
    """
    if fx_px <= 0.0:
        raise ValueError(f"fx must be positive, got {fx_px}")
    # Below ~0.15 m the projection is degenerate (and we have already touched
    # down); clamping stops a divide-by-tiny producing an enormous offset.
    return err_px * max(altitude_m, 0.15) / fx_px


def metric_offsets(
    detection: dict, altitude_m: float, fx_px: float = config.CAMERA_FX_PX
) -> tuple[float, float]:
    """(forward_m, right_m) body-frame ground offset to the marker.

    SIGNS. Derived in docs/CALIBRATION.md from the camera mount
    (pose 0 0 -0.05 0 1.5708 0, a +90 degree pitch about Y):

        image u (rightwards) maps to body RIGHT
        image v (downwards)  maps to body AFT

    So a marker below-centre in the image (positive err_y) is BEHIND the drone,
    and closing on it requires flying backwards: forward = -err_y.
    A marker right-of-centre (positive err_x) is to the drone's right:
    right = +err_x.

    These signs match what the original code used. They are now derived rather
    than guessed, and confirmed empirically by
    scripts/calibrate_camera_signs.py, because inverting one of them turns the
    controller into a positive feedback loop that flies away from the pad.
    """
    err_x = float(detection.get("err_x", 0.0))
    err_y = float(detection.get("err_y", 0.0))

    forward_m = -pixel_to_ground_offset_m(err_y, altitude_m, fx_px)
    right_m = pixel_to_ground_offset_m(err_x, altitude_m, fx_px)
    return forward_m, right_m


def descent_cone_limit_m(altitude_m: float) -> float:
    """Maximum tolerable offset at this altitude before descent is paused.

    A cone: generous high up, tight near the ground. Below the limit we descend;
    above it we climb slightly and re-centre. The alternative -- the previous
    behaviour of descending at a constant rate whatever the offset -- means a
    2 m error at 8 m altitude is still a 2 m error at touchdown, i.e. a miss.
    """
    return config.DESCENT_CONE_SLOPE * altitude_m + config.DESCENT_CONE_INTERCEPT_M


def descent_rate_m_s(altitude_m: float) -> float:
    """Scheduled descent rate: brisk high up, slow for the last metre."""
    if altitude_m > config.DESCENT_SLOW_ALT_M:
        return config.DESCENT_VZ_HIGH_M_S
    if altitude_m > config.DESCENT_FINAL_ALT_M:
        return config.DESCENT_VZ_MID_M_S
    return config.DESCENT_VZ_FINAL_M_S


# ═════════════════════════════════════════════════════════════════════════════
#  SEARCH PATTERN
# ═════════════════════════════════════════════════════════════════════════════


def camera_footprint_m(altitude_m: float) -> float:
    """Ground width the camera sees at this altitude."""
    return 2.0 * altitude_m * math.tan(config.CAMERA_HFOV_RAD / 2.0)


def generate_spiral_waypoints(
    center_lat: float,
    center_lon: float,
    altitude_m: float = config.SEARCH_ALT_M,
    max_radius_m: float = config.SEARCH_SPIRAL_MAX_RADIUS_M,
    step_m: float | None = None,
) -> list[tuple[float, float]]:
    """Expanding spiral over a GPS point, spaced to overlap the camera swath.

    Ring spacing defaults to 0.6 of the camera footprint at the search altitude
    (config.search_ring_step_m). Fixed 2 m spacing regardless of altitude either
    wasted passes -- at 6 m the camera sees 6.9 m of ground, so 2 m rings
    re-cover the same strip three times -- or, at low altitude, striped and left
    gaps the pad could sit in.
    """
    if step_m is None:
        step_m = config.search_ring_step_m(altitude_m)

    waypoints = [(center_lat, center_lon)]
    ring_count = max(1, int(max_radius_m / step_m))

    for ring in range(1, ring_count + 1):
        radius_m = ring * step_m
        # Points per ring proportional to circumference, so along-track spacing
        # stays roughly constant instead of thinning out on the big rings.
        point_count = max(8, int(2.0 * math.pi * radius_m / step_m))

        for i in range(point_count):
            angle_rad = 2.0 * math.pi * i / point_count
            lat, lon = geo.local_enu_to_lat_lon(
                radius_m * math.cos(angle_rad),
                radius_m * math.sin(angle_rad),
                center_lat,
                center_lon,
            )
            waypoints.append((lat, lon))

    return waypoints


def search_timeout_s(
    waypoints: list[tuple[float, float]],
    speed_m_s: float = config.SEARCH_SPEED_M_S,
) -> float:
    """Time budget for the search, hard-capped.

    The old formula was max(LANDING_TIMEOUT_S, path_length / speed + 15), over a
    spiral with 100+ waypoints at 1 m/s. That evaluated to several minutes of
    hovering by a drone that had already failed to find its pad -- burning the
    battery that the return leg needs. config.LANDING_TIMEOUT_MAX_S caps it.
    """
    path_m = sum(
        geo.get_distance_m(*waypoints[i], *waypoints[i + 1]) for i in range(len(waypoints) - 1)
    )
    estimated_s = path_m / max(speed_m_s, 0.1) + 15.0
    return min(max(config.LANDING_TIMEOUT_S, estimated_s), config.LANDING_TIMEOUT_MAX_S)


# ═════════════════════════════════════════════════════════════════════════════
#  TOUCHDOWN DETECTION
# ═════════════════════════════════════════════════════════════════════════════


def is_on_ground(drone_state: dict[str, Any]) -> bool:
    """True when the autopilot itself says we have landed.

    Prefers PX4's own LandedState over an altitude threshold. LandedState fuses
    altitude, vertical velocity and thrust, so it does not fire while descending
    past 0.3 m and does not miss a touchdown on raised ground. The altitude
    threshold stays as a fallback for the case where LandedState never arrives.
    """
    landed_state = drone_state.get("landed_state")
    if landed_state == "ON_GROUND":
        return True
    if landed_state in ("IN_AIR", "TAKING_OFF", "LANDING"):
        # The autopilot is explicit that we are airborne; do not let a noisy
        # altitude sample override it.
        return False
    return drone_state.get("alt", 99.0) < config.TOUCHDOWN_ALT_M


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN LANDING ROUTINE
# ═════════════════════════════════════════════════════════════════════════════


async def execute_precision_landing(
    publisher: SetpointPublisher,
    drone,
    expected_marker_id: int,
    expected_lat: float,
    expected_lon: float,
    vision_data: dict[str, Any],
    drone_state: dict[str, Any],
    *,
    drone_id: str,
    fsm: Any = None,
    search_alt_m: float = config.SEARCH_ALT_M,
) -> str:
    """Search for a specific pad, centre on it, descend, and land.

    Returns a LandingOutcome string. Fires FSM events marker_locked, lock_lost,
    centered_stable, touchdown and search_exhausted.

    Precondition: the drone is hovering near (expected_lat, expected_lon) with
    the setpoint publisher running. This function never starts or stops
    offboard.
    """
    logger.info(
        "[%s] landing on marker %d; pad spans ~%.0f px at %.0f m (needs >= %.0f px to decode)",
        drone_id,
        expected_marker_id,
        config.marker_px_at_altitude(search_alt_m),
        search_alt_m,
        config.MARKER_MIN_DECODE_PX,
    )

    from drone_agent.navigation import altitude_correction, descend_to

    # ── Descend to the acquisition altitude FIRST (half of the F3 fix) ───────
    if drone_state.get("alt", 0.0) > search_alt_m + 1.0:
        await descend_to(publisher, drone_state, search_alt_m, drone_id=drone_id)

    waypoints = generate_spiral_waypoints(expected_lat, expected_lon, altitude_m=search_alt_m)
    timeout_s = search_timeout_s(waypoints)
    logger.info(
        "[%s] spiral: %d waypoints, %.1f m ring spacing, %.0f s budget",
        drone_id,
        len(waypoints),
        config.search_ring_step_m(search_alt_m),
        timeout_s,
    )

    offset_filter = EMAFilter()
    dt_s = 1.0 / config.NAV_HZ
    started_at = time.monotonic()

    locked = False
    waypoint_index = 0
    descending = False

    # Stall watchdog: catches the case where the marker is held but the drone is
    # not actually coming down (a descent gate that never opens, or a pad on a
    # surface we cannot reach).
    last_progress_t = started_at
    last_progress_alt = drone_state.get("alt", 0.0)

    while True:
        now = time.monotonic()
        elapsed = now - started_at
        altitude_m = drone_state.get("alt", 0.0)
        heading_deg = publisher.setpoint.yaw_deg

        if elapsed > timeout_s:
            logger.error(
                "[%s] marker %d not acquired within %.0f s",
                drone_id,
                expected_marker_id,
                timeout_s,
            )
            publisher.hold(heading_deg)
            publisher.setpoint.down_m_s = altitude_correction(altitude_m, search_alt_m)
            if fsm is not None:
                fsm.fire("search_exhausted")
            return LandingOutcome.TIMEOUT

        # ── FAIL CLOSED on a dead vision feed ───────────────────────────────
        # Descending on a stale detection means descending onto where the pad
        # was, using an offset that is no longer being updated.
        if udp_receiver.is_stale(vision_data, udp_receiver.VISION_TS_KEY, now=now):
            drone_state["status"] = "VISION FEED DEAD - holding"
            if descending:
                logger.error(
                    "[%s] vision feed went silent mid-descent at %.1f m - "
                    "stopping the descent and holding.",
                    drone_id,
                    altitude_m,
                )
                descending = False
            publisher.hold(heading_deg)
            publisher.setpoint.down_m_s = altitude_correction(altitude_m, search_alt_m)
            await asyncio.sleep(dt_s)
            continue

        detections = vision_data.get("detections") or []
        target = select_target(detections, expected_marker_id)

        # ── Marker in view ──────────────────────────────────────────────────
        if target is not None:
            if not locked:
                locked = True
                logger.info(
                    "[%s] marker %d locked at %.1f m (%.0f px wide)",
                    drone_id,
                    expected_marker_id,
                    altitude_m,
                    target.get("size_px", 0.0),
                )
                if fsm is not None:
                    fsm.fire("marker_locked")

            # Filter the MEASUREMENT, then apply a fixed metric gain.
            raw_forward_m, raw_right_m = metric_offsets(target, altitude_m)
            forward_m, right_m, _ = offset_filter.update((raw_forward_m, raw_right_m, 0.0))
            offset_m = math.hypot(forward_m, right_m)

            # Deadband: below this, commanding a correction just chases noise.
            if offset_m < config.LANDING_DEADBAND_M:
                forward_cmd = right_cmd = 0.0
            else:
                forward_cmd = geo.clamp(
                    config.LANDING_K_P * forward_m,
                    -config.LANDING_MAX_VEL_M_S,
                    config.LANDING_MAX_VEL_M_S,
                )
                right_cmd = geo.clamp(
                    config.LANDING_K_P * right_m,
                    -config.LANDING_MAX_VEL_M_S,
                    config.LANDING_MAX_VEL_M_S,
                )

            centred = offset_m < config.CENTERED_M
            within_cone = offset_m < descent_cone_limit_m(altitude_m)

            if centred and not descending and fsm is not None:
                if fsm.state.value == "APPROACH":
                    fsm.fire("centered_stable")

            # ── Descent gate ────────────────────────────────────────────────
            if within_cone:
                if not descending:
                    logger.info(
                        "[%s] inside the descent cone (%.2f m < %.2f m at %.1f m) - descending",
                        drone_id,
                        offset_m,
                        descent_cone_limit_m(altitude_m),
                        altitude_m,
                    )
                    descending = True
                down_m_s = descent_rate_m_s(altitude_m)
            else:
                if descending:
                    logger.warning(
                        "[%s] drifted outside the cone (%.2f m > %.2f m at %.1f m) "
                        "- climbing to re-centre",
                        drone_id,
                        offset_m,
                        descent_cone_limit_m(altitude_m),
                        altitude_m,
                    )
                    descending = False
                down_m_s = -config.CLIMB_RECENTER_VZ_M_S

            publisher.command_body_horizontal(forward_cmd, right_cmd, down_m_s, heading_deg)
            drone_state["status"] = (
                f"landing on {expected_marker_id}: {offset_m:.2f} m off, {altitude_m:.1f} m up"
            )

            # ── Touchdown ───────────────────────────────────────────────────
            if is_on_ground(drone_state):
                logger.info("[%s] touchdown at %.2f m", drone_id, altitude_m)
                publisher.hold(heading_deg)
                if await _confirm_landed(drone, drone_id):
                    if fsm is not None:
                        fsm.fire("touchdown")
                    return LandingOutcome.TOUCHDOWN
                # Not actually down: keep going rather than declaring success.
                logger.warning(
                    "[%s] touchdown signalled but the drone did not disarm - "
                    "continuing the descent",
                    drone_id,
                )

            # ── Stall watchdog ──────────────────────────────────────────────
            if now - last_progress_t >= 5.0:
                descended_m = last_progress_alt - altitude_m
                if descending and descended_m < 0.3:
                    logger.error(
                        "[%s] marker held but only %.2f m descended in 5 s at "
                        "%.1f m - aborting this attempt rather than hovering "
                        "until the timeout.",
                        drone_id,
                        descended_m,
                        altitude_m,
                    )
                    publisher.hold(heading_deg)
                    if fsm is not None:
                        fsm.fire("search_exhausted")
                    return LandingOutcome.STALLED
                last_progress_t = now
                last_progress_alt = altitude_m

        # ── Marker not in view: search ──────────────────────────────────────
        else:
            if locked:
                locked = False
                descending = False
                offset_filter.reset()
                logger.warning(
                    "[%s] lost marker %d at %.1f m - resuming search",
                    drone_id,
                    expected_marker_id,
                    altitude_m,
                )
                if fsm is not None:
                    fsm.fire("lock_lost")

            if waypoint_index >= len(waypoints):
                logger.error(
                    "[%s] spiral exhausted (%d waypoints) without seeing marker %d",
                    drone_id,
                    len(waypoints),
                    expected_marker_id,
                )
                publisher.hold(heading_deg)
                publisher.setpoint.down_m_s = altitude_correction(altitude_m, search_alt_m)
                if fsm is not None:
                    fsm.fire("search_exhausted")
                return LandingOutcome.SEARCH_EXHAUSTED

            wp_lat, wp_lon = waypoints[waypoint_index]
            distance_m = geo.get_distance_m(drone_state["lat"], drone_state["lon"], wp_lat, wp_lon)

            if distance_m < 1.0:
                waypoint_index += 1
                if waypoint_index % 10 == 0:
                    logger.info(
                        "[%s] search waypoint %d/%d",
                        drone_id,
                        waypoint_index,
                        len(waypoints),
                    )
            else:
                bearing = geo.get_bearing(drone_state["lat"], drone_state["lon"], wp_lat, wp_lon)
                north, east = geo.bearing_to_ned(bearing, config.SEARCH_SPEED_M_S)
                # ALTITUDE IS HELD. The old search sent vz=0 with no correction,
                # so the drone drifted vertically for the entire pattern -- and
                # marker decodability depends directly on altitude.
                publisher.command_ned(
                    north,
                    east,
                    altitude_correction(altitude_m, search_alt_m),
                    bearing,
                )
                drone_state["status"] = (
                    f"searching for {expected_marker_id}: "
                    f"waypoint {waypoint_index}/{len(waypoints)}"
                )

        await asyncio.sleep(dt_s)


async def _confirm_landed(drone, drone_id: str, timeout_s: float = 12.0) -> bool:
    """Issue land, then wait for the autopilot to confirm disarm.

    Waiting for `armed == False` rather than assuming the land command worked is
    what makes touchdown a fact instead of an intention. Bounded, because an
    unbounded `async for armed in drone.telemetry.armed()` -- the previous
    behaviour -- hangs the mission forever if the drone never disarms.
    """
    try:
        await asyncio.wait_for(drone.action.land(), timeout=5.0)
    except Exception as exc:  # noqa: BLE001
        logger.warning("[%s] land command failed: %s", drone_id, exc)

    async def _wait_disarmed() -> bool:
        async for armed in drone.telemetry.armed():
            if not armed:
                return True
        return False

    try:
        disarmed = await asyncio.wait_for(_wait_disarmed(), timeout=timeout_s)
    except asyncio.TimeoutError:
        logger.warning("[%s] still armed %.0f s after the land command", drone_id, timeout_s)
        return False

    if disarmed:
        logger.info("[%s] disarmed - landing confirmed", drone_id)
    return disarmed
