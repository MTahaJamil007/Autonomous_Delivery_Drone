"""Waypoint navigation with reactive avoidance and planned detours.

Flies world-frame (NED) velocity setpoints by mutating the shared Setpoint that
SetpointPublisher streams. Nothing here talks to the offboard plugin.

WHY NED AND NOT BODY FRAME
--------------------------
Body-frame velocity is relative to where the nose points. When PX4 brakes, the
nose pitches up; a flat-mounted 2D LiDAR then stares at the sky, returns
infinity, and the avoider declares the path clear -- so the drone lunges forward
into the wall it was braking for, brakes again, and ping-pongs until it crashes.
A world-frame command is unaffected by attitude: PX4 works out the tilt itself.

FIXES IN THIS REWRITE
---------------------
P3.6 - Obstacle reports were unreachable. The old code did

           if action != last_action:
               last_action = action          # <-- assigned here
           if action != last_action and action in ("DODGE_LEFT", ...):
               report_obstacle(...)          # <-- so this is never true

       The second condition tested a variable the first had just made equal.
       Every dodge went unreported, so fleet obstacle memory stayed empty and
       "obstacles persisted between missions" could not happen. Fixed by
       capturing the previous action before reassigning.

P3.6 - The report task was fire-and-forget: `asyncio.create_task(...)` with no
       reference kept. An unreferenced task can be garbage-collected before it
       finishes, and its exception is swallowed. References are now held and
       exceptions logged.

P3.7 - Velocity blending was a tick countdown. Replaced with an acceleration
       limit, which is the physically meaningful constraint, matches
       MPC_ACC_HOR, and behaves correctly when the action changes mid-blend --
       a counter restarts and produces a discontinuity.

P5.1 - ESCALATE was a TODO that logged a warning and kept flying at the wall,
       which is the worst available behaviour. It now plans a detour.

P5.2 - Reports are deduplicated within config.OBSTACLE_REPORT_DEDUPE_M and
       placed where the obstacle actually is -- the drone's position projected
       along its heading by the measured clear distance -- rather than at the
       drone, which biased every stored obstacle toward the approach path.

P2.4 - All durations use time.monotonic().
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable
from typing import Any

import config
from drone_agent import geo, udp_receiver
from drone_agent.setpoint import SetpointPublisher

logger = logging.getLogger(__name__)


class NavOutcome:
    """Why a navigation call returned. Strings so they can go straight into logs."""

    ARRIVED = "arrived"
    ESCALATION_EXHAUSTED = "escalation_exhausted"
    SENSOR_LOST = "sensor_lost"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


class ObstacleReporter:
    """Reports discovered obstacles once each, at the right place.

    Per-mission, so the dedupe memory is scoped to one flight: an obstacle
    re-encountered on a later mission should be reported again, because that
    second sighting is what raises its confidence in the shared database.
    """

    def __init__(self, drone_id: str, dedupe_m: float = config.OBSTACLE_REPORT_DEDUPE_M):
        self._drone_id = drone_id
        self._dedupe_m = dedupe_m
        self._reported: list[tuple[float, float]] = []
        # Strong references: an unreferenced task can be collected mid-flight.
        self._tasks: set[asyncio.Task] = set()
        self.reports_sent = 0
        self.reports_suppressed = 0

    def should_report(self, lat: float, lon: float) -> bool:
        """False if we already reported something within the dedupe radius.

        A 30 m wall generates a dodge lasting many seconds. Without this, every
        tick of that dodge would post a new row and one wall would become fifty
        obstacles, each with low confidence, none of them useful to a planner.
        """
        for prev_lat, prev_lon in self._reported:
            if geo.get_distance_m(lat, lon, prev_lat, prev_lon) < self._dedupe_m:
                self.reports_suppressed += 1
                return False
        return True

    def report(
        self,
        drone_lat: float,
        drone_lon: float,
        heading_deg: float,
        clear_distance_m: float,
        radius_m: float = config.OBSTACLE_DEFAULT_RADIUS_M,
    ) -> None:
        """Report the obstacle AHEAD of the drone, not the drone's own position.

        The LiDAR measured `clear_distance_m` of free space along the heading,
        so the obstacle is at least that far away. Storing the drone's own
        coordinates instead -- as the previous code did -- puts every obstacle
        up to SAFE_DIST metres short of its true position, systematically on the
        approach side. A planner using those positions routes around empty air
        and still clips the wall.
        """
        obstacle_lat, obstacle_lon = geo.offset_bearing(
            drone_lat, drone_lon, heading_deg, max(clear_distance_m, 0.5)
        )

        if not self.should_report(obstacle_lat, obstacle_lon):
            return

        self._reported.append((obstacle_lat, obstacle_lon))
        self.reports_sent += 1

        from drone_agent.obstacle_client import report_obstacle

        task = asyncio.create_task(
            report_obstacle(
                lat=obstacle_lat,
                lon=obstacle_lon,
                source_drone=self._drone_id,
                radius_m=radius_m,
                obstacle_type="static_wall",
                confidence=0.5,
            ),
            name=f"report-obstacle-{self.reports_sent}",
        )
        self._tasks.add(task)
        task.add_done_callback(self._on_report_done)

        logger.info(
            "[%s] obstacle reported at (%.6f, %.6f), %.1f m ahead on bearing %.0f",
            self._drone_id,
            obstacle_lat,
            obstacle_lon,
            clear_distance_m,
            heading_deg,
        )

    def _on_report_done(self, task: asyncio.Task) -> None:
        """Log what a report task did instead of letting it vanish silently."""
        self._tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            logger.warning("[%s] obstacle report failed: %s", self._drone_id, exc)

    async def drain(self, timeout_s: float = 3.0) -> None:
        """Wait for in-flight reports at mission end so none are lost on exit."""
        if not self._tasks:
            return
        await asyncio.wait(set(self._tasks), timeout=timeout_s)


def altitude_correction(current_alt_m: float, target_alt_m: float = config.TARGET_ALT_M) -> float:
    """Down-velocity to hold an altitude. NED: negative climbs."""
    error_m = target_alt_m - current_alt_m  # positive: too low
    return geo.clamp(
        -error_m * config.ALT_GAIN,
        -config.ALT_MAX_VEL_M_S,
        config.ALT_MAX_VEL_M_S,
    )


def cruise_speed_for_distance(distance_m: float) -> float:
    """Proportional approach speed: full cruise far out, tapering near the target."""
    return min(
        config.CRUISE_SPEED_M_S,
        max(
            config.MIN_SPEED_M_S,
            config.CRUISE_SPEED_M_S * (distance_m / config.SLOW_RADIUS_M),
        ),
    )


def dodge_velocity(bearing_deg: float, action: str) -> tuple[float, float]:
    """Sideways NED velocity for a dodge, plus a small rearward component.

    The rearward component maintains separation while sliding along the
    obstacle. It is small (config.BACK_SPEED_M_S) because a large one turns a
    dodge into a retreat and the drone never gets past the wall.
    """
    forward_rad = math.radians(bearing_deg)
    side_deg = bearing_deg - 90.0 if action == "DODGE_LEFT" else bearing_deg + 90.0
    side_rad = math.radians(side_deg)

    north = config.DODGE_SPEED_M_S * math.cos(side_rad) - config.BACK_SPEED_M_S * math.cos(
        forward_rad
    )
    east = config.DODGE_SPEED_M_S * math.sin(side_rad) - config.BACK_SPEED_M_S * math.sin(
        forward_rad
    )
    return north, east


async def navigate_to(
    publisher: SetpointPublisher,
    target_lat: float,
    target_lon: float,
    drone_state: dict[str, Any],
    lidar_data: dict[str, Any],
    *,
    drone_id: str,
    reporter: ObstacleReporter | None = None,
    known_obstacles: list[dict[str, Any]] | None = None,
    target_alt_m: float = config.TARGET_ALT_M,
    on_status: Callable[[str], None] | None = None,
    timeout_s: float = config.LEG_TIMEOUT_S,
) -> str:
    """Fly to a target, dodging reactively and detouring when that is not enough.

    Args:
        publisher: The mission's setpoint publisher. This function mutates its
            setpoint; it never starts or stops offboard.
        target_lat, target_lon: Destination.
        drone_state: Live telemetry dict (lat/lon/alt, last_telemetry_ts).
        lidar_data: Live avoider dict (action, eff_front_m, last_lidar_ts).
        reporter: Records newly discovered obstacles. None disables reporting.
        known_obstacles: Obstacles prefetched at mission start, used when a
            detour has to be planned mid-leg.
        target_alt_m: Altitude to hold.
        on_status: Called with a short human-readable status string.
        timeout_s: Give up after this long. Without a ceiling, a leg that
            neither arrives nor fails holds the drone BUSY indefinitely.

    Returns:
        A NavOutcome string.
    """
    waypoints: list[tuple[float, float]] = [(target_lat, target_lon)]
    detour_attempts = 0
    dt_s = 1.0 / config.NAV_HZ
    started_at = time.monotonic()

    previous_action = "CLEAR"
    escalation_handled_for: float | None = None
    warned_stale = False

    def status(text: str) -> None:
        drone_state["status"] = text
        if on_status is not None:
            on_status(text)

    logger.info(
        "[%s] navigating to (%.6f, %.6f) at %.0f m",
        drone_id,
        target_lat,
        target_lon,
        target_alt_m,
    )

    while waypoints:
        leg_lat, leg_lon = waypoints[0]

        while True:
            now = time.monotonic()

            if now - started_at > timeout_s:
                logger.error("[%s] leg timed out after %.0fs without arriving", drone_id, timeout_s)
                publisher.hold()
                return NavOutcome.TIMEOUT

            # ── FAIL CLOSED on a dead obstacle feed ──────────────────────────
            # Hold position; never infer "clear" from silence. This guard was
            # already the intent, but nothing wrote last_lidar_ts, so it was
            # permanently engaged and the drone hovered forever. See
            # drone_agent/udp_receiver.py.
            if udp_receiver.is_stale(lidar_data, udp_receiver.LIDAR_TS_KEY, now=now):
                age = udp_receiver.age_s(lidar_data, udp_receiver.LIDAR_TS_KEY, now=now)
                if not warned_stale:
                    warned_stale = True
                    logger.error(
                        "[%s] obstacle feed silent for %.1fs - holding position "
                        "instead of flying blind. Check avoider_node.py and "
                        "ros_gz_bridge for this drone (see RUN_GUIDE.md).",
                        drone_id,
                        age,
                    )
                status("OBSTACLE FEED DEAD - holding")
                bearing = geo.get_bearing(drone_state["lat"], drone_state["lon"], leg_lat, leg_lon)
                publisher.command_ned(
                    0.0,
                    0.0,
                    altitude_correction(drone_state["alt"], target_alt_m),
                    bearing,
                )
                await asyncio.sleep(dt_s)
                continue

            if warned_stale:
                warned_stale = False
                logger.info("[%s] obstacle feed recovered, resuming", drone_id)

            # ── arrival ──────────────────────────────────────────────────────
            distance_m = geo.get_distance_m(
                drone_state["lat"], drone_state["lon"], leg_lat, leg_lon
            )

            if distance_m <= config.ARRIVAL_M:
                waypoints.pop(0)
                if waypoints:
                    logger.info(
                        "[%s] detour waypoint reached, %d remaining",
                        drone_id,
                        len(waypoints),
                    )
                    break
                # Settle before handing over to the landing controller, so it
                # starts from a stable hover rather than mid-deceleration.
                publisher.hold(
                    geo.get_bearing(drone_state["lat"], drone_state["lon"], leg_lat, leg_lon)
                )
                publisher.setpoint.down_m_s = altitude_correction(drone_state["alt"], target_alt_m)
                await asyncio.sleep(0.5)
                logger.info("[%s] arrived (%.1f m from target)", drone_id, distance_m)
                status("arrived")
                return NavOutcome.ARRIVED

            bearing = geo.get_bearing(drone_state["lat"], drone_state["lon"], leg_lat, leg_lon)
            down_m_s = altitude_correction(drone_state["alt"], target_alt_m)
            action = str(lidar_data.get("action", "CLEAR"))
            eff_front_m = float(lidar_data.get("eff_front_m", config.INF_REPLACE_M))

            # ── report a newly discovered obstacle ───────────────────────────
            # `previous_action` is read BEFORE being reassigned. The old code
            # assigned first and then compared, so this branch was dead.
            entering_dodge = action in ("DODGE_LEFT", "DODGE_RIGHT") and previous_action not in (
                "DODGE_LEFT",
                "DODGE_RIGHT",
            )
            if entering_dodge and reporter is not None:
                reporter.report(drone_state["lat"], drone_state["lon"], bearing, eff_front_m)

            # ── ESCALATE: reactive avoidance has failed, plan around it ──────
            if action == "ESCALATE":
                dodge_start = float(lidar_data.get("dodge_age_s", 0.0))
                # The avoider re-sends ESCALATE every scan (level-triggered, so
                # a dropped datagram cannot lose it), so latch per dodge episode
                # rather than replanning ten times a second.
                episode = now - dodge_start
                already = (
                    escalation_handled_for is not None
                    and abs(episode - escalation_handled_for) < 5.0
                )
                if not already:
                    escalation_handled_for = episode
                    detour_attempts += 1

                    if detour_attempts > config.DETOUR_MAX_ATTEMPTS_PER_LEG:
                        logger.error(
                            "[%s] %d detours attempted on this leg and still "
                            "blocked - holding for the operator rather than "
                            "generating ever longer paths.",
                            drone_id,
                            detour_attempts - 1,
                        )
                        publisher.hold(bearing)
                        status("BLOCKED - detours exhausted")
                        return NavOutcome.ESCALATION_EXHAUSTED

                    new_waypoints = _plan_detour_from_here(
                        drone_state,
                        bearing,
                        eff_front_m,
                        (leg_lat, leg_lon),
                        known_obstacles or [],
                        drone_id,
                    )
                    if new_waypoints:
                        waypoints = new_waypoints + waypoints[1:]
                        status(f"detouring ({detour_attempts})")
                        break  # restart the inner loop on the new waypoint

                    logger.warning(
                        "[%s] no detour found; continuing to dodge reactively",
                        drone_id,
                    )

                # While escalated but unable to plan, keep dodging rather than
                # flying at the obstacle -- which is what the old TODO did.
                target_north, target_east = dodge_velocity(
                    bearing,
                    previous_action if previous_action.startswith("DODGE") else "DODGE_LEFT",
                )
                status("ESCALATED - dodging")

            elif action in ("DODGE_LEFT", "DODGE_RIGHT"):
                target_north, target_east = dodge_velocity(bearing, action)
                status(f"evading ({action})")

            else:
                speed = cruise_speed_for_distance(distance_m)
                target_north, target_east = geo.bearing_to_ned(bearing, speed)
                status(f"cruising, {distance_m:.0f} m to go")

            # ── acceleration-limited command ─────────────────────────────────
            publisher.command_slewed_ned(target_north, target_east, down_m_s, bearing, dt_s)

            previous_action = action
            await asyncio.sleep(dt_s)

    return NavOutcome.ARRIVED


def _plan_detour_from_here(
    drone_state: dict[str, Any],
    bearing_deg: float,
    eff_front_m: float,
    goal: tuple[float, float],
    known_obstacles: list[dict[str, Any]],
    drone_id: str,
) -> list[tuple[float, float]]:
    """Plan around the obstacle the drone is currently stuck on.

    Feeds the planner both the obstacles fetched from fleet memory at mission
    start AND the one live in front of us right now -- the latter matters
    because an obstacle nobody has ever reported is exactly the one that caused
    this escalation, and it would otherwise be invisible to the planner.
    """
    from global_planner.detour import plan_detour

    start = (drone_state["lat"], drone_state["lon"])

    live_lat, live_lon = geo.offset_bearing(start[0], start[1], bearing_deg, max(eff_front_m, 1.0))
    obstacles = [
        *known_obstacles,
        {
            "lat": live_lat,
            "lon": live_lon,
            "radius_m": config.OBSTACLE_DEFAULT_RADIUS_M,
            "obstacle_type": "live_escalation",
        },
    ]

    path = plan_detour(
        start,
        goal,
        obstacles,
        margin_m=config.DETOUR_MARGIN_M,
        max_iterations=config.DETOUR_MAX_ITERATIONS,
    )

    # plan_detour returns [start, goal] when it finds nothing to route around;
    # that is not a detour and following it would fly straight back at the wall.
    if len(path) <= 2:
        return []

    waypoints = path[1:]  # drop `start`, we are already there
    logger.info(
        "[%s] detour planned: %d waypoint(s) around an obstacle %.1f m ahead",
        drone_id,
        len(waypoints),
        eff_front_m,
    )
    return waypoints


async def descend_to(
    publisher: SetpointPublisher,
    drone_state: dict[str, Any],
    target_alt_m: float,
    *,
    drone_id: str,
    tolerance_m: float = 0.5,
    timeout_s: float = 60.0,
) -> bool:
    """Hold position and change altitude. Used to reach the acquisition altitude.

    Descending to config.SEARCH_ALT_M before searching is half of the fix for
    finding F3: a 2 m pad spans 55 px at cruise altitude and 92 px at 6 m, and a
    4x4 ArUco tag needs roughly 25-30 px to decode reliably. The other half is
    the pad size itself.
    """
    dt_s = 1.0 / config.NAV_HZ
    started_at = time.monotonic()
    yaw_deg = publisher.setpoint.yaw_deg

    logger.info(
        "[%s] changing altitude %.1f -> %.1f m",
        drone_id,
        drone_state.get("alt", 0.0),
        target_alt_m,
    )

    while True:
        if time.monotonic() - started_at > timeout_s:
            logger.error(
                "[%s] altitude change to %.1f m timed out at %.1f m",
                drone_id,
                target_alt_m,
                drone_state.get("alt", 0.0),
            )
            publisher.hold(yaw_deg)
            return False

        error_m = target_alt_m - drone_state["alt"]
        if abs(error_m) <= tolerance_m:
            publisher.hold(yaw_deg)
            logger.info("[%s] at %.1f m", drone_id, drone_state["alt"])
            return True

        # Faster than the gentle in-cruise altitude hold: this is a deliberate
        # altitude change, not a correction.
        down_m_s = geo.clamp(-error_m * 0.6, -1.5, 1.5)
        publisher.command_ned(0.0, 0.0, down_m_s, yaw_deg)
        await asyncio.sleep(dt_s)
