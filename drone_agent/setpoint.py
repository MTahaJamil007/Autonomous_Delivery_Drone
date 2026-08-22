"""The offboard setpoint publisher: one task, one stream, no gaps.

THE INVARIANT
-------------
PX4 leaves OFFBOARD mode if the setpoint stream gaps for more than roughly half
a second. Everything else in this file exists to make "the stream never gaps" a
structural property of the system rather than something every code path has to
remember.

One task publishes whatever is currently in `self._setpoint` at
config.SETPOINT_HZ, unconditionally, for as long as the mission runs. Mission
code never talks to the offboard plugin: it mutates the setpoint object and
returns. Navigation, landing, search and hold are all just different values in
that object.

WHAT THIS REPLACES, AND WHY IT WAS A PROBLEM
--------------------------------------------
Before, every phase of flight drove offboard itself:

* `navigate_with_avoidance` called offboard.start() at the top and
  offboard.stop() on arrival. `execute_precision_landing` called start() again.
  Three legs meant six start/stop pairs per mission, and the V5 docstring in
  drone_logic.py itself warned that "PX4 cannot handle rapid mode thrashing".
  The code warned about the thing the code did.

* landing.py mixed frames inside one offboard session: set_velocity_body() for
  the descent and set_velocity_ned() for the spiral search, alternating at
  10 Hz. PX4 tracks which setpoint type a session is streaming; alternating
  types is not a supported way to fly.

* Every `await drone.offboard.set_velocity_*()` was on the mission's critical
  path, so any slow await anywhere in a tick -- a blocking subprocess, an HTTP
  post, a telemetry subscription -- delayed the next setpoint. That is finding
  F5: the mission code was the thing gapping the stream it depended on.

With a dedicated publisher, a mission tick that takes 400 ms is a late
*decision*, not a dropped setpoint. The drone keeps flying its last command
smoothly instead of falling out of OFFBOARD.

ONE FRAME ONLY
--------------
This publisher streams VelocityNedYaw and nothing else. NED is the world frame,
so a nose that pitches up while braking does not rotate the commanded velocity
-- which is what previously made a flat-mounted LiDAR stare at the sky, report
infinity, and trigger a lunge forward into the wall it had just braked for.
Landing wants body-relative motion, but the camera is fixed to the airframe and
the yaw is held constant during descent, so a body-frame offset is converted to
NED once, at the call site, using the current heading.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass

import config
from drone_agent import geo

logger = logging.getLogger(__name__)


@dataclass
class Setpoint:
    """The commanded velocity. Mutated by mission code, read by the publisher.

    NED: north/east positive as named, and DOWN POSITIVE -- so a negative
    `down_m_s` climbs. That sign catches everyone once; it is PX4's convention.
    """

    north_m_s: float = 0.0
    east_m_s: float = 0.0
    down_m_s: float = 0.0
    yaw_deg: float = 0.0

    def hold(self, yaw_deg: float | None = None) -> None:
        """Stop translating but keep streaming. NOT the same as stopping offboard.

        This is what a fail-closed sensor guard commands: hold position, stay in
        OFFBOARD, keep the stream alive so the drone remains under our control
        and can resume the instant the feed returns.
        """
        self.north_m_s = 0.0
        self.east_m_s = 0.0
        self.down_m_s = 0.0
        if yaw_deg is not None:
            self.yaw_deg = yaw_deg


class SetpointPublisher:
    """Streams one drone's offboard setpoints at a fixed rate.

    Usage:
        publisher = SetpointPublisher(drone, drone_id)
        await publisher.start()             # primes, then enters OFFBOARD
        publisher.setpoint.north_m_s = 2.0  # mission code does only this
        ...
        await publisher.stop()              # on mission end only
    """

    def __init__(self, drone, drone_id: str, hz: float = config.SETPOINT_HZ):
        self._drone = drone
        self._drone_id = drone_id
        self._period_s = 1.0 / hz
        self._hz = hz

        self.setpoint = Setpoint()

        self._task: asyncio.Task | None = None
        self._offboard_active = False
        self._publishes = 0
        self._send_failures = 0
        self._max_gap_s = 0.0
        self._last_publish_t = 0.0

    # ── lifecycle ────────────────────────────────────────────────────────────

    async def start(self) -> None:
        """Prime the stream, enter OFFBOARD, and keep publishing until stopped.

        PX4 requires setpoints to be arriving BEFORE offboard.start(), or it
        rejects the mode change outright. Priming first is not belt-and-braces;
        it is the documented handshake.
        """
        from mavsdk.offboard import OffboardError, VelocityNedYaw

        if self._task is not None:
            logger.debug("[%s] setpoint publisher already running", self._drone_id)
            return

        zero = VelocityNedYaw(0.0, 0.0, 0.0, self.setpoint.yaw_deg)
        for _ in range(max(3, int(self._hz * 0.25))):
            await self._drone.offboard.set_velocity_ned(zero)
            await asyncio.sleep(self._period_s)

        try:
            await self._drone.offboard.start()
            self._offboard_active = True
            logger.info("[%s] OFFBOARD engaged, streaming at %.0f Hz", self._drone_id, self._hz)
        except OffboardError as exc:
            # Report and keep publishing: PX4 sometimes reports a failure for a
            # mode it did in fact enter, and a stream with no consumer is
            # harmless whereas a missing stream is not.
            logger.error(
                "[%s] offboard.start() rejected: %s. Continuing to stream; if "
                "the drone does not respond, check the PX4 console for a "
                "rejected-mode message.",
                self._drone_id,
                exc._result.result_str,
            )

        self._task = asyncio.create_task(self._run(), name=f"setpoint-{self._drone_id}")

    async def stop(self) -> None:
        """Stop publishing and leave OFFBOARD. Call once, at mission end.

        Deliberately NOT called between legs. Leaving and re-entering OFFBOARD
        per leg is the mode thrashing this class exists to eliminate.
        """
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        if self._offboard_active:
            try:
                await asyncio.wait_for(self._drone.offboard.stop(), timeout=3.0)
            except Exception as exc:  # noqa: BLE001 - a failed stop must not mask a result
                logger.warning("[%s] offboard.stop() failed: %s", self._drone_id, exc)
            self._offboard_active = False

        logger.info(
            "[%s] setpoint publisher stopped: %d publishes, %d send failures, "
            "worst gap %.3fs (PX4 tolerates ~0.5s)",
            self._drone_id,
            self._publishes,
            self._send_failures,
            self._max_gap_s,
        )

    # ── the loop ─────────────────────────────────────────────────────────────

    async def _run(self) -> None:
        """Publish the current setpoint forever. The one place that sends.

        Uses an absolute schedule rather than sleep(period): sleeping a fixed
        interval accumulates the send duration as drift, so an intended 20 Hz
        becomes 17 Hz and then 15 Hz over a long leg.
        """
        from mavsdk.offboard import VelocityNedYaw

        next_tick = time.monotonic()
        self._last_publish_t = next_tick

        try:
            while True:
                now = time.monotonic()
                gap = now - self._last_publish_t
                if gap > self._max_gap_s:
                    self._max_gap_s = gap
                    if gap > 0.4:
                        # 0.4s is close enough to PX4's tolerance to be worth
                        # shouting about while there is still time to react.
                        logger.error(
                            "[%s] setpoint gap %.3fs - approaching the OFFBOARD "
                            "timeout. Something is blocking the event loop.",
                            self._drone_id,
                            gap,
                        )

                setpoint = self.setpoint
                try:
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(
                            setpoint.north_m_s,
                            setpoint.east_m_s,
                            setpoint.down_m_s,
                            setpoint.yaw_deg,
                        )
                    )
                    self._publishes += 1
                    self._last_publish_t = now
                except Exception as exc:  # noqa: BLE001
                    # Keep the loop alive: one failed send is survivable, a dead
                    # publisher is not.
                    self._send_failures += 1
                    if self._send_failures <= 3 or self._send_failures % 100 == 0:
                        logger.warning(
                            "[%s] setpoint send failed (#%d): %s",
                            self._drone_id,
                            self._send_failures,
                            exc,
                        )

                next_tick += self._period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    await asyncio.sleep(sleep_s)
                else:
                    # Behind schedule. Re-base rather than trying to catch up,
                    # which would burst sends and make the jitter worse.
                    next_tick = time.monotonic()
                    await asyncio.sleep(0)
        except asyncio.CancelledError:
            raise

    # ── convenience setters used by the flight code ──────────────────────────

    def command_ned(
        self, north_m_s: float, east_m_s: float, down_m_s: float, yaw_deg: float
    ) -> None:
        """Set a world-frame velocity."""
        self.setpoint.north_m_s = north_m_s
        self.setpoint.east_m_s = east_m_s
        self.setpoint.down_m_s = down_m_s
        self.setpoint.yaw_deg = yaw_deg

    def command_body_horizontal(
        self,
        forward_m_s: float,
        right_m_s: float,
        down_m_s: float,
        heading_deg: float,
    ) -> None:
        """Set a body-relative horizontal velocity, converted to NED here.

        Landing thinks in body terms because the camera is bolted to the
        airframe: "the pad is 0.4 m forward and 0.2 m right of me". Rotating
        that into NED at the call site keeps the publisher single-frame, which
        is what stops the frame-mixing that previously happened mid-descent.

        Because yaw is held constant while descending, this rotation is exact
        rather than an approximation.
        """
        heading_rad = math.radians(heading_deg)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)

        self.setpoint.north_m_s = forward_m_s * cos_h - right_m_s * sin_h
        self.setpoint.east_m_s = forward_m_s * sin_h + right_m_s * cos_h
        self.setpoint.down_m_s = down_m_s
        self.setpoint.yaw_deg = heading_deg

    def command_slewed_ned(
        self,
        target_north_m_s: float,
        target_east_m_s: float,
        down_m_s: float,
        yaw_deg: float,
        dt_s: float,
        max_accel_m_s2: float = config.MAX_ACCEL_M_S2,
    ) -> None:
        """Move the horizontal setpoint toward a target under an acceleration cap.

        Replaces the old fixed-tick velocity blend. The counter version restarted
        whenever the avoider changed its mind mid-blend, producing a velocity
        discontinuity at exactly the moment smoothness mattered. A slew limit has
        no internal state to reset: it simply cannot exceed the acceleration
        limit regardless of how often the target changes.
        """
        self.setpoint.north_m_s = geo.slew_limit(
            self.setpoint.north_m_s, target_north_m_s, max_accel_m_s2, dt_s
        )
        self.setpoint.east_m_s = geo.slew_limit(
            self.setpoint.east_m_s, target_east_m_s, max_accel_m_s2, dt_s
        )
        self.setpoint.down_m_s = down_m_s
        self.setpoint.yaw_deg = yaw_deg

    def hold(self, yaw_deg: float | None = None) -> None:
        """Zero translation, keep streaming, stay in OFFBOARD."""
        self.setpoint.hold(yaw_deg)

    @property
    def stats(self) -> dict[str, float]:
        return {
            "publishes": self._publishes,
            "send_failures": self._send_failures,
            "max_gap_s": round(self._max_gap_s, 4),
            "offboard_active": self._offboard_active,
        }
