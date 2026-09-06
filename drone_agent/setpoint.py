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
        from mavsdk.offboard import OffboardError

        if self._task is not None:
            # The stream is already up, but that does NOT mean the autopilot is
            # still listening to it. See ensure_offboard.
            logger.debug("[%s] setpoint publisher already running", self._drone_id)
            await self.ensure_offboard()
            return

        await self._prime()

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

    async def _prime(self) -> None:
        """Push zero-velocity setpoints so PX4 will accept an OFFBOARD request.

        Two separate reasons, and both are mandatory:

        * PX4 rejects the mode change outright unless setpoints are already
          arriving. This is the documented handshake, not belt-and-braces.
        * MAVSDK will not even send the request. `OffboardImpl::start()` returns
          `NoSetpointSet` when its `_mode` is `NotActive`, and `_mode` is set by
          calling a `set_*` method -- never by `start()` itself. So after a
          `stop()`, something must push a setpoint through the plugin's own API
          before `start()` can do anything.

        The second point is what broke the first re-entry attempt in flight. The
        publishing task was streaming at 20 Hz, so a setpoint was never more
        than 50 ms away -- but re-entry called `stop()` and `start()` 15 ms
        apart and lost the race:

            15:33:31,548  the autopilot is in HOLD, not OFFBOARD - re-entering
            15:33:31,563  could not re-enter OFFBOARD: No Setpoint Set

        Zero velocity, deliberately: a mode change is not the moment to also
        start moving. The publishing task resumes sending the real setpoint on
        its next tick.
        """
        from mavsdk.offboard import VelocityNedYaw

        zero = VelocityNedYaw(0.0, 0.0, 0.0, self.setpoint.yaw_deg)
        for _ in range(max(3, int(self._hz * 0.25))):
            await self._drone.offboard.set_velocity_ned(zero)
            await asyncio.sleep(self._period_s)

    async def ensure_offboard(self) -> None:
        """Re-enter OFFBOARD if the autopilot has left it behind our back.

        THE BUG THIS FIXES: A MISSION THAT COULD ONLY EVER FLY ONE LEG
        --------------------------------------------------------------
        Touchdown is confirmed by commanding `action.land()` and waiting for a
        real disarm, and that command necessarily takes PX4 OUT of offboard --
        into Land, then Disarmed. The next leg then arms and takes off, which
        leaves the vehicle in Takeoff/Hold. At no point does it come back.

        `start()` used to return early whenever its publishing task was alive,
        on the reasonable-sounding grounds that the publisher was already
        running. It was: the setpoints were streaming at 20 Hz the whole time.
        They were simply being ignored, because PX4 was not in offboard to
        receive them. The drone climbed to its cruise altitude and hovered there
        until the navigation timeout, having never moved horizontally:

            15:08:53  at 9.6 m ... navigating to (30.031293, 72.314083)
            15:11:53  still at x=24.85 y=-0.02 z=9.76     (three minutes later)

        Leg 1 had always failed before the landing fixes landed, so no flight
        had ever reached leg 2 and this had never been observable.

        Public because `landing` needs it too: any code path that issues a land
        command has taken the vehicle out of offboard and must put it back
        before its next setpoint means anything.

        WHY THIS ASKS TELEMETRY AND NOT `offboard.is_active()`
        -----------------------------------------------------
        Because `is_active()` does not answer this question. Its docstring says
        "the vehicle is in offboard mode", but MAVSDK implements it as
        `_mode != Mode::NotActive` over the plugin's OWN local state -- state
        set by `start()` and cleared by `stop()`, and never touched when the
        autopilot changes mode on its own. After a land command it therefore
        still reports True, and a first version of this method believed it:

            15:22:50  OFFBOARD engaged, streaming at 20 Hz      (leg 1)
            15:24:16  at 9.6 m ... navigating to ...            (leg 2)
            15:27:14  still parked over the pickup pad
                      -- and not one "re-entered OFFBOARD" line in between

        `telemetry.flight_mode()` comes from the vehicle's HEARTBEAT, so it
        reports what PX4 is actually doing.

        The same local state is why re-entry is `stop()` then `start()`.
        MAVSDK's `start()` returns Success without sending anything when it
        believes it is already active, so clearing that belief is the only way
        to make the mode command go out at all.
        """
        from mavsdk.offboard import OffboardError

        mode = None
        try:
            mode = await asyncio.wait_for(self._read_flight_mode(), timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            # Not knowing is not a reason to skip. Fall through and re-enter:
            # entering offboard when already in it costs one mode command.
            logger.debug("[%s] could not read the flight mode: %s", self._drone_id, exc)

        if mode == "OFFBOARD":
            self._offboard_active = True
            return

        logger.info(
            "[%s] the autopilot is in %s, not OFFBOARD - re-entering",
            self._drone_id,
            mode or "an unreadable mode",
        )
        try:
            # Clear MAVSDK's stale local state so that start() actually sends
            # the mode command instead of short-circuiting to Success.
            await asyncio.wait_for(self._drone.offboard.stop(), timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            logger.debug("[%s] offboard.stop() before re-entry: %s", self._drone_id, exc)
        self._offboard_active = False

        # Re-arm the handshake. Without this, start() returns NoSetpointSet.
        await self._prime()

        try:
            await asyncio.wait_for(self._drone.offboard.start(), timeout=5.0)
            self._offboard_active = True
            logger.info("[%s] re-entered OFFBOARD", self._drone_id)
        except (OffboardError, asyncio.TimeoutError) as exc:
            detail = getattr(getattr(exc, "_result", None), "result_str", exc)
            logger.error(
                "[%s] could not re-enter OFFBOARD: %s. The setpoint stream is "
                "still running but the drone will not follow it.",
                self._drone_id,
                detail,
            )

    async def _read_flight_mode(self) -> str:
        """One sample of the vehicle's actual flight mode, as a bare name."""
        async for mode in self._drone.telemetry.flight_mode():
            return str(mode).rsplit(".", 1)[-1]
        return "UNKNOWN"

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
