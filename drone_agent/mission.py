"""DroneMission: all per-drone state and the FSM-driven delivery orchestrator.

WHY A CLASS (P3.1)
------------------
drone_logic.py held its state in three module-level dicts:

    drone_state = {...}; vision_data = {...}; lidar_data = {...}

Module globals are shared by every importer, so a second drone in the same
process would overwrite the first's position, its marker detections and its
avoidance action. That, and not the absence of a spawn script, is what actually
limited the system to one drone -- and it is why "multi-drone fleet (3 drones)"
could be documented while being structurally impossible.

Everything per-drone now lives on an instance: state dicts, UDP ports, FSM,
setpoint publisher, safety supervisor, payload bay and obstacle reporter. A
module-level registry maps drone_id to the live instance and backs
get_state_snapshot(), which is what the dispatcher's heartbeat writer reads.

THE MISSION SHAPE (P3.3)
------------------------
IDLE -> TAKEOFF -> ENROUTE -> SEARCHING -> APPROACH -> DESCEND -> PAYLOAD_OP
     -> NEXT_LEG  (x3 legs)  -> DONE

Legs come from world.marker_models.LEG_ORDER, so pickup looks for ArUco ID 0,
drop-off for ID 1 and home for ID 2. The previous code hardcoded marker 0 for
all three legs (drone_logic.py:493/509/525) while spawning three identical
undecodable pads, so marker disambiguation had nothing to disambiguate.

A FAILED LEG ROUTES SOMEWHERE (P3.3)
------------------------------------
The old code read:

    landed = await execute_precision_landing(drone, 0)
    if landed:
        ... wait 5s ...
    # no else -- falls through to the next leg regardless

so a failed landing silently proceeded to the next leg from an unknown
altitude, with an unknown payload state. Here, every leg failure resolves to
HOVER_AND_ALERT or ABORT and ends the mission with a reason.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import Any

import config
import world.marker_models as marker_models
from drone_agent import battery, geo, gz_client, landing, navigation, px4_params, udp_receiver
from drone_agent.contracts import DeliveryJob, LegOutcome, MissionResult
from drone_agent.mission_fsm import MissionFSM, MissionState
from drone_agent.safety_supervisor import SafetySupervisor
from drone_agent.setpoint import SetpointPublisher

logger = logging.getLogger(__name__)


# ═════════════════════════════════════════════════════════════════════════════
#  REGISTRY
# ═════════════════════════════════════════════════════════════════════════════

_registry: dict[str, DroneMission] = {}


def get_mission(drone_id: str) -> DroneMission | None:
    """The live mission for a drone, or None."""
    return _registry.get(drone_id)


def get_state_snapshot(drone_id: str) -> dict[str, Any]:
    """Telemetry snapshot for the dispatcher's heartbeat writer.

    THIS IS THE FUNCTION WHOSE ABSENCE WEDGED THE FLEET. fleet_dispatch/app.py
    imported it from inside run_drone_task and above that function's `try:`, so
    the ImportError skipped the `finally: complete_job(...)` and left the drone
    BUSY forever. It is now a real function, imported at module scope.

    Returns a dict with zeroed values -- never raises -- for a drone that has no
    live mission. A telemetry read must not be able to end a flight, and the
    dispatcher polls this every two seconds for drones that may be idle.
    """
    mission = _registry.get(drone_id)
    if mission is None:
        return {
            "drone_id": drone_id,
            "lat": 0.0, "lon": 0.0, "alt": 0.0,
            "battery_pct": None,
            "status": "idle",
            "fsm_state": MissionState.IDLE.value,
            "live": False,
        }
    return mission.snapshot()


def registered_drones() -> list[str]:
    return sorted(_registry)


# ═════════════════════════════════════════════════════════════════════════════
#  MISSION
# ═════════════════════════════════════════════════════════════════════════════

class DroneMission:
    """One drone, one mission, all of its own state."""

    def __init__(self, drone_id: str):
        config.drone_index(drone_id)          # validates the id shape early

        self.drone_id = drone_id
        self.index = config.drone_index(drone_id)

        # ── per-drone state, formerly module globals ────────────────────────
        self.drone_state: dict[str, Any] = {
            "drone_id": drone_id,
            "lat": 0.0, "lon": 0.0, "alt": 0.0,
            "battery_pct": None,
            "landed_state": None,
            "armed": False,
            "status": "created",
            udp_receiver.TELEMETRY_TS_KEY: 0.0,
        }
        self.vision_data: dict[str, Any] = {
            "detections": [],
            udp_receiver.VISION_TS_KEY: 0.0,
        }
        self.lidar_data: dict[str, Any] = {
            "action": "CLEAR",
            "eff_front_m": config.INF_REPLACE_M,
            udp_receiver.LIDAR_TS_KEY: 0.0,
        }

        self.fsm = MissionFSM(label=drone_id)

        self._drone = None
        self._publisher: SetpointPublisher | None = None
        self._supervisor: SafetySupervisor | None = None
        self._payload = None
        self._reporter: navigation.ObstacleReporter | None = None

        self._tasks: list[asyncio.Task] = []
        self._receivers: list[udp_receiver.UdpJsonReceiver] = []

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.known_obstacles: list[dict[str, Any]] = []
        self.legs: list[LegOutcome] = []
        self._started_at = 0.0

    # ── snapshot ────────────────────────────────────────────────────────────

    def snapshot(self) -> dict[str, Any]:
        """Copy of the live state. A copy, so a reader cannot mutate flight state."""
        return {
            "drone_id": self.drone_id,
            "lat": self.drone_state.get("lat", 0.0),
            "lon": self.drone_state.get("lon", 0.0),
            "alt": self.drone_state.get("alt", 0.0),
            "battery_pct": self.drone_state.get("battery_pct"),
            "status": self.drone_state.get("status", ""),
            "fsm_state": self.fsm.state.value,
            "action": self.lidar_data.get("action"),
            "markers_visible": [d.get("id") for d in self.vision_data.get("detections", [])],
            "live": True,
            "elapsed_s": round(time.monotonic() - self._started_at, 1) if self._started_at else 0.0,
        }

    def _status(self, text: str) -> None:
        self.drone_state["status"] = text

    # ── connection ──────────────────────────────────────────────────────────

    async def connect(self, timeout_s: float = config.MAVSDK_PROBE_TIMEOUT_S):
        """Connect to this drone's PX4 instance. Raises on failure.

        ONE address, derived from the drone id: udpin://0.0.0.0:(14540+index),
        which is what PX4's px4-rc.mavlink publishes to. The old code tried
        three hardcoded spellings of port 14540 in a loop, which meant drone-1
        and drone-2 both tried to connect to drone-0 -- and it "succeeded" on
        the first iteration of a connection_state() loop that broke
        unconditionally, so it reported success without a heartbeat.
        """
        from mavsdk import System

        url = config.mavsdk_url(self.drone_id)
        self._status(f"connecting to {url}")
        logger.info("[%s] connecting on %s", self.drone_id, url)

        # Each drone needs its own mavsdk_server gRPC port AND its own MAVLink
        # component identity. Sharing either across three System objects in one
        # process means three clients answering as the same component, and PX4
        # attributing one drone's command acknowledgements to another.
        drone = System(
            mavsdk_server_address=None,
            port=config.MAVSDK_GRPC_PORT_BASE + self.index,
            sysid=config.MAVSDK_SYSID_BASE + self.index,
        )

        # BOUNDED. System.connect() ends in
        # `await aiogrpc.channel_ready_future(channel)`, which waits FOREVER for
        # the mavsdk_server it just spawned. If that server exits during startup
        # -- a malformed connection URL is enough; see config.MAVSDK_URL_SCHEME
        # -- connect() never returns and the drone stays BUSY indefinitely.
        # That is exactly the failure mode this phase exists to eliminate, so the
        # gRPC handshake gets its own timeout, separate from the heartbeat wait.
        try:
            await asyncio.wait_for(
                drone.connect(system_address=url), timeout=timeout_s + 6.0
            )
        except asyncio.TimeoutError as exc:
            raise ConnectionError(
                f"mavsdk_server did not become ready for {url}. It usually "
                f"exits at startup over a rejected connection URL -- run "
                f"`python3 -c \"import config; print(config.mavsdk_url('{self.drone_id}'))\"` "
                f"and check that scheme against your installed MAVSDK version."
            ) from exc

        async def _wait_connected() -> None:
            async for state in drone.core.connection_state():
                if state.is_connected:
                    return

        try:
            await asyncio.wait_for(_wait_connected(), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise ConnectionError(
                f"No MAVLink heartbeat on {url} after {timeout_s:.0f}s. "
                f"PX4 SITL instance {self.index} is not running. "
                f"Start it with world/spawn_fleet.sh and verify with "
                f"scripts/preflight.py."
            ) from exc

        logger.info("[%s] connected", self.drone_id)
        self._drone = drone
        return drone

    async def wait_for_position(self, timeout_s: float = config.GPS_FIX_TIMEOUT_S) -> None:
        """Block until the autopilot has a global position fix."""
        self._status("waiting for GPS")

        async def _wait() -> None:
            async for health in self._drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    return

        try:
            await asyncio.wait_for(_wait(), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                f"No GPS fix after {timeout_s:.0f}s. In SITL this usually means "
                f"the EKF has not converged; check the PX4 console for "
                f"'Preflight Fail' messages."
            ) from exc
        logger.info("[%s] GPS fix acquired", self.drone_id)

    # ── background tasks ────────────────────────────────────────────────────

    def _spawn_task(self, coro, name: str) -> asyncio.Task:
        """Create a tracked task. Untracked tasks can be collected mid-flight."""
        task = asyncio.create_task(coro, name=f"{name}-{self.drone_id}")
        self._tasks.append(task)
        return task

    async def start_telemetry(self) -> None:
        """Subscribe to the telemetry the mission and supervisor depend on.

        FOUR SEPARATE SUBSCRIPTIONS, EACH OPENED ONCE. The old code opened a
        fresh `async for pos in drone.telemetry.position()` inside the landing
        loop, at 10 Hz, breaking after one sample -- a new MAVLink stream every
        100 ms. Here each stream is opened once and read forever.

        Every writer stamps TELEMETRY_TS_KEY. That timestamp is what the safety
        supervisor's heartbeat check reads, and nothing wrote it before, which is
        why the check fired an emergency RTL on its first tick.
        """
        async def position() -> None:
            async for pos in self._drone.telemetry.position():
                self.drone_state["lat"] = pos.latitude_deg
                self.drone_state["lon"] = pos.longitude_deg
                self.drone_state["alt"] = pos.relative_altitude_m
                self.drone_state[udp_receiver.TELEMETRY_TS_KEY] = time.monotonic()

        async def battery_stream() -> None:
            async for reading in self._drone.telemetry.battery():
                # MAVSDK 2.x: 0-100. See drone_agent/battery.py on the pin.
                self.drone_state["battery_pct"] = reading.remaining_percent
                self.drone_state[udp_receiver.TELEMETRY_TS_KEY] = time.monotonic()

        async def landed_state() -> None:
            # The primary touchdown signal: PX4 fuses altitude, vertical
            # velocity and thrust, so it neither fires while descending past a
            # threshold nor misses a landing on raised ground.
            async for state in self._drone.telemetry.landed_state():
                self.drone_state["landed_state"] = str(state).rsplit(".", 1)[-1]

        async def armed_state() -> None:
            async for armed in self._drone.telemetry.armed():
                self.drone_state["armed"] = armed

        for coro, name in (
            (position(), "telemetry-position"),
            (battery_stream(), "telemetry-battery"),
            (landed_state(), "telemetry-landed"),
            (armed_state(), "telemetry-armed"),
        ):
            self._spawn_task(coro, name)

        # Let the first samples land before anything reads them.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.drone_state[udp_receiver.TELEMETRY_TS_KEY]:
                return
            await asyncio.sleep(0.1)
        logger.warning(
            "[%s] no telemetry sample within 10s of subscribing", self.drone_id
        )

    async def start_sensor_receivers(self) -> None:
        """Start the UDP readers for this drone's vision and LiDAR feeds."""
        for factory, state in (
            (udp_receiver.make_lidar_receiver, self.lidar_data),
            (udp_receiver.make_vision_receiver, self.vision_data),
        ):
            receiver = factory(self.drone_id, state)
            receiver.bind()
            self._receivers.append(receiver)
            self._spawn_task(receiver.run(), "receiver")

        logger.info(
            "[%s] sensor receivers listening: lidar=%d vision=%d",
            self.drone_id,
            config.lidar_port(self.drone_id),
            config.vision_port(self.drone_id),
        )

    # ── setup ───────────────────────────────────────────────────────────────

    async def prepare(self, job: DeliveryJob) -> None:
        """Everything between connecting and the first takeoff."""
        from drone_agent.payload import PayloadBay

        await self.start_sensor_receivers()
        await self.start_telemetry()
        await self.wait_for_position()

        self.home_lat = self.drone_state["lat"]
        self.home_lon = self.drone_state["lon"]
        logger.info(
            "[%s] home is (%.7f, %.7f)", self.drone_id, self.home_lat, self.home_lon
        )

        # P3.9: apply the tuning that used to be a comment asking the operator to
        # type seven values into a PX4 shell.
        _applied, problems = await px4_params.apply_params(self._drone)
        if problems:
            logger.warning(
                "[%s] flying with unconfirmed tuning: %s",
                self.drone_id, "; ".join(problems),
            )

        self._publisher = SetpointPublisher(self._drone, self.drone_id)
        self._reporter = navigation.ObstacleReporter(self.drone_id)

        self._supervisor = SafetySupervisor(
            self._drone, self.drone_state, self.drone_id,
            self.home_lat, self.home_lon,
            lidar_data=self.lidar_data,
        )
        # Autopilot-enforced fence, which survives this process dying.
        await self._supervisor.upload_geofence()
        self._spawn_task(self._supervisor.supervise(), "supervisor")

        self._payload = PayloadBay(self.drone_id, self.home_lat, self.home_lon)

        await self._prefetch_obstacles(job)
        await self._spawn_pads(job)

    async def _prefetch_obstacles(self, job: DeliveryJob) -> None:
        """Fetch known obstacles ONCE, before leg 1, and cache for the mission.

        Once is what the obstacle client's own docstring specifies, and it is the
        right call: mid-mission polling would let the planner change its mind
        about a route the drone is already committed to.

        The bounding box is padded by the detour margin so an obstacle just
        outside the straight-line corridor -- exactly the one worth routing
        around -- is still returned.
        """
        from drone_agent.obstacle_client import fetch_known_obstacles_for_mission

        bbox = geo.bounding_box(
            [job.pickup, job.drop, (self.home_lat, self.home_lon)],
            pad_m=config.DETOUR_MARGIN_M * 4.0,
        )
        self.known_obstacles = await fetch_known_obstacles_for_mission(bbox)
        logger.info(
            "[%s] %d known obstacle(s) prefetched for this mission",
            self.drone_id, len(self.known_obstacles),
        )

    async def _spawn_pads(self, job: DeliveryJob) -> None:
        """Place the three landing pads, each with its own ArUco ID.

        Uses marker_models.PAD_MODELS -- pad_0/pad_1/pad_2 -- not
        `model://arucotag`, whose texture decodes in no dictionary at all
        (finding F1). Pads are namespaced per drone so a fleet does not fight
        over entity names.
        """
        if not gz_client.gz_available():
            logger.warning(
                "[%s] `gz` not on PATH: skipping pad spawning. Precision "
                "landing will find nothing to land on.", self.drone_id,
            )
            return

        targets = {
            "pickup_pad": job.pickup,
            "drop_pad": job.drop,
            "home_pad": (self.home_lat, self.home_lon),
        }

        for role, (lat, lon) in targets.items():
            ok = await gz_client.spawn_pad_at_gps(
                role, lat, lon, self.home_lat, self.home_lon,
                name_prefix=f"{self.drone_id}_",
            )
            if not ok:
                logger.error(
                    "[%s] failed to spawn %s (%s). Landing on marker %d will "
                    "not be possible.",
                    self.drone_id, role, marker_models.PAD_MODELS[role],
                    marker_models.ROLE_TO_ID[role],
                )

        await self._payload.spawn(*job.pickup)

    # ── flight ──────────────────────────────────────────────────────────────

    async def arm_and_takeoff(self, target_alt_m: float = config.TARGET_ALT_M) -> None:
        """Wait for health, arm, take off, and confirm the altitude.

        Waits on is_armable in addition to position health: arming a drone whose
        EKF has not converged is how a takeoff ends up refused with a
        'Preflight Fail' the Python side never sees.
        """
        self._status("waiting for EKF")
        deadline = time.monotonic() + config.EKF_CONVERGE_TIMEOUT_S

        async def _wait_armable() -> None:
            async for health in self._drone.telemetry.health():
                if (health.is_global_position_ok and health.is_home_position_ok
                        and health.is_armable):
                    return
                await asyncio.sleep(0.5)

        try:
            await asyncio.wait_for(
                _wait_armable(), timeout=max(1.0, deadline - time.monotonic())
            )
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                f"EKF/GPS did not converge within {config.EKF_CONVERGE_TIMEOUT_S}s. "
                f"Check the PX4 console for 'Preflight Fail' messages."
            ) from exc

        self._status("arming")
        await self._drone.action.set_takeoff_altitude(target_alt_m)
        await self._drone.action.arm()
        await asyncio.sleep(1.0)

        self._status(f"taking off to {target_alt_m:.0f} m")
        await self._drone.action.takeoff()

        deadline = time.monotonic() + config.TAKEOFF_TIMEOUT_S
        while True:
            altitude_m = self.drone_state["alt"]
            if altitude_m >= target_alt_m - 1.5:
                break
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Takeoff did not reach {target_alt_m:.0f} m within "
                    f"{config.TAKEOFF_TIMEOUT_S}s (stalled at {altitude_m:.1f} m). "
                    f"Check the takeoff command was accepted and the EKF is healthy."
                )
            await asyncio.sleep(0.5)

        await asyncio.sleep(2.0)              # let the hover settle
        logger.info("[%s] at %.1f m", self.drone_id, self.drone_state["alt"])

        # Hand over to offboard from a stable hover, once for the whole leg.
        self._publisher.setpoint.yaw_deg = 0.0
        await self._publisher.start()
        self.fsm.fire("altitude_reached")

    async def fly_leg(self, role: str, target: tuple[float, float]) -> LegOutcome:
        """One leg: takeoff, navigate, land on the right marker, do the payload op.

        Returns a LegOutcome describing how far it got. `detail` is non-empty on
        failure -- a leg that failed without a reason is what made the previous
        behaviour so hard to diagnose.
        """
        marker_id = marker_models.marker_id_for_role(role)
        outcome = LegOutcome(role=role, marker_id=marker_id)
        target_lat, target_lon = target

        logger.info(
            "%s\n[%s] LEG %s -> (%.6f, %.6f), marker %d\n%s",
            "=" * 62, self.drone_id, role, target_lat, target_lon, marker_id, "=" * 62,
        )

        # ── pre-leg battery gate ────────────────────────────────────────────
        allowed, reason = battery.check_leg_battery(
            self.drone_state, target_lat, target_lon, self.fsm
        )
        if not allowed:
            outcome.detail = reason
            return outcome

        # ── takeoff (or resume, if already airborne) ─────────────────────────
        if not self.drone_state.get("armed", False):
            await self.arm_and_takeoff()
        else:
            self.fsm.fire("altitude_reached")

        # ── navigate ────────────────────────────────────────────────────────
        nav_result = await navigation.navigate_to(
            self._publisher, target_lat, target_lon,
            self.drone_state, self.lidar_data,
            drone_id=self.drone_id,
            reporter=self._reporter,
            known_obstacles=self.known_obstacles,
            on_status=self._status,
        )
        outcome.detours = self._reporter.reports_sent if self._reporter else 0

        if nav_result != navigation.NavOutcome.ARRIVED:
            outcome.detail = f"navigation ended as '{nav_result}'"
            if nav_result == navigation.NavOutcome.ESCALATION_EXHAUSTED:
                self.fsm.fire("blocked")
            else:
                self.fsm.force(MissionState.ABORT, outcome.detail)
            return outcome

        outcome.arrived = True
        self.fsm.fire("arrived")

        # ── land on THIS leg's marker ───────────────────────────────────────
        landing_result = await landing.execute_precision_landing(
            self._publisher, self._drone, marker_id,
            target_lat, target_lon,
            self.vision_data, self.drone_state,
            drone_id=self.drone_id, fsm=self.fsm,
        )

        if landing_result != landing.LandingOutcome.TOUCHDOWN:
            # THE MISSING `else`. The old code fell through to the next leg from
            # an unknown altitude when a landing failed.
            outcome.detail = f"landing ended as '{landing_result}'"
            return outcome

        outcome.landed = True

        # ── payload operation ───────────────────────────────────────────────
        if role == "pickup_pad":
            outcome.payload_op_ok = await self._payload.attach()
            await self._payload.start_following(self.drone_state)
        elif role == "drop_pad":
            outcome.payload_op_ok = await self._payload.release()
            await self._payload.stop_following()

        if outcome.payload_op_ok is False:
            self.fsm.fire("op_failed_x2")
            outcome.detail = "payload operation failed"
            return outcome

        self.fsm.fire("op_confirmed")
        await asyncio.sleep(3.0)              # visible dwell on the pad
        return outcome

    # ── entry point ─────────────────────────────────────────────────────────

    async def run(self, job: DeliveryJob) -> MissionResult:
        """Fly the whole job. Always returns; never leaks tasks.

        The MissionResult carries the reason and the per-leg outcomes, so the
        dispatcher can record something an operator can act on.
        """
        _registry[self.drone_id] = self
        self._started_at = time.monotonic()

        try:
            await self.connect()
            await self.prepare(job)

            self.fsm.fire("start")

            legs = [
                ("pickup_pad", job.pickup),
                ("drop_pad", job.drop),
                ("home_pad", (self.home_lat, self.home_lon)),
            ]

            for index, (role, target) in enumerate(legs):
                if self._supervisor and self._supervisor.emergency_triggered:
                    reason = self._supervisor.emergency_reason
                    self.fsm.force(MissionState.ABORT, reason)
                    return MissionResult.aborted(
                        f"Safety supervisor intervened: {reason}",
                        self.legs, self.snapshot(),
                    )

                outcome = await self.fly_leg(role, target)
                self.legs.append(outcome)

                if not (outcome.arrived and outcome.landed):
                    self._status(f"leg {role} failed")
                    return MissionResult.failed(
                        f"Leg '{role}' (marker {outcome.marker_id}) failed: "
                        f"{outcome.detail or 'no detail'}",
                        self.legs, self.snapshot(),
                    )

                if index < len(legs) - 1:
                    self.fsm.fire("more_legs")
                else:
                    self.fsm.fire("mission_complete")

            self._status("mission complete")
            logger.info("[%s] all legs complete", self.drone_id)
            return MissionResult.completed(self.legs, self.snapshot())

        except asyncio.CancelledError:
            self._status("cancelled")
            self.fsm.force(MissionState.ABORT, "cancelled")
            raise

        except Exception as exc:  # noqa: BLE001 - the reason must reach the caller
            logger.error("[%s] mission raised", self.drone_id, exc_info=True)
            self._status(f"failed: {exc}")
            self.fsm.force(MissionState.ABORT, str(exc))
            return MissionResult.failed(
                f"{type(exc).__name__}: {exc}", self.legs, self.snapshot()
            )

        finally:
            await self.shutdown()
            _registry.pop(self.drone_id, None)

    async def shutdown(self) -> None:
        """Stop every task and release every resource. Must never raise.

        Ordering matters: stop commanding before tearing down the things the
        commands depend on, and leave the airframe in a defined state.
        """
        logger.info("[%s] shutting down", self.drone_id)

        if self._reporter is not None:
            with contextlib.suppress(Exception):
                await self._reporter.drain()

        if self._payload is not None:
            with contextlib.suppress(Exception):
                await self._payload.cleanup()

        if self._publisher is not None:
            with contextlib.suppress(Exception):
                await self._publisher.stop()
            logger.info("[%s] setpoint stats: %s", self.drone_id, self._publisher.stats)

        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task
        self._tasks.clear()

        for receiver in self._receivers:
            receiver.close()
        self._receivers.clear()

        logger.info(
            "[%s] shutdown complete; FSM ended in %s after %d transition(s)",
            self.drone_id, self.fsm.state.value, len(self.fsm.history),
        )
