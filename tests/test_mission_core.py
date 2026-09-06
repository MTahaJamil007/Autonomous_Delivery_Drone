"""P3 acceptance: mission core invariants that need no simulator.

The criteria that require a live PX4 (a 3-leg mission reaching DONE, zero
OFFBOARD rejections, the cargo box tracking the airframe) are the SITL
procedures in docs/ACCEPTANCE.md. What is here is everything provable on the
bench: per-drone state isolation, the setpoint stream's continuity, the frame
conversion the landing loop depends on, and the leg wiring that used to hardcode
marker 0 three times.
"""

import asyncio
import math
from pathlib import Path

import pytest

import config
import world.marker_models as marker_models
from drone_agent import geo
from drone_agent.contracts import DeliveryJob
from drone_agent.mission import DroneMission, get_state_snapshot
from drone_agent.mission_fsm import MissionFSM, MissionState
from drone_agent.setpoint import Setpoint, SetpointPublisher

# ─────────────────────────────────────────────────────────────────────────────
#  P3.1 per-drone state
# ─────────────────────────────────────────────────────────────────────────────


def test_two_missions_keep_independent_state():
    """P3 acceptance: two DroneMission instances in one process, no sim.

    drone_logic.py held drone_state, vision_data and lidar_data as MODULE
    GLOBALS. Module globals are shared by every importer, so a second drone
    overwrote the first's position, detections and avoidance action. That, not a
    missing spawn script, is what actually made a fleet impossible.
    """
    first = DroneMission("drone-0")
    second = DroneMission("drone-1")

    first.drone_state["lat"] = 30.0
    second.drone_state["lat"] = 31.0
    first.lidar_data["action"] = "DODGE_LEFT"
    second.lidar_data["action"] = "CLEAR"
    first.vision_data["detections"] = [{"id": 0}]
    second.vision_data["detections"] = [{"id": 2}]

    assert first.drone_state["lat"] == 30.0
    assert second.drone_state["lat"] == 31.0
    assert first.lidar_data["action"] == "DODGE_LEFT"
    assert second.lidar_data["action"] == "CLEAR"
    assert first.vision_data["detections"] != second.vision_data["detections"]

    for attribute in ("drone_state", "vision_data", "lidar_data", "fsm"):
        assert getattr(first, attribute) is not getattr(second, attribute), (
            f"{attribute} is shared between instances"
        )


def test_each_drone_gets_distinct_ports_and_identity():
    """Ports, gRPC ports and MAVLink sysids must not collide across the fleet."""
    seen: set[tuple] = set()
    for index in range(config.MAX_FLEET_SIZE):
        drone_id = f"drone-{index}"
        identity = (
            config.mavsdk_port(drone_id),
            config.vision_port(drone_id),
            config.lidar_port(drone_id),
            config.MAVSDK_GRPC_PORT_BASE + index,
            config.MAVSDK_SYSID_BASE + index,
        )
        assert identity not in seen, f"{drone_id} collides with an earlier drone"
        seen.add(identity)

    # And the whole set must be internally distinct, not just pairwise.
    flattened = [value for identity in seen for value in identity]
    assert len(set(flattened)) == len(flattened), "a port or sysid is reused"


def test_fleet_size_is_capped_where_px4_reuses_a_port():
    """PX4 maps instances above 9 onto port 14549, so 10 is a hard ceiling."""
    assert config.MAX_FLEET_SIZE == 10
    ports = {config.mavsdk_port(f"drone-{i}") for i in range(config.MAX_FLEET_SIZE)}
    assert len(ports) == config.MAX_FLEET_SIZE, "the capped fleet must have unique ports"


def test_snapshot_for_an_unregistered_drone_does_not_raise():
    """The dispatcher polls this every 2 s for drones that may be idle.

    This is the function whose absence raised ImportError above a try block and
    wedged the fleet, so it must be maximally boring: never raise, always return
    the contract shape.
    """
    snapshot = get_state_snapshot("drone-7")
    assert snapshot["live"] is False
    assert snapshot["lat"] == 0.0
    assert snapshot["fsm_state"] == MissionState.IDLE.value


def test_snapshot_is_a_copy_not_the_live_state():
    """A reader must not be able to mutate flight state."""
    mission = DroneMission("drone-0")
    mission.drone_state["lat"] = 12.0
    snapshot = mission.snapshot()
    snapshot["lat"] = 999.0
    assert mission.drone_state["lat"] == 12.0


def test_invalid_drone_id_is_rejected_at_construction():
    for bad in ("drone", "drone-", "drone-x", "quad-0", "drone--1"):
        with pytest.raises(ValueError):
            DroneMission(bad)


# ─────────────────────────────────────────────────────────────────────────────
#  P3.2 setpoint publisher
# ─────────────────────────────────────────────────────────────────────────────


class FakeOffboardError(Exception):
    """Stands in for mavsdk.offboard.OffboardError, which carries a _result."""

    def __init__(self, result_str):
        super().__init__(result_str)

        class _R:
            pass

        self._result = _R()
        self._result.result_str = result_str


class FakeOffboard:
    def __init__(self):
        self.sent: list[tuple] = []
        self.started = 0
        self.stopped = 0
        # TWO separate facts, which is the whole point of this double.
        #
        # `plugin_thinks_active` is MAVSDK's own local flag, set by start() and
        # cleared by stop(). `vehicle_mode` is what PX4 is really doing. They
        # diverge the moment the autopilot changes mode by itself -- a land
        # command, a failsafe, an RC override -- because nothing tells the
        # plugin. A double that collapsed them into one boolean could not
        # reproduce the bug that grounded multi-leg missions.
        self.plugin_thinks_active = False
        self.vehicle_mode = "HOLD"
        # Whether a setpoint has been pushed through the plugin since the last
        # stop(). start() fails with NoSetpointSet until one has.
        self.setpoint_seen = False
        self.start_rejections = 0

    async def set_velocity_ned(self, velocity):
        self.sent.append(
            (
                velocity.north_m_s,
                velocity.east_m_s,
                velocity.down_m_s,
                velocity.yaw_deg,
            )
        )
        # In MAVSDK it is the set_* calls -- never start() -- that take the
        # plugin out of Mode::NotActive. Modelled, because start() returns
        # NoSetpointSet without it.
        self.setpoint_seen = True

    async def start(self):
        self.started += 1
        # MAVSDK short-circuits to Success without sending anything when it
        # already believes offboard is active. Modelled, because it is the
        # reason re-entry has to stop() first.
        if self.plugin_thinks_active:
            return
        if not self.setpoint_seen:
            self.start_rejections += 1
            raise FakeOffboardError("NoSetpointSet")
        self.plugin_thinks_active = True
        self.vehicle_mode = "OFFBOARD"

    async def stop(self):
        self.stopped += 1
        self.plugin_thinks_active = False
        self.setpoint_seen = False
        self.vehicle_mode = "HOLD"

    async def is_active(self):
        # Deliberately reports the PLUGIN's local state, not the vehicle's mode
        # -- which is exactly what MAVSDK does, and exactly why this is not the
        # question SetpointPublisher asks.
        return self.plugin_thinks_active


class FakeFlightModeTelemetry:
    def __init__(self, offboard):
        self._offboard = offboard

    async def flight_mode(self):
        yield f"FlightMode.{self._offboard.vehicle_mode}"


class FakeDrone:
    def __init__(self):
        self.offboard = FakeOffboard()
        self.telemetry = FakeFlightModeTelemetry(self.offboard)


async def test_publisher_streams_continuously_and_enters_offboard_once():
    """The invariant: one OFFBOARD session, no gaps, whatever the mission does.

    Before, navigate_with_avoidance called start() then stop() per leg and
    execute_precision_landing called start() again - six start/stop pairs per
    3-leg mission. drone_logic.py's own V5 docstring warned that "PX4 cannot
    handle rapid mode thrashing" while the code did exactly that.
    """
    drone = FakeDrone()
    publisher = SetpointPublisher(drone, "drone-0", hz=50)

    await publisher.start()
    assert drone.offboard.started == 1

    publisher.command_ned(2.0, 0.0, -0.1, 90.0)
    await asyncio.sleep(0.25)
    publisher.command_ned(0.0, 3.0, 0.0, 180.0)
    await asyncio.sleep(0.25)

    await publisher.stop()

    assert drone.offboard.started == 1, "offboard must be entered exactly once"
    assert drone.offboard.stopped == 1, "and left exactly once, at mission end"

    stats = publisher.stats
    assert stats["publishes"] > 15, f"stream too sparse: {stats}"
    assert stats["send_failures"] == 0
    # PX4 tolerates roughly 0.5 s. A generous bound here still proves the
    # stream is continuous rather than bursty.
    assert stats["max_gap_s"] < 0.4, f"setpoint gap too large: {stats}"

    # Both commands actually reached the wire.
    assert (2.0, 0.0, -0.1, 90.0) in drone.offboard.sent
    assert (0.0, 3.0, 0.0, 180.0) in drone.offboard.sent


async def test_publisher_keeps_streaming_while_mission_code_blocks():
    """A slow mission tick must become a late DECISION, not a dropped setpoint.

    This is finding F5 in miniature: payload.py ran a synchronous subprocess in
    an async tick, freezing the loop for tens to hundreds of milliseconds at
    10 Hz and starving the very stream PX4 requires.
    """
    drone = FakeDrone()
    publisher = SetpointPublisher(drone, "drone-0", hz=50)
    await publisher.start()

    publisher.command_ned(1.0, 0.0, 0.0, 0.0)
    before = publisher.stats["publishes"]
    # A well-behaved slow operation: yields to the loop, as gz_client now does.
    await asyncio.sleep(0.3)
    after = publisher.stats["publishes"]

    await publisher.stop()
    assert after - before >= 10, (
        f"the publisher must keep sending during a slow await, got {after - before} sends in 0.3 s"
    )


def test_hold_keeps_streaming_rather_than_stopping():
    """Fail-closed means "hold position in OFFBOARD", not "leave OFFBOARD".

    Leaving offboard hands control back to the autopilot's failsafe; holding
    keeps the drone ours and lets it resume the instant the feed returns.
    """
    setpoint = Setpoint(north_m_s=3.0, east_m_s=2.0, down_m_s=-0.5, yaw_deg=45.0)
    setpoint.hold()
    assert (setpoint.north_m_s, setpoint.east_m_s, setpoint.down_m_s) == (0.0, 0.0, 0.0)
    assert setpoint.yaw_deg == 45.0, "heading is retained so the drone does not spin"


@pytest.mark.parametrize(
    "forward,right,heading,expected_north,expected_east",
    [
        (1.0, 0.0, 0.0, 1.0, 0.0),  # nose north, fly forward -> north
        (1.0, 0.0, 90.0, 0.0, 1.0),  # nose east,  fly forward -> east
        (1.0, 0.0, 180.0, -1.0, 0.0),  # nose south
        (0.0, 1.0, 0.0, 0.0, 1.0),  # nose north, fly right   -> east
        (0.0, 1.0, 90.0, -1.0, 0.0),  # nose east,  fly right   -> south
    ],
)
def test_body_to_ned_conversion(forward, right, heading, expected_north, expected_east):
    """The landing loop thinks in body terms; the publisher streams one frame.

    Converting at the call site is what keeps the publisher single-frame.
    landing.py used to alternate set_velocity_body() for the descent with
    set_velocity_ned() for the search inside one offboard session, at 10 Hz.
    """
    publisher = SetpointPublisher.__new__(SetpointPublisher)
    publisher.setpoint = Setpoint()
    publisher.command_body_horizontal(forward, right, 0.0, heading)

    assert publisher.setpoint.north_m_s == pytest.approx(expected_north, abs=1e-9)
    assert publisher.setpoint.east_m_s == pytest.approx(expected_east, abs=1e-9)
    assert publisher.setpoint.yaw_deg == heading


def test_acceleration_limit_replaces_the_tick_countdown():
    """P3.7: a slew limit has no state to reset when the target changes.

    The old blend was a tick countdown, so an action change mid-blend restarted
    it and produced a velocity discontinuity at exactly the moment smoothness
    mattered. A slew limit simply cannot exceed the acceleration bound.
    """
    publisher = SetpointPublisher.__new__(SetpointPublisher)
    publisher.setpoint = Setpoint()
    dt_s = 1.0 / config.NAV_HZ

    # Ramp toward +5 m/s north, then reverse the target mid-ramp.
    for _ in range(5):
        publisher.command_slewed_ned(5.0, 0.0, 0.0, 0.0, dt_s)
    mid = publisher.setpoint.north_m_s
    assert 0 < mid < 5.0, "the ramp must be gradual"

    previous = mid
    for _ in range(5):
        publisher.command_slewed_ned(-5.0, 0.0, 0.0, 0.0, dt_s)
        step = abs(publisher.setpoint.north_m_s - previous)
        assert step <= config.MAX_ACCEL_M_S2 * dt_s + 1e-9, (
            f"step {step:.4f} exceeds the {config.MAX_ACCEL_M_S2} m/s^2 limit "
            f"even across a target reversal"
        )
        previous = publisher.setpoint.north_m_s


def test_acceleration_limit_matches_the_autopilots():
    """Our limit and PX4's MPC_ACC_HOR must agree, or one silently dominates."""
    assert config.PX4_PARAMS["MPC_ACC_HOR"] == config.MAX_ACCEL_M_S2


# ─────────────────────────────────────────────────────────────────────────────
#  P3.3 leg wiring
# ─────────────────────────────────────────────────────────────────────────────


def test_each_leg_looks_for_its_own_marker():
    """P3.3: legs come from marker_models, not three hardcoded zeroes.

    drone_logic.py:493/509/525 all passed marker id 0, while spawning three
    identical undecodable pads. Disambiguation had nothing to disambiguate.
    """
    ids = [marker_models.marker_id_for_role(role) for role in marker_models.LEG_ORDER]
    assert ids == [0, 1, 2], f"the three legs must want distinct markers, got {ids}"
    assert len(set(ids)) == 3


def test_every_pad_role_maps_to_a_decodable_vendored_model():
    """Never model://arucotag, whose texture decodes in zero dictionaries (F1)."""
    project_root = Path(__file__).resolve().parents[1]
    for role, uri in marker_models.PAD_MODELS.items():
        assert uri != "model://arucotag", (
            f"{role} points at the undecodable model - this is finding F1"
        )
        model_dir = project_root / "sim" / "models" / uri.removeprefix("model://")
        assert (model_dir / "model.sdf").exists(), f"{role}: {model_dir} missing"


def test_unknown_pad_role_raises_instead_of_defaulting_to_zero():
    """Silently defaulting to marker 0 is how all three legs ended up identical."""
    with pytest.raises(KeyError):
        marker_models.marker_id_for_role("helipad")


def test_marker_id_and_role_maps_are_consistent():
    for role, marker_id in marker_models.ROLE_TO_ID.items():
        assert marker_models.ID_TO_ROLE[marker_id] == role


# ─────────────────────────────────────────────────────────────────────────────
#  P3.3 FSM routing
# ─────────────────────────────────────────────────────────────────────────────


def test_a_failed_landing_routes_somewhere_explicit():
    """The missing `else`.

    `landed = await execute_precision_landing(...); if landed: ...` had no else,
    so a failed landing fell through to the next leg from an unknown altitude
    with an unknown payload state.
    """
    fsm = MissionFSM(label="test")
    fsm.fire("start")
    fsm.fire("altitude_reached")
    fsm.fire("arrived")
    assert fsm.state is MissionState.SEARCHING

    assert fsm.fire("search_exhausted")
    assert fsm.state is MissionState.HOVER_AND_ALERT, (
        "a failed search must reach an explicit alert state"
    )


def test_pre_leg_battery_refusal_can_reach_abort():
    """The gate runs before takeoff, so it needs battery_low in IDLE/NEXT_LEG.

    The original table only allowed battery_low from ENROUTE - i.e. only after
    the drone was already airborne.
    """
    for start_state, path in (
        (MissionState.IDLE, []),
        (
            MissionState.NEXT_LEG,
            ["start", "altitude_reached", "arrived", "marker_locked", "touchdown", "op_confirmed"],
        ),
    ):
        fsm = MissionFSM()
        for event in path:
            assert fsm.fire(event), f"setup event {event} failed"
        assert fsm.state is start_state, f"expected {start_state}, got {fsm.state}"
        assert fsm.fire("battery_low"), f"battery_low must be valid in {start_state}"
        assert fsm.state is MissionState.ABORT


def test_a_blocked_leg_routes_to_hover_and_alert():
    """Detours exhausted must hold for a human, not keep generating paths."""
    fsm = MissionFSM()
    fsm.fire("start")
    fsm.fire("altitude_reached")
    assert fsm.state is MissionState.ENROUTE
    assert fsm.fire("blocked")
    assert fsm.state is MissionState.HOVER_AND_ALERT


def test_supervisor_can_force_an_abort_from_any_state():
    """A safety layer must not need the transition table's permission."""
    for state in MissionState:
        fsm = MissionFSM(initial_state=state)
        fsm.force(MissionState.ABORT, "geofence breach")
        assert fsm.state is MissionState.ABORT
        assert fsm.history[-1][2] == MissionState.ABORT.value


def test_terminal_states_accept_nothing():
    """A late event from a task being torn down must not resurrect a mission."""
    for terminal in (MissionState.DONE, MissionState.ABORT):
        fsm = MissionFSM(initial_state=terminal)
        assert not fsm.fire("start")
        assert fsm.state is terminal


def test_the_full_happy_path_reaches_done():
    """Three legs, each landing on its own marker, ending in DONE."""
    fsm = MissionFSM()
    assert fsm.fire("start")

    for leg in range(3):
        assert fsm.state is MissionState.TAKEOFF, f"leg {leg}: {fsm.state}"
        assert fsm.fire("altitude_reached")
        assert fsm.fire("arrived")
        assert fsm.fire("marker_locked")
        assert fsm.fire("centered_stable")
        assert fsm.fire("touchdown")
        assert fsm.fire("op_confirmed")
        assert fsm.state is MissionState.NEXT_LEG
        assert fsm.fire("more_legs" if leg < 2 else "mission_complete")

    assert fsm.state is MissionState.DONE
    assert len(fsm.history) == 22


# ─────────────────────────────────────────────────────────────────────────────
#  P3.4 obstacle prefetch
# ─────────────────────────────────────────────────────────────────────────────


def test_prefetch_bbox_covers_all_three_legs_with_margin():
    """A tight box excludes exactly the obstacles a detour would route around."""
    job = DeliveryJob("j1", 30.0320, 72.3145, 30.0350, 72.3180)
    home = (30.0315, 72.3141)

    min_lat, max_lat, min_lon, max_lon = geo.bounding_box(
        [job.pickup, job.drop, home], pad_m=config.DETOUR_MARGIN_M * 4.0
    )

    for lat, lon in (job.pickup, job.drop, home):
        assert min_lat < lat < max_lat
        assert min_lon < lon < max_lon

    pad_deg = (config.DETOUR_MARGIN_M * 4.0) / geo.METRES_PER_DEG_LAT
    assert min_lat < min(job.pickup_lat, job.drop_lat, home[0]) - pad_deg * 0.9


def test_empty_bounding_box_raises_rather_than_matching_nothing():
    """A zero-area box reads identically to "there are no obstacles"."""
    with pytest.raises(ValueError):
        geo.bounding_box([])


# ─────────────────────────────────────────────────────────────────────────────
#  P3.9 PX4 parameters
# ─────────────────────────────────────────────────────────────────────────────


async def test_px4_params_are_read_back_not_just_written():
    """PX4 silently ignores unknown names and clamps out-of-range values.

    A write-only "apply" would report success for a value the autopilot rejected,
    so every flight would be of unknown configuration.
    """
    from drone_agent import px4_params

    class FakeParam:
        def __init__(self, clamp: dict[str, float] | None = None):
            self.written: dict[str, float] = {}
            self._clamp = clamp or {}

        async def set_param_float(self, name, value):
            self.written[name] = self._clamp.get(name, value)

        async def get_param_float(self, name):
            return self.written[name]

    class FakeDroneParams:
        def __init__(self, clamp=None):
            self.param = FakeParam(clamp)

    drone = FakeDroneParams()
    applied, problems = await px4_params.apply_params(drone)
    assert problems == [], f"a compliant autopilot must report no problems: {problems}"
    assert applied == {k: float(v) for k, v in config.PX4_PARAMS.items()}

    # Now an autopilot that clamps one value: it must be reported, not hidden.
    clamped = FakeDroneParams(clamp={"MPC_JERK_MAX": 4.0})
    applied, problems = await px4_params.apply_params(clamped)
    assert len(problems) == 1
    assert "MPC_JERK_MAX" in problems[0]
    assert "clamped" in problems[0]
    assert applied["MPC_JERK_MAX"] == 4.0, "the readback value is what we report"


async def test_px4_param_failure_does_not_abort_the_mission():
    """Tuning is an optimisation, not a precondition.

    Refusing to fly because MPC_JERK_MAX could not be set would ground the fleet
    over a comfort setting.
    """
    from drone_agent import px4_params

    class BrokenParam:
        async def set_param_float(self, name, value):
            raise RuntimeError("PARAM_DENIED")

        async def get_param_float(self, name):
            raise RuntimeError("PARAM_DENIED")

    class BrokenDrone:
        param = BrokenParam()

    applied, problems = await px4_params.apply_params(BrokenDrone())
    assert applied == {}
    assert len(problems) == len(config.PX4_PARAMS), "every failure must be reported"


async def test_publisher_re_enters_offboard_after_the_autopilot_leaves_it():
    """A mission must be able to fly more than one leg.

    THE FLIGHT THAT EXPOSED THIS. Leg 1 landed perfectly. Leg 2 armed, climbed
    to 9.6 m, announced that it was navigating -- and then hovered over the
    pickup pad, motionless, until the navigation timeout:

        15:08:33  disarmed - landing confirmed        (leg 1 lands)
        15:08:36  NEXT_LEG --[more_legs]--> TAKEOFF
        15:08:53  at 9.6 m ... navigating to (30.031293, 72.314083)
        15:11:53  still at x=24.85 y=-0.02 z=9.76     (three minutes later)

    THE CAUSE. Confirming a touchdown means commanding `action.land()` and
    waiting for a real disarm, and that takes PX4 out of offboard by design. The
    next leg's takeoff leaves it in Takeoff/Hold. `start()` then returned early
    because its publishing task was still alive -- which was true, and beside
    the point: the setpoints were streaming at 20 Hz into a mode that was not
    listening.

    "Publisher is running" and "the autopilot is following it" are two different
    facts, and only the second one matters. This pins the second.

    It had never been observable before, because until the touchdown-detection
    fixes no flight had ever completed leg 1.
    """
    drone = FakeDrone()
    publisher = SetpointPublisher(drone, "drone-0", hz=50)

    await publisher.start()
    assert drone.offboard.started == 1
    assert drone.offboard.vehicle_mode == "OFFBOARD"

    # Leg 1 ends. Confirming the touchdown lands and disarms, which drops the
    # AUTOPILOT out of offboard -- while MAVSDK's own flag stays stale at True,
    # exactly as it does in the real client.
    drone.offboard.vehicle_mode = "LAND"
    assert await drone.offboard.is_active(), (
        "precondition: MAVSDK still claims offboard is active, which is the "
        "trap this test exists to cover"
    )

    # Leg 2 takes off and hands back over. The task is still alive, so the old
    # code returned here having done nothing.
    await publisher.start()

    assert drone.offboard.started == 2, (
        "the publisher must re-enter OFFBOARD when the autopilot has left it; "
        "otherwise leg 2 streams setpoints nobody is listening to"
    )
    assert drone.offboard.vehicle_mode == "OFFBOARD"

    await publisher.stop()


async def test_publisher_does_not_thrash_offboard_when_it_is_already_active():
    """The other half of the contract, and the reason for the is_active check.

    Re-entering on every call would be exactly the mode thrashing this class was
    written to eliminate. It must re-enter only when the autopilot has actually
    dropped out.
    """
    drone = FakeDrone()
    publisher = SetpointPublisher(drone, "drone-0", hz=50)

    await publisher.start()
    for _ in range(5):
        await publisher.start()

    assert drone.offboard.started == 1, (
        f"offboard entered {drone.offboard.started} times while it was already "
        f"active; it should have been entered once"
    )
    await publisher.stop()


async def test_re_entry_primes_setpoints_so_start_is_not_rejected():
    """MAVSDK will not send the mode request unless a setpoint came first.

    `OffboardImpl::start()` returns `NoSetpointSet` whenever its `_mode` is
    `NotActive`, and only the `set_*` methods clear that -- `start()` never does.
    A `stop()` therefore disarms the handshake, and re-entry has to re-arm it.

    The first flight attempt at re-entry did not, and lost the race against its
    own 20 Hz publisher by 35 ms:

        15:33:31,548  the autopilot is in HOLD, not OFFBOARD - re-entering
        15:33:31,563  could not re-enter OFFBOARD: No Setpoint Set

    The publisher must prime explicitly rather than hope a stream tick lands in
    the gap.
    """
    drone = FakeDrone()
    publisher = SetpointPublisher(drone, "drone-0", hz=50)

    await publisher.start()
    drone.offboard.vehicle_mode = "LAND"  # the autopilot lands and leaves offboard

    await publisher.start()

    assert drone.offboard.start_rejections == 0, (
        "re-entry called start() before pushing a setpoint, so MAVSDK rejected "
        "it with NoSetpointSet and the drone stayed in HOLD"
    )
    assert drone.offboard.vehicle_mode == "OFFBOARD"
    await publisher.stop()
