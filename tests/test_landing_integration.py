"""Drive the REAL execute_precision_landing() against fake hardware.

WHY THIS EXISTS ALONGSIDE test_landing_closed_loop.py
=====================================================
That file simulates the control law to prove the loop converges. It deliberately
reimplements the gates so each one can be switched off in isolation -- which
means it does NOT execute a single line of execute_precision_landing().

So the async function itself, with its phase transitions, its FSM events, its
staleness checks and its touchdown handshake, was untested. That is exactly the
category of code the original defect lived in: not the arithmetic, but what the
loop does when the arithmetic says the pad is gone.

Here the real coroutine runs against a fake publisher, a fake MAVSDK drone and a
vision dict driven by a plant model, with asyncio.sleep patched so a two-minute
flight takes milliseconds.
"""

from __future__ import annotations

import asyncio
import math

import pytest

import config
from drone_agent import landing, udp_receiver

pytestmark = pytest.mark.asyncio


# ─────────────────────────────────────────────────────────────────────────────
#  Fakes
# ─────────────────────────────────────────────────────────────────────────────


class FakeSetpoint:
    def __init__(self):
        self.north_m_s = 0.0
        self.east_m_s = 0.0
        self.down_m_s = 0.0
        self.yaw_deg = 0.0

    def hold(self, yaw_deg=None):
        self.north_m_s = self.east_m_s = self.down_m_s = 0.0
        if yaw_deg is not None:
            self.yaw_deg = yaw_deg


class FakePublisher:
    """Records commands. The real one streams them to PX4; nothing else differs."""

    def __init__(self):
        self.setpoint = FakeSetpoint()
        self.commands: list[tuple[str, float, float, float]] = []
        self.ensure_offboard_calls = 0

    def command_body_horizontal(self, forward_m_s, right_m_s, down_m_s, heading_deg):
        heading_rad = math.radians(heading_deg)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
        self.setpoint.north_m_s = forward_m_s * cos_h - right_m_s * sin_h
        self.setpoint.east_m_s = forward_m_s * sin_h + right_m_s * cos_h
        self.setpoint.down_m_s = down_m_s
        self.setpoint.yaw_deg = heading_deg
        self.commands.append(("body", forward_m_s, right_m_s, down_m_s))

    def command_ned(self, north_m_s, east_m_s, down_m_s, yaw_deg):
        self.setpoint.north_m_s = north_m_s
        self.setpoint.east_m_s = east_m_s
        self.setpoint.down_m_s = down_m_s
        self.setpoint.yaw_deg = yaw_deg
        self.commands.append(("ned", north_m_s, east_m_s, down_m_s))

    async def ensure_offboard(self) -> None:
        """The real publisher re-enters OFFBOARD here when the autopilot has
        left it (a land command does exactly that). Counted so a test can show
        the loop repairs the mode instead of streaming into a dead one."""
        self.ensure_offboard_calls += 1

    def hold(self, yaw_deg=None):
        self.setpoint.hold(yaw_deg)
        self.commands.append(("hold", 0.0, 0.0, 0.0))

    @property
    def vertical_commands(self) -> list[float]:
        return [c[3] for c in self.commands if c[0] in ("body", "ned")]


class FakeAction:
    def __init__(self, owner):
        self._owner = owner

    async def land(self):
        self._owner.land_calls += 1


class FakeTelemetry:
    def __init__(self, owner):
        self._owner = owner

    async def armed(self):
        # Disarms once land() has been issued, which is what the real autopilot
        # does after its land detector latches -- IF it latches. A drone built
        # with disarms_on_land=False never does, which is how the failure path
        # of _confirm_landed gets exercised at all.
        for _ in range(50):
            if self._owner.disarms_on_land:
                yield self._owner.land_calls == 0
            else:
                yield True
            await asyncio.sleep(0)


class FakeDrone:
    def __init__(self, disarms_on_land: bool = True):
        self.land_calls = 0
        # Whether a land command actually ends in a disarm. False models a real
        # and dangerous case: the autopilot accepts the command, leaves OFFBOARD
        # to execute it, and then does not finish -- caught on an obstruction,
        # or with a land detector that will not latch.
        self.disarms_on_land = disarms_on_land
        self.action = FakeAction(self)
        self.telemetry = FakeTelemetry(self)


class FakeFSM:
    """Records fired events. State is whatever the test wants it to be."""

    class _State:
        def __init__(self, value):
            self.value = value

    def __init__(self, state="APPROACH"):
        self.state = self._State(state)
        self.events: list[str] = []

    def fire(self, event: str) -> bool:
        self.events.append(event)
        return True


class World:
    """A plant plus a camera, wired into the dicts the landing loop reads.

    Advanced one tick per patched asyncio.sleep, so the loop's own pacing drives
    the simulation and nothing has to be scheduled by hand.
    """

    def __init__(
        self,
        publisher: FakePublisher,
        *,
        offset_m: float = 0.8,
        altitude_m: float = 5.0,
        visible_below_m: float = None,
        tau_v_s: float = 0.35,
        heading_deg: float = 0.0,
        land_detector_latches: bool = True,
    ):
        self.publisher = publisher
        # Whether PX4's land detector ever reports ON_GROUND.
        #
        # The default, True, is the friendly case. Setting it False reproduces
        # what a REAL PX4 does while the loop streams a downward velocity
        # setpoint in OFFBOARD: the land detector wants sustained low thrust and
        # near-zero velocity, is being commanded to fly, and so never latches --
        # LandedState stays IN_AIR through touchdown and beyond.
        self.land_detector_latches = land_detector_latches
        self.forward_m = offset_m  # pad offset in the BODY frame
        self.right_m = 0.0
        self.v_forward = 0.0
        self.v_right = 0.0
        self.altitude_m = altitude_m
        self.tau_v_s = tau_v_s
        self.heading_deg = heading_deg
        self.visible_below_m = (
            config.PAD_VISION_MIN_ALT_M if visible_below_m is None else visible_below_m
        )
        self.ticks = 0
        self.now = 1000.0

        self.drone_state = {
            "lat": 30.0315,
            "lon": 72.3140,
            "alt": altitude_m,
            "landed_state": "IN_AIR",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "heading_deg": heading_deg,
            udp_receiver.TELEMETRY_TS_KEY: 1000.0,
        }
        self.vision_data = {"pads": [], "detections": [], udp_receiver.VISION_TS_KEY: 1000.0}
        self._publish_vision()

    # ── the plant ────────────────────────────────────────────────────────────

    def tick(self, dt_s: float = 0.1) -> None:
        setpoint = self.publisher.setpoint

        # Rotate the commanded NED velocity back into the body frame.
        heading_rad = math.radians(self.heading_deg)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
        cmd_forward = setpoint.north_m_s * cos_h + setpoint.east_m_s * sin_h
        cmd_right = -setpoint.north_m_s * sin_h + setpoint.east_m_s * cos_h

        self.v_forward += (cmd_forward - self.v_forward) * (dt_s / self.tau_v_s)
        self.v_right += (cmd_right - self.v_right) * (dt_s / self.tau_v_s)

        # Flying forward reduces a positive forward offset to the pad.
        self.forward_m -= self.v_forward * dt_s
        self.right_m -= self.v_right * dt_s

        self.altitude_m = max(0.0, self.altitude_m - setpoint.down_m_s * dt_s)
        self.drone_state["alt"] = self.altitude_m
        if self.altitude_m <= 0.05 and self.land_detector_latches:
            self.drone_state["landed_state"] = "ON_GROUND"

        self.ticks += 1
        self.now += dt_s
        self.drone_state[udp_receiver.TELEMETRY_TS_KEY] = self.now
        self._publish_vision()

    # ── the camera ───────────────────────────────────────────────────────────

    def _publish_vision(self) -> None:
        self.vision_data[udp_receiver.VISION_TS_KEY] = self.now
        if self.altitude_m < self.visible_below_m:
            self.vision_data["pads"] = []
            return

        px_per_m = config.CAMERA_FX_PX / max(self.altitude_m, 0.05)
        # Invert landing.body_offsets_m: forward = -down_m, right = +right_m.
        err_u = self.right_m * px_per_m
        err_v = -self.forward_m * px_per_m
        self.vision_data["pads"] = [
            {
                "pad_id": 0,
                "err_x": err_u,
                "err_y": err_v,
                "px_per_m": px_per_m,
                "range_m": self.altitude_m,
                "residual_m": 0.001,
                "markers": [0],
                "largest_side_px": 40.0,
                "clipped": False,
            }
        ]

    @property
    def offset_m(self) -> float:
        return math.hypot(self.forward_m, self.right_m)


async def run_landing(
    world: World, monkeypatch, *, fsm=None, drone=None, max_ticks: int = 6000, **kwargs
):
    """Run the real coroutine, advancing the world on every sleep."""
    real_sleep = asyncio.sleep

    async def fake_sleep(delay, *a, **kw):
        world.tick(delay if delay else 0.1)
        if world.ticks > max_ticks:
            raise TimeoutError("landing did not terminate")
        await real_sleep(0)

    monkeypatch.setattr(landing.asyncio, "sleep", fake_sleep)
    monkeypatch.setattr(landing.time, "monotonic", lambda: world.now)

    return await landing.execute_precision_landing(
        world.publisher,
        drone if drone is not None else FakeDrone(),
        0,
        world.drone_state["lat"],
        world.drone_state["lon"],
        world.vision_data,
        world.drone_state,
        drone_id="drone-0",
        fsm=fsm,
        **kwargs,
    )


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS
# ═════════════════════════════════════════════════════════════════════════════


async def test_the_real_coroutine_lands(monkeypatch):
    """End to end, through the actual async function."""
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.8, altitude_m=5.0)
    fsm = FakeFSM()

    outcome = await run_landing(world, monkeypatch, fsm=fsm)

    assert outcome == landing.LandingOutcome.TOUCHDOWN, (
        f"ended as {outcome} at {world.altitude_m:.2f} m, {world.offset_m:.2f} m off"
    )
    assert world.offset_m < 0.5, f"landed {world.offset_m:.2f} m off the pad centre"
    assert "marker_locked" in fsm.events
    assert "touchdown" in fsm.events


async def test_it_never_climbs_once_it_is_low(monkeypatch):
    """The guarantee that fixes the reported hover, on the real code path.

    The pad goes invisible below the vision floor, which is exactly the
    condition that used to trigger a climb back to the search altitude.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.5, altitude_m=4.0)

    await run_landing(world, monkeypatch)

    # A negative down_m_s is a climb. None may be commanded below NO_CLIMB_ALT_M.
    climbs = [vz for vz in publisher.vertical_commands if vz < -1e-9]
    assert not climbs, (
        f"the loop commanded {len(climbs)} climb(s) during a landing from 4 m, "
        f"steepest {min(climbs):.2f} m/s. Below config.NO_CLIMB_ALT_M "
        f"({config.NO_CLIMB_ALT_M} m) that is forbidden -- it is the limit cycle "
        f"this design exists to prevent."
    )


async def test_it_commits_when_the_pad_leaves_the_frame(monkeypatch):
    """Losing the pad low down must lead to a landing, not a hold.

    This is the specific sequence that used to loop forever: descend, lose the
    pad because it no longer fits the frame, and decide what to do about it.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.3, altitude_m=3.0)

    outcome = await run_landing(world, monkeypatch)

    assert outcome == landing.LandingOutcome.TOUCHDOWN
    assert world.drone_state["landing_phase"] in (
        landing.LandingPhase.COMMIT,
        landing.LandingPhase.TOUCHDOWN,
    )


async def test_a_dead_vision_feed_holds_instead_of_descending_blind(monkeypatch):
    """Fail closed. A silent camera must stop the descent, not continue it.

    And it must HOLD altitude rather than climb: a dead camera is a reason to
    stop, not a reason to undo the descent already achieved.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.4, altitude_m=4.0)

    # Freeze the vision timestamp so is_stale() fires from the first tick.
    original_publish = world._publish_vision

    def stale_publish():
        original_publish()
        world.vision_data[udp_receiver.VISION_TS_KEY] = 0.0

    world._publish_vision = stale_publish
    world.vision_data[udp_receiver.VISION_TS_KEY] = 0.0

    outcome = await run_landing(world, monkeypatch, max_ticks=3000)

    assert outcome == landing.LandingOutcome.TIMEOUT, (
        f"a dead feed should end in a timed-out hold, not {outcome}"
    )
    assert world.altitude_m > 3.0, (
        f"it must not descend on a dead feed; fell to {world.altitude_m:.2f} m"
    )
    assert world.drone_state["landing_phase"] == landing.LandingPhase.HOLD


async def test_a_dropped_frame_does_not_reset_the_lock(monkeypatch):
    """One missed detection is normal at 10 Hz and must be survivable.

    The original loop treated any frame without the marker as a lost lock and
    fell straight into the search branch, so ordinary detector jitter was enough
    to trigger the climb.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.5, altitude_m=4.0)
    fsm = FakeFSM()

    original_publish = world._publish_vision

    def flaky_publish():
        original_publish()
        if world.ticks % 3 == 0:  # drop a third of all frames
            world.vision_data["pads"] = []

    world._publish_vision = flaky_publish

    outcome = await run_landing(world, monkeypatch, fsm=fsm)

    assert outcome == landing.LandingOutcome.TOUCHDOWN
    assert fsm.events.count("lock_lost") <= 1, (
        f"a third of frames dropped should not repeatedly break the lock; "
        f"lock_lost fired {fsm.events.count('lock_lost')} times"
    )


async def test_the_wrong_pad_is_ignored(monkeypatch):
    """Three pads stand within metres of each other.

    A drone told to land on pad 0 must not be steered by pad 1's offset, however
    prominent pad 1 is in the frame.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.4, altitude_m=4.0)

    original_publish = world._publish_vision

    def decoy_publish():
        original_publish()
        decoy = {
            "pad_id": 1,
            "err_x": 140.0,
            "err_y": -120.0,
            "px_per_m": config.CAMERA_FX_PX / max(world.altitude_m, 0.05),
            "range_m": world.altitude_m,
            "residual_m": 0.001,
            "markers": [1],
            "largest_side_px": 60.0,
            "clipped": False,
        }
        world.vision_data["pads"] = [decoy, *world.vision_data["pads"]]

    world._publish_vision = decoy_publish

    outcome = await run_landing(world, monkeypatch)

    assert outcome == landing.LandingOutcome.TOUCHDOWN
    assert world.offset_m < 0.5, (
        f"the decoy pad pulled the drone {world.offset_m:.2f} m off pad 0's centre"
    )


async def test_a_frame_whose_markers_disagree_is_not_steered_on(monkeypatch):
    """A high residual means at least one contributing marker is wrong."""
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.4, altitude_m=4.0)

    original_publish = world._publish_vision

    def corrupt_publish():
        original_publish()
        for pad in world.vision_data["pads"]:
            if world.ticks % 4 == 0:
                pad["err_x"] += 400.0  # a wildly wrong estimate...
                pad["residual_m"] = config.MAX_MEASUREMENT_RESIDUAL_M * 5  # ...that says so

    world._publish_vision = corrupt_publish

    outcome = await run_landing(world, monkeypatch)

    assert outcome == landing.LandingOutcome.TOUCHDOWN
    assert world.offset_m < 0.5, (
        f"frames flagged as inconsistent still moved the drone {world.offset_m:.2f} m off"
    )


async def test_it_lands_from_a_non_zero_heading(monkeypatch):
    """The body-to-NED rotation must use the drone's TRUE heading.

    The previous code read back its own last COMMANDED yaw, so any difference
    between commanded and actual heading rotated every correction by that error.
    A drone pointing 135 degrees must still fly to its pad.
    """
    for heading_deg in (0.0, 90.0, 135.0, 250.0):
        publisher = FakePublisher()
        world = World(publisher, offset_m=0.8, altitude_m=4.0, heading_deg=heading_deg)
        publisher.setpoint.yaw_deg = heading_deg

        outcome = await run_landing(world, monkeypatch)

        assert outcome == landing.LandingOutcome.TOUCHDOWN, (
            f"heading {heading_deg} deg ended as {outcome}"
        )
        assert world.offset_m < 0.5, (
            f"at heading {heading_deg} deg the drone landed {world.offset_m:.2f} m off"
        )


async def test_the_search_runs_when_the_pad_is_nowhere_to_be_seen(monkeypatch):
    """No pad at all must end in a bounded, reported failure -- not a hover.

    A drone that has failed to find its pad should stop burning the battery its
    return leg needs.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.0, altitude_m=6.0)
    world._publish_vision = lambda: world.vision_data.update(
        {"pads": [], udp_receiver.VISION_TS_KEY: world.now}
    )
    world.vision_data["pads"] = []
    fsm = FakeFSM(state="SEARCHING")

    outcome = await run_landing(world, monkeypatch, fsm=fsm, max_ticks=6000)

    assert outcome in (
        landing.LandingOutcome.SEARCH_EXHAUSTED,
        landing.LandingOutcome.TIMEOUT,
    ), f"a missing pad should terminate the attempt, got {outcome}"
    assert "search_exhausted" in fsm.events


async def test_it_lands_even_when_the_autopilot_never_reports_on_ground(monkeypatch):
    """The regression a live SITL flight found, pinned here.

    THE FLIGHT THAT FAILED. Everything up to touchdown went well:

        14:56:20  pad 0 locked at 6.97 m via markers [10, 11, 12, 13]
        14:56:20  descending: 1.659 m off at 6.97 m (gate 2.591 m)
        14:56:27  committing to the landing from 1.17 m, 0.042 m off centre
        14:56:39  commit did not reach the ground within 12 s (still -0.03 m up)

    Gazebo ground truth put the drone stopped on the pad, 0.12 m from its
    centre. The controller reported STALLED, the leg failed, and the whole
    delivery was recorded FAILED -- because LandedState was still IN_AIR and
    is_on_ground let that veto a height reading of -0.03 m.

    WHY THE SUITE MISSED IT. Every existing test used a plant whose land
    detector latches ON_GROUND at 0.05 m. That fake was kinder than the real
    autopilot: PX4 will not declare a landing while OFFBOARD is streaming it a
    0.2 m/s downward velocity setpoint, which is exactly what the commit does
    all the way to the ground. A fake that is more generous than the hardware
    cannot fail the way the hardware fails.

    So this plant never reports ON_GROUND at all, and the loop must still land.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.6, altitude_m=5.0, land_detector_latches=False)
    fsm = FakeFSM()

    outcome = await run_landing(world, monkeypatch, fsm=fsm)

    assert outcome == landing.LandingOutcome.TOUCHDOWN, (
        f"a landing must not depend on PX4 latching ON_GROUND, got {outcome} "
        f"at {world.altitude_m:.2f} m, {world.offset_m:.2f} m off"
    )
    assert world.altitude_m <= config.TOUCHDOWN_ALT_M, (
        f"it should be on the ground, not {world.altitude_m:.2f} m up"
    )
    assert world.offset_m < 0.5, f"landed {world.offset_m:.2f} m off the pad centre"
    assert "touchdown" in fsm.events, f"FSM events fired: {fsm.events}"


async def test_a_low_lock_loss_leaves_the_real_fsm_able_to_accept_a_touchdown(monkeypatch):
    """Run the loop against the REAL MissionFSM, which enforces its transitions.

    THE FLIGHT THAT EXPOSED THIS. The landing worked and the state machine
    reporting on it did not:

        15:16:47  pad 0 locked at 6.73 m ; SEARCHING --[marker_locked]--> APPROACH
        15:16:55  no lock at 1.45 m but already below the commit altitude -
                  committing on the last known trim
        15:17:13  disarmed - landing confirmed
        15:17:13  event 'touchdown' is not valid in SEARCHING
        15:17:13  event 'op_confirmed' is not valid in SEARCHING
        15:17:16  event 'more_legs' is not valid in SEARCHING

    The drone had landed and taken its cargo. The FSM had been driven back to
    SEARCHING on the way down and rejected every event for the rest of the
    mission, so mission state stopped describing the mission.

    THE CAUSE. The loop fired `lock_lost` on every lock loss. Below
    NO_CLIMB_ALT_M a lock loss is not a setback -- it is the field of view doing
    what it must as the drone closes on the pad -- and the loop answers it by
    holding and committing, never by climbing away to search. Announcing
    "searching again" while committing to a landing contradicted the
    controller's own design, and SEARCHING has no `touchdown` transition.

    WHY FakeFSM COULD NOT CATCH IT. FakeFSM records events and returns True for
    all of them. Only the real transition table can tell a legal event from an
    illegal one, so this test uses it.
    """
    from drone_agent.mission_fsm import MissionFSM, MissionState

    fsm = MissionFSM(MissionState.SEARCHING, label="drone-0")
    publisher = FakePublisher()
    # The pad disappears well ABOVE the commit altitude, which forces the loop
    # through the lock-loss branch before it commits -- the flight's sequence.
    world = World(publisher, offset_m=0.4, altitude_m=4.0, visible_below_m=1.6)

    outcome = await run_landing(world, monkeypatch, fsm=fsm)

    assert outcome == landing.LandingOutcome.TOUCHDOWN, (
        f"ended as {outcome} at {world.altitude_m:.2f} m"
    )
    assert fsm.state == MissionState.PAYLOAD_OP, (
        f"the FSM must be able to accept the touchdown it just achieved, but it "
        f"is in {fsm.state.value}. History: {fsm.history}"
    )
    # And the payload op that follows must be legal from there.
    assert fsm.fire("op_confirmed"), "the mission cannot continue from this state"


async def test_a_land_command_that_does_not_disarm_puts_offboard_back(monkeypatch):
    """The failure path of _confirm_landed, which nothing used to exercise.

    `_confirm_landed` commands a land and waits for a real disarm. The land
    command takes the vehicle OUT of OFFBOARD whether or not the disarm ever
    arrives -- so when it does not arrive, every setpoint the loop publishes
    afterwards goes into a mode that is not listening. The loop would sit there
    logging that it was "continuing to push down" while commanding nothing at
    all, until the commit timeout.

    This was found by auditing the doubles rather than by a flight: the old
    FakeTelemetry disarmed the instant land() was called, so `_confirm_landed`
    could only ever succeed and its failure branch was dead code as far as the
    suite was concerned.
    """
    publisher = FakePublisher()
    world = World(publisher, offset_m=0.3, altitude_m=3.0)
    drone = FakeDrone(disarms_on_land=False)

    outcome = await run_landing(world, monkeypatch, drone=drone)

    assert drone.land_calls >= 1, "it should have tried to land"
    assert publisher.ensure_offboard_calls >= 1, (
        "after a land command that did not disarm, the loop must put the vehicle "
        "back into OFFBOARD; otherwise the descent it reports is a fiction"
    )
    assert outcome == landing.LandingOutcome.STALLED, (
        f"an autopilot that never disarms must be reported, not papered over; got {outcome}"
    )
