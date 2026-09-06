"""Closed-loop simulation of the landing controller against a plant model.

WHY THIS FILE EXISTS
====================
Every landing test that came before it was a pure-function check: does this gain
have this value, does this cone narrow, does this conversion invert. All of them
passed while the drone could not land. They had to, because a controller is not
a function -- it is a loop, and a loop's behaviour lives in the interaction
between the plant, the sensor and the gates, not in any one of them.

So this file simulates the whole loop. The plant is four lines of physics:

    v[k+1] = v[k] + (v_cmd - v[k]) * dt / tau       first-order velocity lag
    x[k+1] = x[k] + v[k+1] * dt                     horizontal offset
    h[k+1] = h[k] - vz_cmd * dt                     altitude
    visible = (marker fits the frame at h) and (marker is big enough at h)

That last line is the one that mattered. It is not a detail of the plant; it IS
the defect, and no amount of testing the control law in isolation could have
found it.

WHAT IS PINNED HERE
-------------------
* test_the_original_failure_reproduces  -- the old configuration hovers, exactly
  as reported, so we know the model is reproducing the real bug and not merely
  agreeing with the new code.
* test_the_shipped_configuration_lands  -- the shipped pad and gates land, from
  a spread of starting offsets and with noise.
* the individual mechanisms, each disabled in turn, so a regression names the
  guarantee it broke rather than just failing.

The plant is deliberately simple. It is not a flight-dynamics model and it will
not predict touchdown accuracy in Gazebo. It models the three things that decide
whether a landing loop converges at all -- lag, delay, and whether you can see
the target -- and that is enough to have caught this failure before flight.
"""

from __future__ import annotations

import math
import random

import pytest

import config
import world.pad_layout as pad_layout
from drone_agent import landing

# ─────────────────────────────────────────────────────────────────────────────
#  The plant
# ─────────────────────────────────────────────────────────────────────────────

TAU_V_S = 0.35
"""Time constant of PX4's horizontal velocity loop.

Consistent with MPC_ACC_HOR of a few m/s^2 against a 1.5 m/s command. The exact
figure is not critical; the tests below sweep it.
"""

GRAVITY_M_S2 = 9.81


class Plant:
    """A drone that lags its velocity command, seen by a camera that lags too."""

    def __init__(
        self,
        offset_m: float,
        altitude_m: float,
        *,
        seed: int = 0,
        tau_v_s: float = TAU_V_S,
        delay_ticks: int = 2,
        pixel_noise_px: float = 0.6,
        tilt_compensated: bool = True,
        visible_below_m: float = 0.0,
        visible_above_m: float = 1e9,
    ):
        self.x_m = offset_m  # horizontal offset, one axis is enough
        self.v_m_s = 0.0
        self.h_m = altitude_m
        self.tau_v_s = tau_v_s
        self._rng = random.Random(seed)
        self._pixel_noise_px = pixel_noise_px
        self._tilt_compensated = tilt_compensated
        self._visible_below_m = visible_below_m
        self._visible_above_m = visible_above_m
        # Transport delay: what the controller sees is what was true `delay`
        # ticks ago. This is the camera exposure, the detector, the datagram and
        # one tick of decision staleness, lumped.
        self._pipeline = [(self.x_m, 0.0)] * (delay_ticks + 1)
        self.ticks = 0

    @property
    def visible(self) -> bool:
        """Whether the pad can be measured at this altitude.

        The bound that matters is the LOW one: a marker that overflows the frame
        cannot be detected however close it is.
        """
        return self._visible_below_m <= self.h_m <= self._visible_above_m

    def measure(self) -> float | None:
        """The offset as the controller sees it, or None when the pad is not visible."""
        delayed_x, delayed_tilt_rad = self._pipeline[0]
        if not self.visible:
            return None

        # A rigidly-mounted downward camera that is not level reports the pad
        # displaced by h * tan(tilt). Compensating it is the controller's job.
        bias_m = 0.0 if self._tilt_compensated else self.h_m * math.tan(delayed_tilt_rad)
        noise_m = self._rng.gauss(0.0, self._pixel_noise_px) * self.h_m / config.CAMERA_FX_PX
        return delayed_x + bias_m + noise_m

    def step(self, v_cmd_m_s: float, vz_cmd_m_s: float, dt_s: float) -> None:
        previous_v = self.v_m_s
        # v_cmd is the velocity that CLOSES the offset, so it acts against x.
        self.v_m_s += ((-v_cmd_m_s) - self.v_m_s) * (dt_s / self.tau_v_s)
        accel_m_s2 = (self.v_m_s - previous_v) / dt_s
        tilt_rad = math.atan2(accel_m_s2, GRAVITY_M_S2)

        self.x_m += self.v_m_s * dt_s
        self.h_m = max(0.0, self.h_m - vz_cmd_m_s * dt_s)

        self._pipeline.pop(0)
        self._pipeline.append((self.x_m, tilt_rad))
        self.ticks += 1


# ─────────────────────────────────────────────────────────────────────────────
#  The controller under test
# ─────────────────────────────────────────────────────────────────────────────
#
# This mirrors the gates of landing.execute_precision_landing without its async
# plumbing, MAVSDK, UDP or FSM. Each guarantee is a named flag so a test can
# switch exactly one off and show what it was buying.

LEGACY_CLIMB_RECENTER_M_S = 0.3
"""The climb rate the shipped controller used when it drifted outside its cone.

Removed from config.py along with the behaviour -- failing the descent gate now
pauses the descent instead of climbing. Kept here as a literal because these
tests reproduce the OLD behaviour on purpose, and a test that imported a
deleted constant would be a landmine rather than a reproduction.
"""

LANDED = "landed"
HOVERING = "hovering"
CRASHED_OFF_PAD = "off_pad"


def fly(
    plant: Plant,
    *,
    search_alt_m: float = config.SEARCH_ALT_M,
    climb_on_loss: bool = False,
    no_climb_below_m: float = config.NO_CLIMB_ALT_M,
    use_ratchet: bool = True,
    gate_floor_m: float = config.DESCENT_ACCEPT_FLOOR_M,
    cone_slope: float = config.DESCENT_CONE_SLOPE,
    cone_intercept_m: float = config.DESCENT_CONE_INTERCEPT_M,
    pause_instead_of_climb: bool = True,
    commit_alt_m: float = config.COMMIT_ALT_M,
    use_commit: bool = True,
    k_p: float = config.LANDING_K_P,
    k_d: float = config.LANDING_K_D,
    max_time_s: float = 180.0,
    touchdown_alt_m: float = config.TOUCHDOWN_ALT_M,
) -> tuple[str, float, float, float]:
    """Fly the loop to a conclusion.

    Returns (outcome, elapsed_s, final_altitude_m, final_offset_m).
    """
    dt_s = 1.0 / config.NAV_HZ
    tracker = landing.AlphaBetaTracker()
    ratchet = landing.AltitudeRatchet()

    descending = False
    committed = False
    elapsed_s = 0.0
    ticks_since_seen = 0

    while elapsed_s < max_time_s:
        ratchet.observe(plant.h_m)

        if committed:
            # Zero horizontal command, mirroring the real loop: the commit hands
            # the last few metres to PX4's position hold. Steering on an
            # extrapolated estimate here ramps away from the truth and pushes
            # the drone off the pad -- see the note in landing.py.
            plant.step(0.0, config.DESCENT_VZ_FINAL_M_S, dt_s)
            elapsed_s += dt_s
            if plant.h_m <= touchdown_alt_m:
                return LANDED, elapsed_s, plant.h_m, abs(plant.x_m)
            continue

        measurement = plant.measure()
        if measurement is not None:
            ticks_since_seen = 0
            filtered = tracker.update((measurement, 0.0), dt_s)
        else:
            ticks_since_seen += 1
            if tracker.position is not None and ticks_since_seen * dt_s <= config.LOCK_LOSS_GRACE_S:
                filtered = tracker.predict(dt_s)
            else:
                tracker.reset()
                filtered = None

        if filtered is not None:
            offset_m = abs(filtered[0])

            if use_commit and plant.h_m <= commit_alt_m and offset_m <= config.COMMIT_MAX_OFFSET_M:
                committed = True
                continue

            v_cmd = 0.0
            if offset_m >= config.LANDING_DEADBAND_M:
                v_cmd = max(
                    -config.LANDING_MAX_VEL_M_S,
                    min(
                        config.LANDING_MAX_VEL_M_S,
                        k_p * filtered[0] + k_d * tracker.velocity[0],
                    ),
                )

            cone_m = cone_slope * plant.h_m + cone_intercept_m
            limit_m = max(gate_floor_m, cone_m)
            if descending:
                descending = offset_m < limit_m * config.DESCENT_EXIT_HYSTERESIS
            else:
                descending = offset_m < limit_m

            if descending:
                vz_cmd = landing.descent_rate_m_s(plant.h_m)
            elif pause_instead_of_climb:
                vz_cmd = 0.0
            else:
                vz_cmd = -LEGACY_CLIMB_RECENTER_M_S
        else:
            # Lock lost.
            v_cmd = 0.0
            descending = False
            may_climb = climb_on_loss and plant.h_m > no_climb_below_m
            if may_climb:
                # The shipped behaviour: fly back up to the search altitude.
                vz_cmd = max(-1.0, min(1.0, -0.5 * (search_alt_m - plant.h_m)))
            else:
                vz_cmd = 0.0
                if use_commit and plant.h_m <= commit_alt_m + 0.3:
                    committed = True
                    continue

        if use_ratchet:
            vz_cmd = ratchet.limit_climb(plant.h_m, vz_cmd)

        plant.step(v_cmd, vz_cmd, dt_s)
        elapsed_s += dt_s

        if plant.h_m <= touchdown_alt_m:
            return LANDED, elapsed_s, plant.h_m, abs(plant.x_m)

    return HOVERING, elapsed_s, plant.h_m, abs(plant.x_m)


# ─────────────────────────────────────────────────────────────────────────────
#  Visibility of the OLD and NEW pads, derived rather than hardcoded
# ─────────────────────────────────────────────────────────────────────────────


def _tan_half_fov() -> tuple[float, float]:
    tan_h = math.tan(config.CAMERA_HFOV_RAD / 2.0)
    tan_v = (config.CAMERA_HEIGHT_PX / 2.0) / config.CAMERA_FX_PX
    return tan_h, tan_v


LEGACY_MEASURED_FLOOR_M = 2.52
"""Altitude at which the OLD pad's marker was MEASURED to stop decoding.

Not a formula. Obtained by rendering sim/models/pad_0's original aruco_0.png
through a supersampled synthetic pinhole camera at this camera's geometry and
running the real cv2.aruco detector over the result: the last altitude that
decoded centred was 2.52 m, and 3.42 m with 8 degrees of airframe tilt. Below
1.8 m nothing decoded in any condition.

The purely geometric bound -- the altitude at which the 2.0 m pad plane spans
the 240 px frame height -- is legacy_visibility_floor_m() at 2.31 m. The
measured floor sits above it because the decoder needs the quiet zone intact,
not merely the pad's outline inside the frame. The measured number is the one
these tests fly against, because it is what actually happened.
"""


def legacy_visibility_floor_m() -> float:
    """Geometric floor for the OLD single-marker pad: the whole 2.0 m plane
    edge-to-edge in a 240 px frame. Its marker filled 0.796 of the plane and the
    remaining eighth on each side WAS the quiet zone, so the pad's outline and
    the decodable extent are the same thing here."""
    _, tan_v = _tan_half_fov()
    return pad_layout.PAD_SIZE_M / (2.0 * tan_v)


def shipped_visibility_band_m() -> tuple[float, float]:
    tan_h, tan_v = _tan_half_fov()
    return pad_layout.coverage_m(config.CAMERA_FX_PX, tan_h, tan_v)


# ═════════════════════════════════════════════════════════════════════════════
#  THE HEADLINE RESULTS
# ═════════════════════════════════════════════════════════════════════════════


def test_the_original_failure_reproduces():
    """The reported bug, reproduced in simulation.

    Old pad (marker unreadable below ~2.8 m) plus the old response to losing it
    (climb back to the search altitude) plus no commit and no ratchet. The drone
    must FAIL to land, and must fail by parking in the band where it keeps
    losing and regaining the marker -- that is the "hovers around it and moves
    down in a spiral" the operator saw.

    If this test ever starts passing as a landing, the plant model has stopped
    reproducing the real defect and every other result in this file is worthless.
    """
    geometric_m = legacy_visibility_floor_m()
    assert 2.2 < geometric_m < 2.6, (
        f"the old 2.0 m pad spans the 240 px frame at about 2.31 m, got {geometric_m:.2f} m"
    )
    assert LEGACY_MEASURED_FLOOR_M >= geometric_m, (
        "the measured decode floor cannot be below the geometric one"
    )

    outcome, _elapsed_s, altitude_m, _offset_m = fly(
        Plant(1.0, 6.0, visible_below_m=LEGACY_MEASURED_FLOOR_M),
        climb_on_loss=True,
        use_ratchet=False,
        pause_instead_of_climb=False,
        use_commit=False,
        no_climb_below_m=0.0,
    )

    assert outcome == HOVERING, "the shipped configuration is not supposed to land"
    assert altitude_m > 1.5, (
        f"it should be stuck up in the visibility band, not near the ground; "
        f"ended at {altitude_m:.2f} m"
    )


def test_the_shipped_configuration_lands():
    """The configuration this repository actually ships must land, repeatably.

    Twelve seeds, four starting offsets, with measurement noise and a transport
    delay. The threshold is every single run, not most of them: a landing that
    works 90 percent of the time is a delivery drone that strands a package
    every tenth flight.
    """
    low_m, _high_m = shipped_visibility_band_m()

    failures = []
    errors_m = []
    for seed in range(12):
        for offset_m in (0.0, 0.5, 1.0, 1.5):
            outcome, elapsed_s, _altitude_m, final_offset_m = fly(
                Plant(offset_m, 6.0, seed=seed, visible_below_m=low_m)
            )
            if outcome != LANDED:
                failures.append((seed, offset_m, outcome, elapsed_s))
            else:
                errors_m.append(final_offset_m)

    assert not failures, f"{len(failures)} of 48 runs did not land: {failures[:5]}"

    worst_m = max(errors_m)
    assert worst_m < 0.35, (
        f"worst touchdown offset {worst_m:.3f} m; the pad is "
        f"{pad_layout.PAD_SIZE_M} m across, so anything approaching 1.0 m is a miss"
    )


def test_the_pad_is_what_makes_the_difference():
    """Same controller, only the pad changed. This isolates the root cause.

    The controller in both runs is byte-for-byte the shipped one. The only
    difference is whether the pad can be seen below 2.8 m. If the run with the
    old pad lands anyway, then the pad was not the root cause and the analysis
    behind this rewrite is wrong.
    """
    low_m, _ = shipped_visibility_band_m()

    old_outcome, _, old_altitude_m, _ = fly(
        Plant(1.0, 6.0, seed=3, visible_below_m=LEGACY_MEASURED_FLOOR_M),
        use_commit=False,
    )
    new_outcome, _, _, _ = fly(Plant(1.0, 6.0, seed=3, visible_below_m=low_m))

    assert new_outcome == LANDED
    assert old_outcome == HOVERING, (
        f"with the old pad and no commit the drone should stall at the "
        f"visibility floor, but it reached {old_altitude_m:.2f} m"
    )


# ═════════════════════════════════════════════════════════════════════════════
#  EACH GUARANTEE, ISOLATED
# ═════════════════════════════════════════════════════════════════════════════


def test_climbing_on_lock_loss_is_what_creates_the_limit_cycle():
    """Turn the old climb-on-loss back on and the loop stops converging.

    This is the mechanism, stated as a test: going up restores the lock, which
    permits a descent, which loses the lock, which sends it back up. Nothing
    else about the configuration changes.
    """
    floor_m = LEGACY_MEASURED_FLOOR_M

    with_climb, _, altitude_with_m, _ = fly(
        Plant(0.5, 6.0, seed=1, visible_below_m=floor_m),
        climb_on_loss=True,
        no_climb_below_m=0.0,
        use_ratchet=False,
        use_commit=False,
    )
    without_climb, _, altitude_without_m, _ = fly(
        Plant(0.5, 6.0, seed=1, visible_below_m=floor_m),
        climb_on_loss=False,
        use_ratchet=False,
        use_commit=False,
    )

    assert with_climb == HOVERING
    assert altitude_without_m < altitude_with_m, (
        "refusing to climb should leave the drone lower than climbing does; got "
        f"{altitude_without_m:.2f} m vs {altitude_with_m:.2f} m"
    )


def test_the_commit_is_what_gets_the_last_metre():
    """Without the open-loop commit, the drone stalls where vision ends.

    Even with a perfectly visible pad and no climbing, the descent has to cross
    the altitude at which the pad leaves the frame. The commit is what carries
    it across.
    """
    low_m, _ = shipped_visibility_band_m()

    no_commit, _, altitude_m, _ = fly(
        Plant(0.4, 6.0, seed=5, visible_below_m=low_m), use_commit=False
    )
    with_commit, _, _, _ = fly(Plant(0.4, 6.0, seed=5, visible_below_m=low_m))

    assert with_commit == LANDED
    assert no_commit == HOVERING
    assert altitude_m == pytest.approx(low_m, abs=0.35), (
        f"without a commit it should stall at the visibility floor {low_m:.2f} m, "
        f"but it stopped at {altitude_m:.2f} m"
    )


def test_the_descent_gate_is_satisfiable_at_every_altitude():
    """The invariant that stops the gate deadlocking, stated directly.

    The failure mode being excluded is a gate that keeps narrowing until it
    demands better centring than the measurement can deliver, at which point the
    descent stops for good while the drone still holds a perfect lock. So: at
    every altitude from the ground to well above the search altitude, the gate
    must admit an offset comfortably larger than the measurement noise.

    The measured noise floor is 3-8 mm below 3 m and 21 mm at 8 m
    (tests/test_pad_estimator.py), so a 10x margin is 0.21 m at worst. The gate
    clears that everywhere except in the last few centimetres, where the commit
    has already taken over and vision is no longer in the loop.
    """
    worst_measurement_error_m = 0.021

    for altitude_m in [x / 20.0 for x in range(0, 201)]:
        limit_m = landing.descent_gate_limit_m(altitude_m)

        assert limit_m >= config.DESCENT_ACCEPT_FLOOR_M, (
            f"the gate must never fall below its floor; at {altitude_m:.2f} m "
            f"it was {limit_m:.4f} m"
        )
        if altitude_m >= config.COMMIT_ALT_M:
            assert limit_m > 5.0 * worst_measurement_error_m, (
                f"at {altitude_m:.2f} m -- still under closed-loop control -- the "
                f"gate admits only {limit_m:.3f} m, which is within a factor of 5 "
                f"of the measurement error. It will close on noise."
            )


def test_the_floor_is_what_saves_a_badly_chosen_cone():
    """The floor is dormant in the shipped config, and that is intentional.

    config.DESCENT_CONE_INTERCEPT_M is 0.15 m and the floor is 0.18 m, so the
    cone already exceeds the floor above 0.086 m -- the floor changes nothing
    today, and a test that claimed otherwise would be measuring nothing.

    It earns its place as a GUARD. If someone later tightens the cone toward
    zero -- a reasonable-looking change, since a tighter cone sounds like a more
    accurate landing -- the floor is what stops the gate becoming unsatisfiable.
    Without it that change would produce a drone that holds a perfect lock and
    silently refuses to descend, which is the failure this rewrite exists to
    remove and is very hard to attribute after the fact.

    So: a deliberately starved cone, with and without the floor.
    """
    shipped_limit_m = landing.descent_gate_limit_m(0.5)
    assert shipped_limit_m == pytest.approx(
        config.DESCENT_CONE_SLOPE * 0.5 + config.DESCENT_CONE_INTERCEPT_M
    ), "with the shipped constants the cone, not the floor, sets the limit"

    # A cone that narrows far too fast: 0.06 m of tolerance at 6 m, which is
    # below the 0.065 m of measurement noise a 3 px error implies up there.
    starved_cone = dict(cone_slope=0.01, cone_intercept_m=0.0)

    without_floor, _, altitude_without_m, _ = fly(
        Plant(0.6, 6.0, seed=2, pixel_noise_px=3.0, visible_below_m=0.0),
        gate_floor_m=0.0,
        use_commit=False,
        commit_alt_m=0.0,
        touchdown_alt_m=0.05,
        **starved_cone,
    )
    with_floor, _, altitude_with_m, _ = fly(
        Plant(0.6, 6.0, seed=2, pixel_noise_px=3.0, visible_below_m=0.0),
        gate_floor_m=config.DESCENT_ACCEPT_FLOOR_M,
        use_commit=False,
        commit_alt_m=0.0,
        touchdown_alt_m=0.05,
        **starved_cone,
    )

    assert without_floor == HOVERING, (
        f"a cone this tight should deadlock the descent, but the drone reached "
        f"{altitude_without_m:.2f} m"
    )
    assert altitude_with_m < altitude_without_m, (
        "the floor should rescue the descent from a starved cone; got "
        f"{altitude_with_m:.2f} m with a floor vs {altitude_without_m:.2f} m without"
    )


def test_the_ratchet_forbids_returning_to_altitude():
    """The last line of defence, tested on its own.

    Even with climb-on-loss deliberately enabled and the no-climb altitude
    disabled, the ratchet must stop the drone from walking back up to the search
    altitude. It is what makes a landing attempt monotonic whatever else fails.
    """
    floor_m = LEGACY_MEASURED_FLOOR_M

    _outcome, _elapsed, altitude_m, _offset = fly(
        Plant(0.5, 6.0, seed=4, visible_below_m=floor_m),
        climb_on_loss=True,
        no_climb_below_m=0.0,
        use_ratchet=True,
        use_commit=False,
    )

    ceiling_m = floor_m + config.ALTITUDE_FLOOR_HYSTERESIS_M
    assert altitude_m <= ceiling_m + 0.3, (
        f"the ratchet should have pinned the drone near its lowest reached "
        f"altitude (about {ceiling_m:.2f} m), but it climbed to {altitude_m:.2f} m"
    )


def test_the_loop_is_stable_across_a_range_of_plants():
    """The gains must not be tuned to one exact plant.

    A real vehicle is heavier, slower or laggier than the model. Sweeping the
    velocity time constant and the transport delay well past their nominal
    values shows how much margin the gains actually have.
    """
    low_m, _ = shipped_visibility_band_m()

    failures = []
    for tau_v_s in (0.2, 0.35, 0.6, 0.9):
        for delay_ticks in (1, 2, 4):
            outcome, _elapsed, _altitude, offset_m = fly(
                Plant(
                    1.0,
                    6.0,
                    seed=11,
                    tau_v_s=tau_v_s,
                    delay_ticks=delay_ticks,
                    visible_below_m=low_m,
                )
            )
            if outcome != LANDED or offset_m > 0.5:
                failures.append((tau_v_s, delay_ticks, outcome, round(offset_m, 3)))

    assert not failures, f"unstable outside the nominal plant: {failures}"


def test_tilt_compensation_matters_most_at_altitude():
    """An uncompensated tilt bias is a real error, and the loop survives it.

    The bias is h * tan(tilt), so it is large high up and small near the ground
    -- which is why it degrades the approach rather than the touchdown. The
    controller must still land without the correction, because attitude
    telemetry can be absent; it should simply do it less tidily.
    """
    low_m, _ = shipped_visibility_band_m()

    uncompensated, _, _, _ = fly(
        Plant(1.2, 6.0, seed=8, tilt_compensated=False, visible_below_m=low_m)
    )
    compensated, _, _, _ = fly(
        Plant(1.2, 6.0, seed=8, tilt_compensated=True, visible_below_m=low_m)
    )

    assert compensated == LANDED
    assert uncompensated == LANDED, (
        "tilt compensation is an improvement, not a dependency: a drone with no "
        "attitude telemetry must still land"
    )


def test_a_dropped_frame_does_not_abort_the_descent():
    """Intermittent detection must be survivable.

    Half the frames are dropped at random. The grace period plus the tracker's
    velocity state should carry the loop through; the shipped code treated a
    single missed frame as a lost lock.
    """
    low_m, _ = shipped_visibility_band_m()

    class Flaky(Plant):
        def measure(self):
            if self._rng.random() < 0.5:
                return None
            return super().measure()

    outcome, _elapsed, _altitude, offset_m = fly(Flaky(0.8, 6.0, seed=6, visible_below_m=low_m))
    assert outcome == LANDED, "a 50 percent frame drop rate must not prevent a landing"
    assert offset_m < 0.5
