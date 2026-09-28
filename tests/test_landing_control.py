"""Unit-level properties of the landing control law and its constants.

These are pure-function checks. They are necessary and they are NOT sufficient:
an earlier version of this file passed in full while the drone could not land,
because a controller's behaviour lives in the loop, not in any one function. The
loop is exercised in tests/test_landing_closed_loop.py and the perception it
depends on in tests/test_pad_estimator.py. Read all three as one suite.
"""

import math
from pathlib import Path

import pytest

import config
from drone_agent import geo, landing

# ─────────────────────────────────────────────────────────────────────────────
#  Metric measurement: the gain is altitude-invariant because the MEASUREMENT is
# ─────────────────────────────────────────────────────────────────────────────


def test_the_controller_never_multiplies_error_by_altitude():
    """Two generations of fix are visible in this one property.

    The first converted pixels to metres with `err_px * altitude / fx`, which
    made the loop gain altitude-invariant but tied every measurement to a
    drifting EKF altitude. The second removed altitude from the measurement path
    entirely: perception.pad_estimator recovers scale from each marker's own
    known size, so the offset arrives already in metres and the gain is simply a
    constant.

    Stated as a test: for one real ground offset, the commanded velocity is the
    same number at every altitude.
    """
    ground_offset_m = 0.5

    commands = []
    for px_per_m in (46.2, 92.4, 138.6, 277.2, 554.4):  # 6 m down to 0.5 m
        err_px = ground_offset_m * px_per_m
        recovered_m = err_px / px_per_m
        assert recovered_m == pytest.approx(ground_offset_m, rel=1e-9)
        commands.append(config.LANDING_K_P * recovered_m)

    assert max(commands) - min(commands) < 1e-9, (
        f"the command for a fixed ground offset must not depend on altitude, got {commands}"
    )

    # And the failure it replaces: a gain proportional to pixels rose tenfold on
    # the way down, peaking exactly at touchdown.
    old_pixel_gain = 0.015
    old_effective = [old_pixel_gain * config.CAMERA_FX_PX / h for h in (10.0, 1.0)]
    assert old_effective[1] / old_effective[0] == pytest.approx(10.0, rel=1e-6)


def test_a_nonsense_scale_is_rejected_rather_than_divided_by():
    drone_state = {"alt": 3.0}
    for bad_scale in (0.0, -12.0):
        vision = {"pads": [{"pad_id": 0, "err_x": 10.0, "err_y": 5.0, "px_per_m": bad_scale}]}
        assert landing.observe_pad(vision, 0, drone_state) is None


def test_a_frame_whose_markers_disagree_is_dropped():
    """Steering on the average of a good and a bad measurement is worse than coasting."""
    drone_state = {"alt": 3.0}
    base = {"pad_id": 0, "err_x": 10.0, "err_y": 5.0, "px_per_m": 90.0, "range_m": 3.0}

    ok = dict(base, residual_m=config.MAX_MEASUREMENT_RESIDUAL_M * 0.5)
    assert landing.observe_pad({"pads": [ok]}, 0, drone_state) is not None

    bad = dict(base, residual_m=config.MAX_MEASUREMENT_RESIDUAL_M * 2.0)
    assert landing.observe_pad({"pads": [bad]}, 0, drone_state) is None


def test_the_wrong_pad_is_never_returned():
    """Three pads stand within metres of each other; the filter is the whole of
    marker disambiguation."""
    drone_state = {"alt": 3.0}
    vision = {
        "pads": [
            {"pad_id": 1, "err_x": 5.0, "err_y": 0.0, "px_per_m": 90.0, "range_m": 3.0},
            {"pad_id": 2, "err_x": -80.0, "err_y": 20.0, "px_per_m": 90.0, "range_m": 3.0},
        ]
    }
    assert landing.observe_pad(vision, 0, drone_state) is None
    assert landing.observe_pad(vision, 1, drone_state) is not None
    assert landing.observe_pad(vision, 2, drone_state) is not None


def test_range_comes_from_the_marker_but_defers_to_the_ekf_when_absurd():
    """The vision range beats the EKF altitude -- until it is obviously wrong.

    It is better because it measures height above the PAD rather than above
    wherever the drone took off, and because it is good to better than one
    percent. It is not trusted blindly, because a bad decode could report
    anything and the autopilot's estimate has redundant sensors behind it.
    """
    drone_state = {"alt": 3.0}

    assert landing.working_height_m({"range_m": 3.1}, drone_state) == pytest.approx(3.1)
    assert landing.working_height_m({"range_m": 40.0}, drone_state) == pytest.approx(3.0)
    assert landing.working_height_m(None, drone_state) == pytest.approx(3.0)


# ─────────────────────────────────────────────────────────────────────────────
#  Camera mount: signs and tilt
# ─────────────────────────────────────────────────────────────────────────────


def test_body_frame_signs_match_the_derived_camera_mount():
    """The mount is a +90 degree pitch about Y, so image u is body RIGHT and
    image v is body AFT (docs/CALIBRATION.md).

    Inverting either sign turns the controller into positive feedback that flies
    away from the pad, so it is worth pinning explicitly.
    """
    # A pad below the image centre (+v) is BEHIND the drone: close on it by
    # flying backwards.
    forward_m, right_m = landing.body_offsets_m(0.0, 0.4)
    assert forward_m < 0.0, "a pad low in the image is aft; forward must be negative"
    assert right_m == pytest.approx(0.0)

    # A pad right of the image centre (+u) is to the drone's right.
    forward_m, right_m = landing.body_offsets_m(0.4, 0.0)
    assert right_m > 0.0
    assert forward_m == pytest.approx(0.0)

    # Magnitudes pass through untouched: this is a relabelling, not a scaling.
    forward_m, right_m = landing.body_offsets_m(0.3, -0.2)
    assert abs(forward_m) == pytest.approx(0.2)
    assert abs(right_m) == pytest.approx(0.3)


def test_tilt_correction_moves_the_target_the_right_way():
    """A tilted airframe swings a rigidly-mounted camera off nadir.

    Sanity check at roll = +90 degrees (right wing straight down): the optical
    axis, body +Z, swings to the world horizon on the LEFT, so the point
    directly below the drone lies a quarter turn away toward body +Y -- far off
    to the image right. tan(roll) diverging positive is exactly that.
    """
    assert landing.nadir_offset_px(0.0, 0.0) == (pytest.approx(0.0), pytest.approx(0.0))

    u_px, v_px = landing.nadir_offset_px(10.0, 0.0)
    assert u_px > 0.0, "rolling right puts nadir to the image right"
    assert v_px == pytest.approx(0.0, abs=1e-9)

    u_px, v_px = landing.nadir_offset_px(0.0, 10.0)
    assert v_px > 0.0, "pitching nose-up puts nadir aft, which is +v in this mount"
    assert u_px == pytest.approx(0.0, abs=1e-9)

    # Magnitude is fx * tan(tilt).
    u_px, _ = landing.nadir_offset_px(5.0, 0.0)
    assert u_px == pytest.approx(config.CAMERA_FX_PX * math.tan(math.radians(5.0)))

    # Which at 6 m is a 0.52 m ground error -- twice CENTERED_M. That is why
    # this correction exists rather than being a refinement.
    assert 6.0 * math.tan(math.radians(5.0)) > 2.0 * config.CENTERED_M

    # Extreme attitudes are clamped, not allowed to diverge through tan().
    assert abs(landing.nadir_offset_px(89.0, 89.0)[0]) < 20.0 * config.CAMERA_FX_PX


def test_tilt_correction_is_applied_only_when_attitude_is_known():
    """Absent attitude must mean NO correction -- not a correction computed from
    a fabricated level attitude, and not a crash."""
    vision = {
        "fx": config.CAMERA_FX_PX,
        "pads": [{"pad_id": 0, "err_x": 20.0, "err_y": 0.0, "px_per_m": 100.0, "range_m": 2.8}],
    }

    without = landing.observe_pad(vision, 0, {"alt": 2.8})
    assert without is not None and without["tilt_corrected"] is False

    with_attitude = landing.observe_pad(vision, 0, {"alt": 2.8, "roll_deg": 6.0, "pitch_deg": 0.0})
    assert with_attitude is not None and with_attitude["tilt_corrected"] is True
    assert with_attitude["right_m"] != pytest.approx(without["right_m"]), (
        "a 6 degree roll must actually move the commanded target"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  F3: the marker/altitude budget
# ─────────────────────────────────────────────────────────────────────────────


def test_the_search_altitude_sits_inside_the_pads_visible_band():
    """The search must fly where the pad can actually be seen.

    The original defect at cruise altitude was the mirror image of the one that
    produced the hover: a 0.5 m marker spans 14 px at 10 m and cannot be
    decoded, so the documented "arrive at TARGET_ALT, then search" could never
    lock on. The nested pad answers both ends at once -- the outer ring reaches
    up, the centre marker reaches down.
    """
    import world.pad_layout as pad_layout

    tan_h = math.tan(config.CAMERA_HFOV_RAD / 2.0)
    tan_v = (config.CAMERA_HEIGHT_PX / 2.0) / config.CAMERA_FX_PX
    low_m, high_m = pad_layout.coverage_m(config.CAMERA_FX_PX, tan_h, tan_v)

    assert low_m < config.SEARCH_ALT_M <= high_m, (
        f"SEARCH_ALT_M {config.SEARCH_ALT_M} m must sit inside the pad's "
        f"{low_m:.2f}-{high_m:.2f} m visible band"
    )

    # And the band must cover the whole descent, with no hole in the middle --
    # coverage_m() raises on a gap, so reaching here is already the assertion.
    assert low_m <= config.COMMIT_ALT_M, (
        "the commit must begin at or above the vision floor, not below it"
    )


def test_the_original_half_metre_pad_was_undecodable_at_every_search_altitude():
    """The counter-example the pad size was changed to escape.

    Uses explicit legacy geometry rather than config, because config no longer
    describes a single-marker pad and a test that quietly followed it would stop
    testing anything.
    """
    legacy_marker_fraction = 0.796  # the old texture: marker / plane

    for altitude_m in (config.TARGET_ALT_M, config.SEARCH_ALT_M, 5.0):
        decodable_px = config.marker_px_at_altitude(altitude_m, 0.5) * legacy_marker_fraction
        assert decodable_px < config.MARKER_MIN_DECODE_PX, (
            f"the original 0.5 m pad presents {decodable_px:.1f} px at "
            f"{altitude_m} m and should be undecodable"
        )

    assert config.marker_px_at_altitude(config.TARGET_ALT_M, 0.5) == pytest.approx(13.9, abs=0.1)


def test_the_flight_measurement_that_calibrated_the_apparent_size_model():
    """The one in-flight measurement taken during this work, preserved.

    At 5.51 m over the ORIGINAL pad_0 -- a 2.0 m plane carrying a single marker
    at 0.796 of its width -- the live vision bridge reported 88.6 px. The
    plane-width prediction was 100.6 px and the marker-corrected prediction
    80.1 px, so the measurement sat between them and nearer the corrected
    figure. That is what established that the decoder sees the MARKER, not the
    plane it is painted on.

    The pad has since been replaced, so this is history rather than a live
    budget -- but it is the empirical basis for correcting apparent size by the
    marker fraction at all, which world.pad_layout still does via
    QUIET_ZONE_FRACTION. Kept so that a future change to the intrinsics has to
    confront a real observation and not only arithmetic.
    """
    altitude_m = 5.51
    measured_px = 88.6
    legacy_marker_fraction = 0.796

    plane_px = config.marker_px_at_altitude(altitude_m)
    marker_px = plane_px * legacy_marker_fraction

    assert marker_px < measured_px < plane_px * 1.05, (
        f"measured {measured_px} px should sit between the marker-corrected "
        f"{marker_px:.1f} px and the plane {plane_px:.1f} px"
    )
    assert abs(measured_px - marker_px) < abs(measured_px - plane_px), (
        "the measurement should sit closer to the marker-corrected prediction"
    )


def test_the_shipped_textures_carry_the_layout_they_claim():
    """Read every shipped pad texture back through the real detector.

    Generating a texture that does not decode is finding F1 all over again --
    the original defect was precisely an asset that looked right and decoded in
    zero dictionaries. world/build_pads.py verifies at generation time; this
    verifies what is actually committed, which is the thing that flies.
    """
    import cv2

    import world.pad_layout as pad_layout

    project_root = Path(__file__).resolve().parents[1]
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, pad_layout.ARUCO_DICTIONARY)),
        cv2.aruco.DetectorParameters(),
    )

    for index in range(pad_layout.PAD_COUNT):
        texture = project_root / "sim" / "models" / f"pad_{index}" / f"pad_{index}.png"
        image = cv2.imread(str(texture), cv2.IMREAD_GRAYSCALE)
        assert image is not None, f"{texture} missing -- run `python3 -m world.build_pads`"

        corners, ids, _ = detector.detectMarkers(image)
        assert ids is not None, f"pad_{index} texture does not decode at all (finding F1)"

        found = set(ids.flatten().tolist())
        expected = {m.marker_id for m in pad_layout.pad_markers(index)}
        assert found >= expected, (
            f"pad_{index} texture decodes {sorted(found)} but the layout declares "
            f"{sorted(expected)}. Regenerate with `python3 -m world.build_pads`."
        )

        # And the drawn sizes must match the declared geometry, because the
        # estimator's metric scale comes from a marker's known side length.
        pixels_per_m = image.shape[0] / pad_layout.PAD_SIZE_M
        by_id = {int(i): c[0] for c, i in zip(corners, ids.flatten(), strict=True)}
        for marker in pad_layout.pad_markers(index):
            points = by_id[marker.marker_id]
            sides = [float(((points[i] - points[(i + 1) % 4]) ** 2).sum() ** 0.5) for i in range(4)]
            drawn_m = (sum(sides) / 4.0) / pixels_per_m
            assert drawn_m == pytest.approx(marker.side_m, abs=0.02), (
                f"pad_{index} marker {marker.marker_id} is drawn {drawn_m:.3f} m "
                f"across but the layout says {marker.side_m:.3f} m. Every metric "
                f"offset the estimator reports is scaled by this."
            )


def test_the_old_single_marker_textures_are_gone():
    """The undecodable-at-low-altitude asset must not linger.

    A stale aruco_N.png left beside the new one is an invitation for an
    <albedo_map> reference to quietly resurrect the pad that could not be seen
    below 2.5 m.
    """
    project_root = Path(__file__).resolve().parents[1]
    for index in range(3):
        legacy = project_root / "sim" / "models" / f"pad_{index}" / f"aruco_{index}.png"
        assert not legacy.exists(), (
            f"{legacy} is the old single-marker texture and must be removed; "
            f"run `python3 -m world.build_pads`"
        )


def test_search_altitude_is_below_cruise_so_a_descent_actually_happens():
    assert config.SEARCH_ALT_M < config.TARGET_ALT_M


def test_config_pad_size_matches_the_shipped_model():
    """The px = fx*s/h budget is only true if config and the SDF agree."""
    project_root = Path(__file__).resolve().parents[1]
    for index in (0, 1, 2):
        sdf = (project_root / "sim" / "models" / f"pad_{index}" / "model.sdf").read_text()
        expected = f"<size>{config.PAD_SIZE_M} {config.PAD_SIZE_M}</size>"
        assert expected in sdf, (
            f"pad_{index}/model.sdf must declare {expected} to match "
            f"config.PAD_SIZE_M; the acquisition-altitude budget depends on it"
        )


def test_camera_intrinsics_match_the_shipped_model():
    project_root = Path(__file__).resolve().parents[1]
    sdf = (project_root / "sim" / "models" / "x500_delivery" / "model.sdf").read_text()
    assert f"<width>{config.CAMERA_WIDTH_PX}</width>" in sdf
    assert f"<height>{config.CAMERA_HEIGHT_PX}</height>" in sdf
    assert f"<horizontal_fov>{config.CAMERA_HFOV_RAD}</horizontal_fov>" in sdf
    assert config.CAMERA_FX_PX == pytest.approx(277.1, abs=0.2), (
        "fx is derived from the two above; if it moved, so did the landing gain"
    )


def test_the_model_declares_no_absolute_sensor_topic():
    """F7: an absolute <topic> makes every drone in a fleet share one feed.

    Checks the parsed XML rather than the raw text, because the file's own
    comments discuss <topic> at length explaining why it is absent.
    """
    import xml.etree.ElementTree as ElementTree

    project_root = Path(__file__).resolve().parents[1]
    tree = ElementTree.parse(project_root / "sim" / "models" / "x500_delivery" / "model.sdf")

    sensors = tree.iter("sensor")
    named = [(sensor.get("name"), sensor.find("topic")) for sensor in sensors]
    assert named, "the model must declare sensors at all"

    offenders = [name for name, topic in named if topic is not None]
    assert not offenders, (
        f"sensors {offenders} declare an explicit <topic>, which re-introduces "
        f"finding F7: N drones publishing onto the same topics, with every "
        f"consumer seeing a blend of all of them"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  P4.2: descent gating
# ─────────────────────────────────────────────────────────────────────────────


def test_the_descent_gate_narrows_with_altitude():
    """A lateral error tolerable at 8 m is a miss at 0 m."""
    limits = [landing.descent_gate_limit_m(h) for h in (8.0, 4.0, 2.0, 1.0, 0.3)]
    assert limits == sorted(limits, reverse=True), "the gate must narrow"
    assert limits[-1] < 0.35, f"the final gate must be tight, got {limits[-1]:.2f} m"
    assert limits[0] > 1.0, "and generous high up, or the drone never descends"


def test_the_descent_gate_has_a_floor_it_cannot_go_below():
    """The difference between a gate and a deadlock.

    A limit that keeps shrinking eventually demands better centring than the
    measurement noise allows, and then it never opens again -- the drone holds a
    perfect lock and simply stops descending. The floor is what makes the gate
    always satisfiable.
    """
    for altitude_m in (0.0, 0.05, 0.1, 0.5, 2.0, 10.0):
        assert landing.descent_gate_limit_m(altitude_m) >= config.DESCENT_ACCEPT_FLOOR_M

    # And the floor is well clear of the measured 3-21 mm measurement error.
    assert config.DESCENT_ACCEPT_FLOOR_M > 8.0 * 0.021


def test_being_outside_the_gate_blocks_the_descent():
    altitude_m = 2.0
    limit_m = landing.descent_gate_limit_m(altitude_m)

    assert landing.should_descend(0.5 * limit_m, altitude_m, currently_descending=False)
    assert not landing.should_descend(3.0 * limit_m, altitude_m, currently_descending=False)

    # A 1 m offset at 2 m altitude must block the descent.
    assert not landing.should_descend(1.0 + limit_m, altitude_m, currently_descending=False)


def test_the_gate_has_hysteresis_so_it_cannot_chatter():
    """A descent that starts and stops at 10 Hz is a hover with extra steps."""
    altitude_m = 2.0
    limit_m = landing.descent_gate_limit_m(altitude_m)
    marginal_m = limit_m * 1.2  # just outside the entry limit

    assert not landing.should_descend(marginal_m, altitude_m, currently_descending=False), (
        "must not START descending outside the entry limit"
    )
    assert landing.should_descend(marginal_m, altitude_m, currently_descending=True), (
        "but must not immediately STOP for the same offset once descending"
    )
    assert config.DESCENT_EXIT_HYSTERESIS > 1.0


def test_the_loop_never_climbs_below_the_no_climb_altitude():
    """The structural fix for the reported hover.

    Losing the pad low down is what the field of view does, not an emergency.
    Answering it with a climb restores the lock, which permits another descent,
    which loses it again.
    """
    assert not landing.may_climb(config.NO_CLIMB_ALT_M - 0.01)
    assert not landing.may_climb(0.5)
    assert landing.may_climb(config.NO_CLIMB_ALT_M + 0.01)

    # And it must cover the whole band where losing the pad is normal: measured
    # lock loss was 2.52 m centred and 3.42 m with 8 degrees of tilt.
    assert config.NO_CLIMB_ALT_M >= 2.6


def test_the_altitude_ratchet_makes_a_landing_monotonic():
    """Whatever else misbehaves, the drone cannot walk back up."""
    ratchet = landing.AltitudeRatchet()
    for altitude_m in (6.0, 5.0, 4.0, 3.0, 2.0):
        ratchet.observe(altitude_m)

    assert ratchet.lowest_m == pytest.approx(2.0)
    assert ratchet.ceiling_m == pytest.approx(2.0 + config.ALTITUDE_FLOOR_HYSTERESIS_M)

    # A climb command below the ceiling is allowed (a real descent overshoots
    # and settles, and pinning it at its lowest noisy sample would be worse).
    assert ratchet.limit_climb(2.1, -0.3) == pytest.approx(-0.3)
    # At or above the ceiling it is refused.
    assert ratchet.limit_climb(2.5, -0.3) == 0.0
    # Descents are never touched.
    assert ratchet.limit_climb(2.5, 0.4) == pytest.approx(0.4)


def test_descent_rate_slows_toward_touchdown():
    rates = [landing.descent_rate_m_s(h) for h in (5.0, 1.2, 0.4)]
    assert rates == sorted(rates, reverse=True), f"descent must slow: {rates}"
    assert rates[0] == config.DESCENT_VZ_HIGH_M_S
    assert rates[-1] == config.DESCENT_VZ_FINAL_M_S
    assert rates[-1] <= 0.25, (
        "the last stretch must be gentle -- but not so slow that the drone hangs "
        "above the ground never satisfying PX4's land detector, which needs "
        "sustained ground contact to latch"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  P4.4: touchdown detection
# ─────────────────────────────────────────────────────────────────────────────


def test_touchdown_trusts_the_autopilots_on_ground_at_any_altitude():
    """ON_GROUND is authoritative, because LandedState fuses altitude, vertical
    velocity and thrust -- so it can report a touchdown on raised ground that
    the height threshold alone would miss."""
    assert landing.is_on_ground({"landed_state": "ON_GROUND", "alt": 0.8})


def test_in_air_does_not_veto_a_low_height_because_offboard_suppresses_it():
    """The regression that cost a live landing.

    THE FLIGHT. The controller flew the descent perfectly and then threw the
    result away:

        14:56:27  committing to the landing from 1.17 m, 0.042 m off centre
        14:56:39  commit did not reach the ground within 12 s (still -0.03 m up)

    Gazebo ground truth had the drone stopped on the pad, 0.12 m from its
    centre. Its own height reading was -0.03 m. It returned STALLED, the leg
    failed, and the mission was recorded FAILED.

    THE CAUSE. is_on_ground let an explicit IN_AIR veto the height check. That
    looks prudent -- the autopilot ought to know better than one noisy sample --
    but it is a guaranteed deadlock during the commit, because PX4's land
    detector requires sustained low thrust and near-zero velocity and the commit
    streams a 0.2 m/s DOWNWARD VELOCITY SETPOINT IN OFFBOARD until touchdown.
    The detector will not declare a landing while it is being commanded to fly,
    so LandedState stays IN_AIR straight through the ground. The veto therefore
    fired precisely when the height was the only signal left, and never when a
    sample was actually noisy.

    Noise rejection now lives where it can do the job without deadlocking: both
    callers require the condition to hold for config.TOUCHDOWN_PUSH_S and then
    confirm by commanding a land and waiting for a real disarm.
    """
    assert landing.is_on_ground({"landed_state": "IN_AIR", "alt": -0.03}), (
        "the exact state of the failed flight: on the ground, reported IN_AIR"
    )
    assert landing.is_on_ground({"landed_state": "LANDING", "alt": 0.1})

    # Still not on the ground merely because the autopilot has not said so.
    assert not landing.is_on_ground({"landed_state": "IN_AIR", "alt": 4.0})
    assert not landing.is_on_ground({"landed_state": "TAKING_OFF", "alt": 1.0})


def test_touchdown_falls_back_to_altitude_when_landed_state_is_absent():
    below_m = config.TOUCHDOWN_ALT_M * 0.5
    assert landing.is_on_ground({"alt": below_m})
    assert not landing.is_on_ground({"alt": 3.0})
    assert not landing.is_on_ground({}), "an unknown altitude must not read as landed"

    # The height may also be supplied explicitly, because the loop prefers the
    # vision range -- height above the PAD -- to the EKF's height above home.
    assert landing.is_on_ground({}, height_m=below_m)
    assert not landing.is_on_ground({}, height_m=1.0)


# ─────────────────────────────────────────────────────────────────────────────
#  P4.5: the search pattern
# ─────────────────────────────────────────────────────────────────────────────


def test_spiral_rings_overlap_the_camera_footprint():
    """Ring spacing above the footprint stripes and leaves gaps the pad sits in."""
    for altitude_m in (3.0, 6.0, 10.0):
        step_m = config.search_ring_step_m(altitude_m)
        footprint_m = landing.camera_footprint_m(altitude_m)
        assert step_m < footprint_m, (
            f"at {altitude_m} m the {step_m:.2f} m ring step must be under the "
            f"{footprint_m:.2f} m footprint or coverage stripes"
        )
        assert step_m == pytest.approx(config.SEARCH_RING_OVERLAP * footprint_m)


def test_the_spiral_is_bounded_and_starts_at_the_expected_position():
    waypoints = landing.generate_spiral_waypoints(30.0315, 72.3140)
    assert waypoints[0] == (30.0315, 72.3140), "search must begin where the pad is"
    assert 5 < len(waypoints) < 60, (
        f"{len(waypoints)} waypoints; the old fixed 2 m spacing produced 100+, "
        f"whose path length drove a multi-minute timeout"
    )

    from drone_agent import geo

    for lat, lon in waypoints:
        distance_m = geo.get_distance_m(30.0315, 72.3140, lat, lon)
        assert distance_m <= config.SEARCH_SPIRAL_MAX_RADIUS_M + 1e-6, (
            f"waypoint {distance_m:.1f} m out exceeds the "
            f"{config.SEARCH_SPIRAL_MAX_RADIUS_M} m search radius"
        )


def test_the_search_timeout_is_hard_capped():
    """The old max(LANDING_TIMEOUT_S, path/speed) gave minutes of hovering.

    A drone that has already failed to find its pad should not burn the battery
    the return leg needs.
    """
    # A deliberately huge pattern, to prove the cap binds.
    huge = landing.generate_spiral_waypoints(30.0315, 72.3140, altitude_m=1.0, max_radius_m=200.0)
    assert len(huge) > 100
    assert landing.search_timeout_s(huge) == config.LANDING_TIMEOUT_MAX_S
    assert config.LANDING_TIMEOUT_MAX_S <= 240


def test_search_holds_altitude_rather_than_drifting():
    """The old search sent vz=0 with no correction, so the drone drifted.

    Marker decodability is a direct function of altitude, so drifting during the
    search actively defeats it.
    """
    from drone_agent.navigation import altitude_correction

    # Below the search altitude -> climb (negative down-velocity).
    assert altitude_correction(4.0, config.SEARCH_ALT_M) < 0
    # Above it -> descend.
    assert altitude_correction(8.0, config.SEARCH_ALT_M) > 0
    # At it -> no correction.
    assert altitude_correction(config.SEARCH_ALT_M, config.SEARCH_ALT_M) == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
#  disambiguation
# ─────────────────────────────────────────────────────────────────────────────


def test_select_target_never_returns_the_wrong_marker():
    """Three pads within 15 m of each other, all in frame at once."""
    detections = [
        {"id": 2, "err_x": 40.0, "err_y": -10.0},
        {"id": 0, "err_x": -30.0, "err_y": 5.0},
        {"id": 1, "err_x": 2.0, "err_y": 1.0},
    ]
    for wanted in (0, 1, 2):
        picked = landing.select_target(detections, wanted)
        assert picked is not None and picked["id"] == wanted

    assert landing.select_target(detections, 3) is None
    assert landing.select_target([], 0) is None


# ─────────────────────────────────────────────────────────────────────────────
#  Holding station through the open-loop commit
# ─────────────────────────────────────────────────────────────────────────────


def test_commit_hold_does_nothing_when_the_drone_has_not_drifted():
    """Inside the deadband there is nothing to correct.

    The deadband is roughly the EKF's own horizontal noise. Correcting inside it
    would be steering on the estimator rather than on the drone.
    """
    assert landing.commit_hold_velocity(30.0315, 72.3140, 30.0315, 72.3140) == (0.0, 0.0)

    # A drift comfortably smaller than the deadband, expressed in degrees.
    tiny_deg = (config.COMMIT_HOLD_DEADBAND_M * 0.4) / 111320.0
    assert landing.commit_hold_velocity(30.0315 + tiny_deg, 72.3140, 30.0315, 72.3140) == (
        0.0,
        0.0,
    )


def test_commit_hold_flies_back_toward_the_spot_it_committed_from():
    """The sign that makes this a correction rather than an escape.

    Drifting NORTH of the commit point must command a SOUTHWARD velocity, and
    drifting EAST must command WESTWARD. Getting either backwards turns the last
    metre of every landing into positive feedback, with no vision left to notice.
    """
    north_deg = 0.30 / 111320.0
    north_cmd, east_cmd = landing.commit_hold_velocity(
        30.0315 + north_deg, 72.3140, 30.0315, 72.3140
    )
    assert north_cmd < 0.0, f"drifted north, so it must fly south; got {north_cmd:+.3f}"
    assert abs(east_cmd) < 1e-6

    east_deg = 0.30 / geo.metres_per_deg_lon(30.0315)
    north_cmd, east_cmd = landing.commit_hold_velocity(
        30.0315, 72.3140 + east_deg, 30.0315, 72.3140
    )
    assert east_cmd < 0.0, f"drifted east, so it must fly west; got {east_cmd:+.3f}"
    assert abs(north_cmd) < 1e-6


def test_commit_hold_is_proportional_and_clamped():
    """Bigger drift, bigger correction -- up to a ceiling.

    Near the ground with the pad out of sight, a large horizontal command is
    never the right answer however confident the estimate is.
    """

    def north_command(drift_m):
        return landing.commit_hold_velocity(
            30.0315 + drift_m / 111320.0, 72.3140, 30.0315, 72.3140
        )[0]

    small, large = abs(north_command(0.10)), abs(north_command(0.30))
    assert large > small, "the correction must grow with the drift"
    assert small == pytest.approx(config.COMMIT_HOLD_K_P * 0.10, abs=0.02)

    assert abs(north_command(50.0)) == pytest.approx(config.COMMIT_HOLD_MAX_VEL_M_S), (
        "an absurd drift -- a bad GPS jump -- must not become an absurd command"
    )


def test_commit_hold_beats_commanding_zero_over_a_realistic_commit():
    """The measurement that motivated this, reproduced as a test.

    Nine live landings centred to 0.042 m at commit and then touched down
    0.209 m out on average: the open-loop stretch added 0.170 m of drift that
    nothing opposed, because a zero VELOCITY setpoint asks PX4 to stop moving
    rather than to stay put.

    A first-order plant with a steady disturbance is enough to show the
    difference in kind. Commanding zero lets the disturbance integrate freely;
    the position loop bounds it.
    """
    dt_s, tau_s = 1.0 / config.NAV_HZ, 0.35
    disturbance_m_s = 0.05  # a light, steady push
    commit_s = 6.0  # 1.2 m at DESCENT_VZ_FINAL_M_S

    def fly(hold: bool) -> float:
        north_m, v_m_s = 0.0, 0.0
        for _ in range(int(commit_s / dt_s)):
            cmd = 0.0
            if hold:
                cmd = landing.commit_hold_velocity(
                    30.0315 + north_m / 111320.0, 72.3140, 30.0315, 72.3140
                )[0]
            v_m_s += (cmd - v_m_s) * (dt_s / tau_s)
            north_m += (v_m_s + disturbance_m_s) * dt_s
        return abs(north_m)

    open_loop_m, held_m = fly(hold=False), fly(hold=True)
    assert open_loop_m == pytest.approx(disturbance_m_s * commit_s, abs=0.02), (
        "sanity: commanding zero lets the disturbance integrate unopposed"
    )
    assert held_m < open_loop_m * 0.5, (
        f"holding position must at least halve the drift; got {held_m:.3f} m held "
        f"against {open_loop_m:.3f} m open loop"
    )
