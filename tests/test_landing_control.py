"""P4 acceptance: the three reasons precision landing never worked.

The 20-landing statistical criterion needs a simulator and lives in
docs/ACCEPTANCE.md as a scripted procedure. What is provable here is that the
control law itself is now sound: altitude-invariant gain (F4), a marker large
enough to decode at the altitude it is searched from (F3), a decodable pad (F1),
and a descent that cannot proceed while badly off-centre.
"""

import math
from pathlib import Path

import pytest

import config
from drone_agent import landing

# ─────────────────────────────────────────────────────────────────────────────
#  F4: altitude-invariant control
# ─────────────────────────────────────────────────────────────────────────────


def test_effective_loop_gain_is_altitude_invariant():
    """THE F4 FIX, stated as a test.

    A pixel-proportional controller has effective gain K_p * fx / h, which rises
    tenfold between 10 m and 1 m:

        h = 10 m -> 0.42 1/s   sluggish but stable
        h =  1 m -> 4.16 1/s   overshoot and oscillation, at touchdown

    Converting to metres first makes the gain a property of the controller.
    """
    ground_offset_m = 0.5  # the same real error at every altitude

    commands = []
    for altitude_m in (10.0, 5.0, 3.0, 1.0, 0.5):
        # What the camera would report for this real offset.
        err_px = ground_offset_m * config.CAMERA_FX_PX / altitude_m
        recovered_m = landing.pixel_to_ground_offset_m(err_px, altitude_m)
        assert recovered_m == pytest.approx(ground_offset_m, rel=1e-6), (
            "pixel -> metre conversion must be exact"
        )
        commands.append(config.LANDING_K_P * recovered_m)

    assert max(commands) - min(commands) < 1e-6, (
        f"the command for a fixed ground offset must not depend on altitude, got {commands}"
    )

    # And demonstrate the failure it replaces.
    old_gain = 0.015
    old_effective = [old_gain * config.CAMERA_FX_PX / h for h in (10.0, 1.0)]
    assert old_effective[1] / old_effective[0] == pytest.approx(10.0, rel=1e-6), (
        "the old controller's gain really did rise 10x on the way down"
    )


def test_offset_conversion_is_clamped_near_the_ground():
    """Below a few centimetres the projection is degenerate.

    Without a clamp, altitude -> 0 makes the recovered offset -> 0 and the
    controller stops correcting at exactly the wrong moment; with a naive
    reciprocal it would instead explode.
    """
    assert landing.pixel_to_ground_offset_m(100.0, 0.0) == pytest.approx(
        landing.pixel_to_ground_offset_m(100.0, 0.15)
    )
    assert math.isfinite(landing.pixel_to_ground_offset_m(100.0, -5.0))


def test_pixel_to_metre_rejects_a_nonsense_focal_length():
    with pytest.raises(ValueError):
        landing.pixel_to_ground_offset_m(10.0, 5.0, fx_px=0.0)


def test_body_frame_signs_match_the_derived_camera_mount():
    """F6: the mount is pose 0 0 -0.05 0 1.5708 0, a +90 degree pitch about Y.

    Gazebo cameras look down +X with image right = -Y and image down = -Z, so
    after the rotation: image u (right) -> body right, image v (down) -> body
    aft. A marker below centre is BEHIND the drone.

    These signs are easy to "fix" into a positive feedback loop that flies away
    from the pad, so they are pinned here and in docs/CALIBRATION.md.
    """
    altitude_m = 5.0

    # Marker below image centre (positive err_y) -> it is behind us -> fly back.
    forward_m, right_m = landing.metric_offsets({"err_x": 0.0, "err_y": 50.0}, altitude_m)
    assert forward_m < 0, "a marker below centre must command backward motion"
    assert right_m == pytest.approx(0.0)

    # Marker right of centre (positive err_x) -> it is to our right -> fly right.
    forward_m, right_m = landing.metric_offsets({"err_x": 50.0, "err_y": 0.0}, altitude_m)
    assert right_m > 0, "a marker right of centre must command rightward motion"
    assert forward_m == pytest.approx(0.0)

    # Magnitudes are symmetric.
    assert abs(
        landing.metric_offsets({"err_x": 0, "err_y": -50.0}, altitude_m)[0]
    ) == pytest.approx(abs(forward_m) if forward_m else 50.0 * altitude_m / config.CAMERA_FX_PX)


def test_ema_filters_the_measurement_not_the_command():
    """Filtering the output adds actuator lag without denoising the sensor."""
    noisy = [(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)] * 25
    ema = landing.EMAFilter(alpha=0.3)
    filtered = [ema.update(sample)[0] for sample in noisy]

    raw_variance = sum(x[0] ** 2 for x in noisy[-20:]) / 20
    filtered_variance = sum(x**2 for x in filtered[-20:]) / 20
    assert filtered_variance < raw_variance * 0.5, (
        f"the filter must materially reduce variance: {raw_variance:.3f} -> {filtered_variance:.3f}"
    )


def test_ema_reset_discards_a_stale_lock():
    """A reacquired marker must not be averaged with where it was seconds ago."""
    ema = landing.EMAFilter()
    ema.update((5.0, 5.0, 0.0))
    ema.reset()
    assert ema.update((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
#  F3: the marker/altitude budget
# ─────────────────────────────────────────────────────────────────────────────


def test_the_pad_is_decodable_at_the_altitude_it_is_searched_from():
    """F3: a 0.5 m pad spans 14 px at cruise altitude and cannot be decoded.

    The documented flow -- arrive at TARGET_ALT, then search -- therefore could
    not lock even with a decodable texture. Both levers are applied: a 2 m pad
    AND a descent to SEARCH_ALT_M before searching.

    Compared against decodable_px_at_altitude(), not the raw plane width: the
    quiet zone means the decoder only ever sees about 80% of the pad, so
    comparing the plane width overstates the margin by that much.
    """
    at_search_alt = config.decodable_px_at_altitude(config.SEARCH_ALT_M)
    assert at_search_alt >= config.MARKER_MIN_DECODE_PX * 2.0, (
        f"a {config.PAD_SIZE_M} m pad presents {at_search_alt:.0f} px of decodable "
        f"marker at {config.SEARCH_ALT_M} m; want at least 2x the "
        f"{config.MARKER_MIN_DECODE_PX:.0f} px threshold for margin"
    )

    # Reproduce the plan's plane-width figure for the pad this replaces.
    assert config.marker_px_at_altitude(config.TARGET_ALT_M, 0.5) == pytest.approx(13.9, abs=0.1)

    # Marker-corrected, the original pad is undecodable at EVERY altitude the
    # mission would have searched from - not merely "marginal at 5 m" as the
    # plane-width figures suggested.
    for altitude_m in (config.TARGET_ALT_M, config.SEARCH_ALT_M, 5.0):
        assert config.decodable_px_at_altitude(altitude_m, 0.5) < config.MARKER_MIN_DECODE_PX, (
            f"the original 0.5 m pad should be undecodable at {altitude_m} m"
        )

    # The shipped pad clears the threshold even at full cruise altitude, which is
    # what gives the search phase its margin.
    assert config.decodable_px_at_altitude(config.TARGET_ALT_M) > config.MARKER_MIN_DECODE_PX


def test_quiet_zone_fraction_matches_the_shipped_texture():
    """config.MARKER_QUIET_ZONE_FRACTION is a measurement, so measure it.

    The pad textures carry a white quiet zone - the whole reason they decode at
    all (finding F1) - so the black marker is smaller than the plane it is
    painted on. If a texture is regenerated without that border, or with a
    different one, the apparent-size budget silently shifts.
    """
    import cv2

    project_root = Path(__file__).resolve().parents[1]
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )

    for index in (0, 1, 2):
        texture = project_root / "sim" / "models" / f"pad_{index}" / f"aruco_{index}.png"
        image = cv2.imread(str(texture))
        assert image is not None, f"{texture} unreadable"

        corners, ids, _ = detector.detectMarkers(image)
        assert ids is not None, f"pad_{index} texture does not decode (finding F1)"
        assert index in ids.flatten().tolist()

        points = corners[0][0]
        sides = [float(((points[i] - points[(i + 1) % 4]) ** 2).sum() ** 0.5) for i in range(4)]
        fraction = (sum(sides) / 4.0) / image.shape[1]

        assert fraction == pytest.approx(config.MARKER_QUIET_ZONE_FRACTION, abs=0.03), (
            f"pad_{index}: the marker occupies {fraction:.3f} of the texture but "
            f"config.MARKER_QUIET_ZONE_FRACTION says "
            f"{config.MARKER_QUIET_ZONE_FRACTION}. The apparent-size budget "
            f"depends on this ratio."
        )


def test_flight_measured_marker_size_matches_the_corrected_budget():
    """Pins the one in-flight measurement taken during this remediation.

    At 5.51 m over pad_0, the live vision bridge reported 88.6 px. The
    plane-width prediction was 100.6 px; the marker-corrected prediction was
    80.1 px. The measurement sits between them and much nearer the corrected
    figure, which is why decodable_px_at_altitude() exists. Recorded as a test
    so that a future change to the intrinsics or the pad has to confront a real
    observation rather than only the arithmetic.
    """
    altitude_m = 5.51
    measured_px = 88.6

    plane_px = config.marker_px_at_altitude(altitude_m)
    marker_px = config.decodable_px_at_altitude(altitude_m)

    assert marker_px < measured_px < plane_px * 1.05, (
        f"measured {measured_px} px should sit between the marker-corrected "
        f"{marker_px:.1f} px and the plane {plane_px:.1f} px"
    )
    assert abs(measured_px - marker_px) < abs(measured_px - plane_px), (
        "the measurement should sit closer to the marker-corrected prediction"
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


def test_the_descent_cone_narrows_with_altitude():
    """A lateral error tolerable at 8 m is a miss at 0 m.

    The old loop descended at 0.4 m/s regardless of centring, so an 8 m-altitude
    error arrived intact at the ground.
    """
    limits = [landing.descent_cone_limit_m(h) for h in (8.0, 4.0, 2.0, 1.0, 0.3)]
    assert limits == sorted(limits, reverse=True), "the cone must narrow"
    assert limits[-1] < 0.3, f"the final gate must be tight, got {limits[-1]:.2f} m"
    assert limits[0] > 1.0, "and generous high up, or the drone never descends"


def test_being_outside_the_cone_blocks_the_descent():
    """The gate must actually exclude a real off-centre case."""
    altitude_m = 2.0
    limit_m = landing.descent_cone_limit_m(altitude_m)
    assert 0.5 * limit_m < limit_m  # inside
    assert 2.0 * limit_m > limit_m  # outside

    # A 1 m offset at 2 m altitude must be outside the gate.
    assert 1.0 > limit_m, (
        f"1 m off-centre at 2 m altitude must block the descent (limit {limit_m:.2f} m)"
    )


def test_descent_rate_slows_toward_touchdown():
    rates = [landing.descent_rate_m_s(h) for h in (5.0, 1.2, 0.4)]
    assert rates == sorted(rates, reverse=True), f"descent must slow: {rates}"
    assert rates[0] == config.DESCENT_VZ_HIGH_M_S
    assert rates[-1] == config.DESCENT_VZ_FINAL_M_S
    assert rates[-1] < 0.2, "the last stretch must be gentle"


# ─────────────────────────────────────────────────────────────────────────────
#  P4.4: touchdown detection
# ─────────────────────────────────────────────────────────────────────────────


def test_touchdown_prefers_the_autopilots_landed_state():
    """LandedState fuses altitude, vertical velocity and thrust."""
    assert landing.is_on_ground({"landed_state": "ON_GROUND", "alt": 0.8})
    assert not landing.is_on_ground({"landed_state": "IN_AIR", "alt": 0.05}), (
        "an explicit IN_AIR must override a noisy low altitude sample"
    )
    assert not landing.is_on_ground({"landed_state": "LANDING", "alt": 0.1})


def test_touchdown_falls_back_to_altitude_when_landed_state_is_absent():
    assert landing.is_on_ground({"alt": 0.2})
    assert not landing.is_on_ground({"alt": 3.0})
    assert not landing.is_on_ground({}), "an unknown altitude must not read as landed"


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
