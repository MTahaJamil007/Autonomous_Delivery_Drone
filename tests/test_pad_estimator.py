"""The nested pad and its estimator, measured against ground truth.

These tests render the SHIPPED pad texture through a synthetic pinhole camera
matching the shipped model.sdf, run the REAL cv2.aruco detector over the result,
and compare what perception.pad_estimator recovers against the pose that was
rendered. Nothing here is a mock: if OpenCV, the texture or the geometry change
in a way that breaks the landing loop, these fail.

The camera model is a supersampled warp -- render at 4x and box-downsample --
because that is what a real sensor does to a scene, and because a naive
single-sample warp aliases badly enough at large upscales to produce detection
results that are artefacts of the renderer rather than of the geometry.

WHAT THIS PROTECTS
------------------
The original pad's failure was invisible to every unit test in the repo because
no test ever asked the detector whether it could actually see the pad at the
altitude the controller would be flying. test_at_least_one_marker_is_readable
_through_the_whole_descent is that question, asked directly.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import config
import world.pad_layout as pad_layout

cv2 = pytest.importorskip("cv2", reason="pad estimation needs OpenCV")

from perception import pad_estimator  # noqa: E402

SUPERSAMPLE = 4

WIDTH_PX = config.CAMERA_WIDTH_PX
HEIGHT_PX = config.CAMERA_HEIGHT_PX
FX_PX = config.CAMERA_FX_PX


@pytest.fixture(scope="module")
def detector():
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, pad_layout.ARUCO_DICTIONARY)),
        params,
    )


@pytest.fixture(scope="module")
def textures():
    return {index: pad_layout.render_pad_texture(index) for index in range(pad_layout.PAD_COUNT)}


def render(
    texture: np.ndarray,
    altitude_m: float,
    offset_right_m: float = 0.0,
    offset_down_m: float = 0.0,
    rotation_deg: float = 0.0,
) -> np.ndarray:
    """The pad as this camera would see it, nadir, at `altitude_m`.

    `offset_right_m`/`offset_down_m` place the PAD relative to the optical axis,
    in the image plane's own directions, which is exactly what the estimator
    reports back. Rendering and measuring in one frame keeps these tests from
    silently encoding the body-frame sign convention, which is a separate
    concern tested in tests/test_landing_control.py.
    """
    texture_h, texture_w = texture.shape
    big_w, big_h = WIDTH_PX * SUPERSAMPLE, HEIGHT_PX * SUPERSAMPLE
    image = np.full((big_h, big_w), 120, np.uint8)  # grass-grey ground

    scale = (FX_PX * SUPERSAMPLE) / altitude_m
    pad_px = pad_layout.PAD_SIZE_M * scale
    centre_x = big_w / 2.0 + offset_right_m * scale
    centre_y = big_h / 2.0 + offset_down_m * scale

    cos_r = math.cos(math.radians(rotation_deg))
    sin_r = math.sin(math.radians(rotation_deg))
    destination = []
    for dx, dy in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
        x, y = dx * pad_px, dy * pad_px
        destination.append([centre_x + x * cos_r - y * sin_r, centre_y + x * sin_r + y * cos_r])

    source = np.float32([[0, 0], [texture_w, 0], [texture_w, texture_h], [0, texture_h]])
    cv2.warpPerspective(
        texture,
        cv2.getPerspectiveTransform(source, np.float32(destination)),
        (big_w, big_h),
        dst=image,
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_TRANSPARENT,
    )
    return cv2.resize(image, (WIDTH_PX, HEIGHT_PX), interpolation=cv2.INTER_AREA)


def observe(detector, image: np.ndarray, pad_index: int):
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return None
    raw = [(int(i), c[0]) for c, i in zip(corners, ids.flatten(), strict=True)]
    return pad_estimator.select_pad(
        pad_estimator.estimate_pads(raw, WIDTH_PX, HEIGHT_PX, FX_PX), pad_index
    )


# ═════════════════════════════════════════════════════════════════════════════
#  THE TEST THE OLD PAD WOULD HAVE FAILED
# ═════════════════════════════════════════════════════════════════════════════


def test_at_least_one_marker_is_readable_through_the_whole_descent(detector, textures):
    """The question nobody asked of the original pad.

    A landing loop can only work if the pad is measurable at every altitude it
    passes through. The old pad was unreadable below about 2.5 m, so the descent
    went blind exactly where it mattered -- and no unit test noticed, because
    none of them ran the detector at a descent altitude.
    """
    blind_at = []
    for altitude_m in (8.0, 6.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.7):
        observation = observe(detector, render(textures[0], altitude_m), 0)
        if observation is None:
            blind_at.append(altitude_m)

    assert not blind_at, (
        f"the pad must be visible throughout the descent, but no marker decoded "
        f"at {blind_at} m. The controller descends through every one of these."
    )


def test_the_commit_gate_is_satisfiable(detector, textures):
    """The commit gate must be reachable in the conditions it was written for.

    The gate is "below COMMIT_ALT_M and within COMMIT_MAX_OFFSET_M". If the pad
    is not measurable at COMMIT_ALT_M *while COMMIT_MAX_OFFSET_M off centre*,
    the drone goes blind just before the gate it is flying toward and can never
    prove it is centred -- which is the original defect wearing a different hat.

    This is why config.COMMIT_ALT_M is derived from the pad geometry rather than
    chosen: a hand-picked 0.70 m passed the centred version of this test and
    failed this one.
    """
    for right_m, down_m in (
        (0.0, 0.0),
        (config.COMMIT_MAX_OFFSET_M, 0.0),
        (0.0, config.COMMIT_MAX_OFFSET_M),
        (0.0, -config.COMMIT_MAX_OFFSET_M),
    ):
        observation = observe(
            detector, render(textures[0], config.COMMIT_ALT_M, right_m, down_m), 0
        )
        assert observation is not None, (
            f"the pad must be measurable at the commit altitude "
            f"({config.COMMIT_ALT_M:.2f} m) with an offset of "
            f"({right_m:+.2f}, {down_m:+.2f}) m, or the commit gate can never be met"
        )


def test_the_commit_altitude_is_derived_from_the_geometry(detector, textures):
    """A regression guard on the constant itself.

    Someone lowering COMMIT_ALT_M toward the centred vision floor would
    reintroduce the unsatisfiable gate above. Pin the relationship, not the
    number.
    """
    tan_h = math.tan(config.CAMERA_HFOV_RAD / 2.0)
    tan_v = (config.CAMERA_HEIGHT_PX / 2.0) / config.CAMERA_FX_PX
    floor_m = pad_layout.visibility_floor_m(
        config.CAMERA_FX_PX, tan_h, tan_v, offset_m=config.COMMIT_MAX_OFFSET_M
    )

    assert config.COMMIT_ALT_M > floor_m, (
        f"COMMIT_ALT_M ({config.COMMIT_ALT_M:.2f} m) must sit above the "
        f"geometric floor for a pad {config.COMMIT_MAX_OFFSET_M} m off axis "
        f"({floor_m:.2f} m), with margin"
    )
    assert config.COMMIT_ALT_M < config.NO_CLIMB_ALT_M, (
        "the commit must happen inside the band where climbing is already "
        "forbidden, or the drone could climb away from a committed landing"
    )


def test_the_old_single_marker_pad_really_does_go_blind(detector):
    """The counter-example, so the test above is known to be discriminating.

    Renders a pad carrying ONE marker at 0.796 of the plane -- the original
    layout -- and shows it becomes undetectable partway down. If this ever
    passes, the test above has stopped proving anything.
    """
    size_px = 1000
    marker_px = int(round(size_px * 0.796))
    legacy = np.full((size_px, size_px), 255, np.uint8)
    drawn = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, marker_px
    )
    inset = (size_px - marker_px) // 2
    legacy[inset : inset + marker_px, inset : inset + marker_px] = drawn

    detected = []
    for altitude_m in (6.0, 4.0, 3.0, 2.0, 1.5, 1.0):
        _, ids, _ = detector.detectMarkers(render(legacy, altitude_m))
        detected.append(ids is not None and 0 in ids.flatten())

    assert detected[0] and detected[1], "the old pad worked fine high up; that was never the issue"
    assert not detected[-1] and not detected[-2], (
        "the old pad must be undetectable near the ground -- that is the defect "
        f"being reproduced. Got {detected} for 6/4/3/2/1.5/1.0 m."
    )


# ═════════════════════════════════════════════════════════════════════════════
#  ACCURACY
# ═════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize(
    ("altitude_m", "offset"),
    [
        # Offsets shrink with altitude because the FRAME shrinks with altitude.
        # Asking for a 0.25 m offset at 0.8 m is asking to see something outside
        # the field of view; the controller is centred to far better than this
        # by the time it is that low, which is why COMMIT_MAX_OFFSET_M exists.
        (6.0, (0.0, 0.0)),
        (6.0, (0.60, -0.40)),
        (4.0, (0.0, 0.0)),
        (4.0, (0.40, -0.30)),
        (3.0, (0.0, 0.0)),
        (3.0, (0.30, -0.20)),
        (2.0, (0.0, 0.0)),
        (2.0, (0.20, -0.15)),
        (1.5, (0.0, 0.0)),
        (1.5, (0.15, -0.10)),
        (1.2, (0.0, 0.0)),
        (1.2, (0.10, -0.08)),
        (0.8, (0.0, 0.0)),
        (0.8, (0.05, -0.05)),
    ],
)
def test_offset_is_recovered_in_metres(detector, textures, altitude_m, offset):
    """The estimate must match the rendered truth, in metres.

    The tolerance is 60 mm, comfortably above the 3-21 mm actually achieved, so
    this fails on a real regression rather than on renderer noise.
    """
    right_m, down_m = offset
    observation = observe(detector, render(textures[0], altitude_m, right_m, down_m), 0)
    assert observation is not None, f"no detection at {altitude_m} m, offset {offset}"

    estimated_right_m, estimated_down_m = observation.offset_m
    error_m = math.hypot(estimated_right_m - right_m, estimated_down_m - down_m)
    assert error_m < 0.06, (
        f"offset error {error_m * 1000:.0f} mm at {altitude_m} m "
        f"(estimated {estimated_right_m:+.3f},{estimated_down_m:+.3f}, "
        f"true {right_m:+.3f},{down_m:+.3f})"
    )


@pytest.mark.parametrize("altitude_m", [6.0, 4.0, 3.0, 2.0, 1.5, 1.0, 0.8])
def test_range_is_recovered_without_an_altitude_input(detector, textures, altitude_m):
    """The estimator is told nothing about altitude and must still report it.

    This is what frees the landing loop from multiplying its error by a drifting
    EKF altitude, and what gives it a height measured against the PAD rather
    than against wherever the drone took off.
    """
    observation = observe(detector, render(textures[0], altitude_m), 0)
    assert observation is not None

    relative_error = abs(observation.range_m - altitude_m) / altitude_m
    assert relative_error < 0.08, (
        f"range {observation.range_m:.2f} m vs true {altitude_m:.2f} m "
        f"({relative_error * 100:.1f}% off)"
    )


def test_a_single_marker_is_enough(detector, textures):
    """Any ONE marker fixes the pad centre, including an off-centre one.

    This is the property that makes the nested layout work: at 1 m only the
    small centre marker is in frame, and at 6 m the centre marker is nearly too
    small to read while the outer ring is comfortable. Neither altitude has all
    five, and neither needs them.
    """
    image = render(textures[0], 2.0, 0.30, 0.0)
    corners, ids, _ = detector.detectMarkers(image)
    assert ids is not None

    raw = [(int(i), c[0]) for c, i in zip(corners, ids.flatten(), strict=True)]
    assert len(raw) >= 2, "this altitude should show more than one marker"

    full = pad_estimator.select_pad(pad_estimator.estimate_pads(raw, WIDTH_PX, HEIGHT_PX, FX_PX), 0)
    assert full is not None

    for marker_id, marker_corners in raw:
        single = pad_estimator.select_pad(
            pad_estimator.estimate_pads([(marker_id, marker_corners)], WIDTH_PX, HEIGHT_PX, FX_PX),
            0,
        )
        assert single is not None, f"marker {marker_id} alone should still locate the pad"
        disagreement_m = math.hypot(
            single.offset_m[0] - full.offset_m[0], single.offset_m[1] - full.offset_m[1]
        )
        assert disagreement_m < 0.10, (
            f"marker {marker_id} alone puts the pad centre {disagreement_m * 1000:.0f} mm "
            f"from where all markers together put it"
        )


def test_residual_reports_disagreement_rather_than_averaging_it_away(detector, textures):
    """A corrupted marker must raise residual_m, not quietly shift the answer.

    Displacing one marker's corners by 20 px simulates a bad refinement or a
    misdecode. The estimator cannot know which marker is wrong, but it must say
    that something is, so the landing loop can drop the frame instead of
    steering on the average of a good measurement and a bad one.
    """
    image = render(textures[0], 3.0)
    corners, ids, _ = detector.detectMarkers(image)
    assert ids is not None and len(ids) >= 3

    raw = [(int(i), c[0].copy()) for c, i in zip(corners, ids.flatten(), strict=True)]
    clean = pad_estimator.select_pad(
        pad_estimator.estimate_pads(raw, WIDTH_PX, HEIGHT_PX, FX_PX), 0
    )
    assert clean is not None
    assert clean.residual_m < 0.05, "an undisturbed frame should agree with itself"

    corrupted = [(mid, pts.copy()) for mid, pts in raw]
    corrupted[1][1][:, 0] += 20.0
    dirty = pad_estimator.select_pad(
        pad_estimator.estimate_pads(corrupted, WIDTH_PX, HEIGHT_PX, FX_PX), 0
    )
    assert dirty is not None
    assert dirty.residual_m > clean.residual_m * 3.0, (
        f"a displaced marker must show up as disagreement: clean "
        f"{clean.residual_m * 1000:.1f} mm, corrupted {dirty.residual_m * 1000:.1f} mm"
    )


def test_pads_do_not_impersonate_each_other(detector, textures):
    """Three pads within metres of each other must stay distinguishable.

    Marker disambiguation is the documented behaviour, and it only means
    anything if each pad's markers are unique to it.
    """
    for pad_index in range(pad_layout.PAD_COUNT):
        observation = observe(detector, render(textures[pad_index], 3.0), pad_index)
        assert observation is not None, f"pad {pad_index} not detected"
        assert observation.pad_index == pad_index

        for other in range(pad_layout.PAD_COUNT):
            if other == pad_index:
                continue
            assert observe(detector, render(textures[pad_index], 3.0), other) is None, (
                f"pad {pad_index}'s texture was reported as pad {other}"
            )


def test_a_rotated_pad_is_handled(detector, textures):
    """The drone's heading is not tied to the pad's, so the pad appears rotated.

    The homography absorbs rotation exactly, which is one of the reasons the
    estimate is built from corners rather than from a marker centroid and an
    assumed scale.
    """
    for rotation_deg in (0.0, 17.0, 45.0, 90.0, 137.0):
        observation = observe(detector, render(textures[0], 3.0, 0.20, 0.10, rotation_deg), 0)
        assert observation is not None, f"no detection at {rotation_deg} degrees"
        error_m = math.hypot(observation.offset_m[0] - 0.20, observation.offset_m[1] - 0.10)
        assert error_m < 0.06, (
            f"rotation {rotation_deg} deg moved the estimate by {error_m * 1000:.0f} mm"
        )


def test_a_clipped_marker_is_discarded(detector, textures):
    """A marker cut by the frame edge must not contribute.

    Its clipped corner is wherever the crop fell, not where the marker is, so
    its homography is confidently wrong. Discarding it is the difference between
    degrading and lying.
    """
    # Far enough off-centre that the outer ring is cut by the frame.
    image = render(textures[0], 2.5, 1.2, 0.0)
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        pytest.skip("nothing detected in this geometry; the clipping test needs a detection")

    raw = [(int(i), c[0]) for c, i in zip(corners, ids.flatten(), strict=True)]
    observation = pad_estimator.select_pad(
        pad_estimator.estimate_pads(raw, WIDTH_PX, HEIGHT_PX, FX_PX), 0
    )
    if observation is None:
        return  # every marker was clipped; refusing the frame entirely is correct

    for marker_id in observation.marker_ids:
        points = next(pts for mid, pts in raw if mid == marker_id)
        assert points[:, 0].min() >= 1.0 and points[:, 1].min() >= 1.0
        assert points[:, 0].max() <= WIDTH_PX - 2.0 and points[:, 1].max() <= HEIGHT_PX - 2.0
