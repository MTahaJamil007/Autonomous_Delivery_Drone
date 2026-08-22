"""P2 acceptance: the sensor feeds say what their consumers expect.

The replay test is the important one. It drives the avoider's state machine
through CLEAR -> DODGE -> ESCALATE -> CLEAR from a recorded scan sequence with
NO wall-clock dependence at all: time is a parameter, so a twelve-second
escalation is exercised in microseconds and the test cannot flake on a slow
machine or an NTP step.
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

import config
from avoider_node import (
    CLEAR,
    DODGE_LEFT,
    DODGE_RIGHT,
    ESCALATE,
    clean_range,
    decide_action,
    median,
    sector_ranges,
    summarize_scan,
    validate_scan_geometry,
)

# ─────────────────────────────────────────────────────────────────────────────
#  Scan synthesis
# ─────────────────────────────────────────────────────────────────────────────


def make_scan(
    front_m: float, left_m: float = 12.0, right_m: float = 12.0, samples: int = config.LIDAR_SAMPLES
) -> list[float]:
    """A synthetic 360-sample scan with the given distance in each sector.

    Index i corresponds to angle_min + i * increment, matching the real message
    layout, so the sector maths under test is genuinely exercised rather than
    bypassed.
    """
    increment = (config.LIDAR_ANGLE_MAX_RAD - config.LIDAR_ANGLE_MIN_RAD) / samples
    ranges = []
    for i in range(samples):
        angle_deg = math.degrees(config.LIDAR_ANGLE_MIN_RAD + i * increment)
        if -config.FRONT_HALF_DEG <= angle_deg <= config.FRONT_HALF_DEG:
            ranges.append(front_m)
        elif config.SIDE_START_DEG <= angle_deg <= config.SIDE_END_DEG:
            ranges.append(left_m)
        elif -config.SIDE_END_DEG <= angle_deg <= -config.SIDE_START_DEG:
            ranges.append(right_m)
        else:
            ranges.append(float("inf"))
    return ranges


def summarize(ranges: list[float]) -> tuple[float, float, float]:
    increment = (config.LIDAR_ANGLE_MAX_RAD - config.LIDAR_ANGLE_MIN_RAD) / len(ranges)
    return summarize_scan(ranges, config.LIDAR_ANGLE_MIN_RAD, increment)


# ─────────────────────────────────────────────────────────────────────────────
#  P2 acceptance: the replay test
# ─────────────────────────────────────────────────────────────────────────────


def test_replay_clear_dodge_escalate_clear_without_wall_clock():
    """A recorded scan sequence drives the full state cycle deterministically.

    `now` is supplied by the test, so nothing here reads a clock. That is the
    point: ESCALATION_LOCK_S is 12 seconds, and a test that actually waited
    twelve seconds would be skipped in practice and would flake when it was not.
    """
    # (elapsed_s, front_m) -- a wall closes in, blocks for well over the
    # escalation window, then clears and stays clear.
    script = [
        (0.0, 15.0),
        (0.1, 15.0),  # open road
        (0.2, 5.0),  # wall detected -> dodge
        (1.0, 5.0),
        (5.0, 5.0),
        (11.0, 5.0),  # still blocked, not yet 12 s
        (12.5, 5.0),
        (14.0, 5.0),  # past the window -> escalate
        (14.5, 15.0),  # front opens
        (15.0, 15.0),
        (16.0, 15.0),  # confirming (needs 2.5 s)
        (17.5, 15.0),  # confirmed -> clear
    ]

    dodge_dir = None
    dodge_start_t = 0.0
    clear_first_seen = 0.0
    observed = []

    for elapsed_s, front_m in script:
        eff_front, med_left, med_right = summarize(make_scan(front_m, left_m=12.0, right_m=8.0))
        action, dodge_dir, dodge_start_t, clear_first_seen = decide_action(
            eff_front,
            med_left,
            med_right,
            elapsed_s,
            dodge_dir,
            dodge_start_t,
            clear_first_seen,
            config.SAFE_DIST,
            config.CLEAR_DIST,
            config.CLEAR_CONFIRM_S,
            config.MIN_LOCK_S,
            config.ESCALATION_LOCK_S,
        )
        observed.append((elapsed_s, action))

    actions = [a for _, a in observed]

    assert actions[0] == CLEAR, f"open road must be CLEAR, got {actions[0]}"
    assert actions[2] == DODGE_LEFT, (
        f"left sector is more open (12 m vs 8 m) so the dodge must go left, got {actions[2]}"
    )
    # Not yet escalated at 11 s.
    assert actions[5] == DODGE_LEFT, f"11 s < 12 s must still be a dodge, got {actions[5]}"
    # Escalated past the window.
    assert actions[6] == ESCALATE, f"12.5 s must escalate, got {actions[6]}"
    assert actions[7] == ESCALATE, (
        "ESCALATE is level-triggered, not edge-triggered: UDP is lossy, so a "
        "once-only signal that got dropped would never fire again"
    )
    # Clearing takes CLEAR_CONFIRM_S of sustained clear front.
    assert actions[8] == DODGE_LEFT, "clearing must be confirmed, not instant"
    assert actions[-1] == CLEAR, f"a sustained clear front must unlock, got {actions[-1]}"

    assert ESCALATE in actions, "the whole cycle must include an escalation"
    assert observed[-1][0] < 20.0, "the whole 17.5 s scenario runs instantly"


def test_dodge_direction_follows_the_more_open_side():
    """The committed side must be the one with more room."""
    for left_m, right_m, expected in [
        (12.0, 4.0, DODGE_LEFT),
        (4.0, 12.0, DODGE_RIGHT),
        (8.0, 8.0, DODGE_LEFT),
    ]:  # tie -> left
        eff_front, med_left, med_right = summarize(make_scan(5.0, left_m, right_m))
        action, *_ = decide_action(
            eff_front,
            med_left,
            med_right,
            0.0,
            None,
            0.0,
            0.0,
            config.SAFE_DIST,
            config.CLEAR_DIST,
            config.CLEAR_CONFIRM_S,
            config.MIN_LOCK_S,
            config.ESCALATION_LOCK_S,
        )
        assert action == expected, (
            f"left={left_m} right={right_m} should give {expected}, got {action}"
        )


def test_dodge_does_not_unlock_before_min_lock():
    """One lucky clear reading must not abort a dodge that just started."""
    eff_front, med_left, med_right = summarize(make_scan(5.0))
    action, dodge_dir, start_t, clear_seen = decide_action(
        eff_front,
        med_left,
        med_right,
        100.0,
        None,
        0.0,
        0.0,
        config.SAFE_DIST,
        config.CLEAR_DIST,
        config.CLEAR_CONFIRM_S,
        config.MIN_LOCK_S,
        config.ESCALATION_LOCK_S,
    )
    assert dodge_dir is not None

    # Front looks wide open only 0.5 s later - well inside MIN_LOCK_S.
    clear_front, left, right = summarize(make_scan(15.0))
    action, dodge_dir, start_t, clear_seen = decide_action(
        clear_front,
        left,
        right,
        100.5,
        dodge_dir,
        start_t,
        clear_seen,
        config.SAFE_DIST,
        config.CLEAR_DIST,
        config.CLEAR_CONFIRM_S,
        config.MIN_LOCK_S,
        config.ESCALATION_LOCK_S,
    )
    assert action != CLEAR, "MIN_LOCK_S must hold the dodge"


def test_hysteresis_band_keeps_dodging():
    """Between SAFE_DIST and CLEAR_DIST is neither dangerous nor confirmed clear.

    That gap is the whole anti-oscillation mechanism: without it the drone
    ping-pongs at a wall edge.
    """
    assert config.CLEAR_DIST > config.SAFE_DIST, "the hysteresis band must exist"
    mid_m = (config.SAFE_DIST + config.CLEAR_DIST) / 2.0

    eff, left, right = summarize(make_scan(5.0))
    _, dodge_dir, start_t, clear_seen = decide_action(
        eff,
        left,
        right,
        0.0,
        None,
        0.0,
        0.0,
        config.SAFE_DIST,
        config.CLEAR_DIST,
        config.CLEAR_CONFIRM_S,
        config.MIN_LOCK_S,
        config.ESCALATION_LOCK_S,
    )

    eff, left, right = summarize(make_scan(mid_m))
    action, _, _, _ = decide_action(
        eff,
        left,
        right,
        5.0,
        dodge_dir,
        start_t,
        clear_seen,
        config.SAFE_DIST,
        config.CLEAR_DIST,
        config.CLEAR_CONFIRM_S,
        config.MIN_LOCK_S,
        config.ESCALATION_LOCK_S,
    )
    assert action == dodge_dir, (
        f"{mid_m} m is inside the hysteresis band and must not clear the dodge"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Self-hit masking
# ─────────────────────────────────────────────────────────────────────────────


def test_propeller_returns_are_masked_not_treated_as_a_wall():
    """A 0.29 m prop return must not read as an unclearable obstacle.

    The props sit about 0.29 m out and the sensor floor is 0.15 m, so these are
    genuine returns from a real object. Without the mask the avoider locks a
    dodge it can never clear, escalates, and the mission stalls forever.
    """
    assert config.MIN_VALID_RANGE_M > 0.29, "the mask must clear the propeller radius"
    assert config.MIN_VALID_RANGE_M < config.SAFE_DIST, "the mask must not swallow real obstacles"

    assert clean_range(0.29) == config.INF_REPLACE_M
    assert clean_range(0.16) == config.INF_REPLACE_M  # above sensor floor, still a prop
    assert clean_range(7.0) == 7.0  # a real obstacle survives

    # A whole scan of prop returns must read as clear, not as a wall.
    eff_front, _, _ = summarize(make_scan(0.29))
    assert eff_front == config.INF_REPLACE_M
    action, *_ = decide_action(
        eff_front,
        15.0,
        15.0,
        0.0,
        None,
        0.0,
        0.0,
        config.SAFE_DIST,
        config.CLEAR_DIST,
        config.CLEAR_CONFIRM_S,
        config.MIN_LOCK_S,
        config.ESCALATION_LOCK_S,
    )
    assert action == CLEAR, "a scan of nothing but prop hits must read as CLEAR"


def test_empty_world_reads_as_clear():
    """P2 acceptance: hovering in an empty world, eff_front is the inf substitute."""
    eff_front, med_left, med_right = summarize([float("inf")] * config.LIDAR_SAMPLES)
    assert eff_front == config.INF_REPLACE_M == 15.0
    assert med_left == med_right == config.INF_REPLACE_M
    assert config.INF_REPLACE_M > config.CLEAR_DIST, (
        "INF_REPLACE_M must exceed CLEAR_DIST or an empty world never clears"
    )


def test_narrow_obstacle_is_not_averaged_away():
    """eff_front = min(median, min) so a thin obstacle still registers.

    A mast or a wall edge lights up only a few beams. A median alone ignores it
    entirely, which is the difference between avoiding a pole and hitting it.
    """
    ranges = [float("inf")] * config.LIDAR_SAMPLES
    # Three beams straight ahead see something close. Index 180 is angle 0.
    for i in (179, 180, 181):
        ranges[i] = 3.0

    eff_front, _, _ = summarize(ranges)
    assert eff_front == 3.0, (
        f"a 3-beam obstacle must set eff_front, got {eff_front}. The median "
        f"alone would report {config.INF_REPLACE_M}."
    )
    assert eff_front < config.SAFE_DIST, "and it must trigger a dodge"


# ─────────────────────────────────────────────────────────────────────────────
#  Geometry validation
# ─────────────────────────────────────────────────────────────────────────────


def test_geometry_validation_accepts_the_configured_sensor():
    increment = (config.LIDAR_ANGLE_MAX_RAD - config.LIDAR_ANGLE_MIN_RAD) / config.LIDAR_SAMPLES
    problems = validate_scan_geometry(
        config.LIDAR_SAMPLES,
        config.LIDAR_ANGLE_MIN_RAD,
        increment,
        config.LIDAR_RANGE_MAX_M,
    )
    assert problems == [], f"the configured sensor must validate cleanly: {problems}"


def test_geometry_validation_rejects_a_changed_sensor():
    """A model change that moves the front cone must be loud, not subtle."""
    problems = validate_scan_geometry(720, 0.0, 0.00873, 30.0)
    assert len(problems) >= 3, f"expected several complaints, got {problems}"
    joined = " ".join(problems)
    assert "sample count" in joined
    assert "angle_min" in joined
    assert "range_max" in joined


def test_sector_indices_are_derived_not_assumed():
    """Sector lookup must follow the message's own angle_min/increment."""
    samples = 720  # twice the configured resolution
    increment = 2 * math.pi / samples
    ranges = [float("inf")] * samples
    # Put a close return at exactly straight ahead for this resolution.
    ranges[samples // 2] = 4.0

    front = sector_ranges(
        ranges, -math.pi, increment, -config.FRONT_HALF_DEG, config.FRONT_HALF_DEG
    )
    assert min(front) == 4.0, "the front sector must find a straight-ahead return at any resolution"
    assert len(front) == 2 * config.FRONT_HALF_DEG + 1


def test_median_of_empty_sector_is_the_inf_substitute():
    assert median([]) == config.INF_REPLACE_M
    assert median([1.0, 5.0, 3.0]) == 3.0


# ─────────────────────────────────────────────────────────────────────────────
#  Vision contract
# ─────────────────────────────────────────────────────────────────────────────


def test_two_markers_appear_in_one_frame_with_distinct_ids():
    """P2 acceptance: both markers in frame, both reported, IDs correct.

    The old bridge sent corners[0][0] and ids[0][0] -- the first marker only --
    so with three pads within metres of each other, landing.select_target()
    could never see the one it was looking for.
    """
    import cv2

    from perception.vision_bridge import ArucoDetectorWrapper

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((240, 320, 3), 255, np.uint8)
    for marker_id, x in ((1, 20), (2, 190)):
        tile = cv2.aruco.generateImageMarker(dictionary, marker_id, 80)
        canvas[80:160, x : x + 80] = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)

    detections = ArucoDetectorWrapper().detect(canvas)
    ids = sorted(d["id"] for d in detections)
    assert ids == [1, 2], f"both markers must be reported, got {ids}"

    # And each must be selectable by id, which is what disambiguation means.
    from drone_agent.landing import select_target

    assert select_target(detections, 1)["id"] == 1
    assert select_target(detections, 2)["id"] == 2
    assert select_target(detections, 0) is None

    # Left marker is left of centre, right marker is right of centre.
    left = select_target(detections, 1)
    right = select_target(detections, 2)
    assert left["err_x"] < 0 < right["err_x"], (
        f"err_x signs are wrong: {left['err_x']}, {right['err_x']}"
    )


def test_vision_payload_is_json_serialisable_and_complete():
    """The consumer parses JSON; the producer once sent CSV.

    That mismatch shipped because no document owned the boundary.
    docs/ARCHITECTURE.md now does, and this test pins the field names.
    """
    import cv2

    from perception.vision_bridge import ArucoDetectorWrapper

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((240, 320, 3), 255, np.uint8)
    tile = cv2.aruco.generateImageMarker(dictionary, 0, 100)
    canvas[70:170, 110:210] = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)

    detections = ArucoDetectorWrapper().detect(canvas)
    assert len(detections) == 1

    payload = {
        "t": 123.456,
        "seq": 7,
        "w": 320,
        "h": 240,
        "fx": config.CAMERA_FX_PX,
        "drone_id": "drone-0",
        "detections": detections,
    }
    round_tripped = json.loads(json.dumps(payload))

    detection = round_tripped["detections"][0]
    for field in ("id", "err_x", "err_y", "area_px", "size_px", "corners"):
        assert field in detection, f"contract field {field!r} missing"
    assert len(detection["corners"]) == 4


def test_empty_detection_frames_are_still_reported():
    """ "No marker" and "bridge dead" must be distinguishable.

    An empty detections list is a positive statement that the camera is alive
    and sees nothing. Sending nothing at all is indistinguishable from a crashed
    process, which is why the receiver's staleness check needs these frames.
    """
    from perception.vision_bridge import ArucoDetectorWrapper

    blank = np.full((240, 320, 3), 255, np.uint8)
    assert ArucoDetectorWrapper().detect(blank) == []


# ─────────────────────────────────────────────────────────────────────────────
#  Fail-closed staleness
# ─────────────────────────────────────────────────────────────────────────────


def test_never_heard_from_counts_as_stale():
    """The fail-closed guard's core case: no data is not "clear".

    navigation.py already intended this, but nothing wrote last_lidar_ts, so the
    guard was permanently engaged and the drone hovered forever instead of ever
    flying. Both halves have to be right.
    """
    from drone_agent import udp_receiver

    empty: dict = {}
    assert udp_receiver.is_stale(empty, udp_receiver.LIDAR_TS_KEY)
    assert not udp_receiver.has_ever_arrived(empty, udp_receiver.LIDAR_TS_KEY)
    assert udp_receiver.age_s(empty, udp_receiver.LIDAR_TS_KEY) == float("inf")


def test_fresh_then_stale_transition():
    from drone_agent import udp_receiver

    state = {udp_receiver.LIDAR_TS_KEY: 1000.0}
    assert not udp_receiver.is_stale(state, udp_receiver.LIDAR_TS_KEY, now=1000.5)
    assert udp_receiver.is_stale(state, udp_receiver.LIDAR_TS_KEY, now=1002.0)
    assert udp_receiver.has_ever_arrived(state, udp_receiver.LIDAR_TS_KEY), (
        "a feed that has stopped is different from one that never started - the "
        "supervisor's heartbeat check depends on that distinction to avoid "
        "firing an emergency RTL at t=0"
    )
