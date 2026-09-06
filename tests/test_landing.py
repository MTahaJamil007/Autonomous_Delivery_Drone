"""
Unit tests for landing module - marker disambiguation.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_agent.landing import AlphaBetaTracker, select_target


def test_marker_disambiguation():
    """
    Acceptance test: two markers in frame, expected_id set to one of them.
    select_target should return only the correct one across multiple frames.
    """
    # Simulate detections from vision system
    detections_frame1 = [{"id": 0, "err_x": 10, "err_y": 15}, {"id": 1, "err_x": -20, "err_y": 5}]

    detections_frame2 = [{"id": 1, "err_x": -18, "err_y": 7}, {"id": 0, "err_x": 12, "err_y": 14}]

    # Test 100 simulated frames
    for i in range(100):
        if i % 2 == 0:
            detections = detections_frame1
        else:
            detections = detections_frame2

        # Looking for marker ID 1
        result = select_target(detections, expected_id=1)

        assert result is not None, f"Frame {i}: Marker 1 should be found"
        assert result["id"] == 1, f"Frame {i}: Wrong marker ID returned"

        # Looking for marker ID 0
        result = select_target(detections, expected_id=0)

        assert result is not None, f"Frame {i}: Marker 0 should be found"
        assert result["id"] == 0, f"Frame {i}: Wrong marker ID returned"

    print("✅ Marker disambiguation test passed: 100 consecutive frames, zero incorrect picks")


def test_tracker_reduces_variance():
    """The tracker that replaced the EMA must still smooth a noisy measurement."""
    import random

    rng = random.Random(7)
    true_value = (5.0, 3.0)
    noise_amplitude = 1.0
    dt_s = 0.1

    samples = [
        tuple(v + rng.uniform(-noise_amplitude, noise_amplitude) for v in true_value)
        for _ in range(100)
    ]

    tracker = AlphaBetaTracker()
    filtered = [tracker.update(s, dt_s) for s in samples]

    raw_variance = sum((s[0] - true_value[0]) ** 2 for s in samples[-50:]) / 50
    filtered_variance = sum((s[0] - true_value[0]) ** 2 for s in filtered[-50:]) / 50

    assert filtered_variance < raw_variance, (
        f"the tracker should reduce variance: raw={raw_variance:.3f}, "
        f"filtered={filtered_variance:.3f}"
    )


def test_tracker_coasts_through_a_dropped_frame():
    """THE CAPABILITY THE EMA DID NOT HAVE.

    An EMA has only a position state, so a frame with no detection leaves it
    holding a stale value and the loop treats the gap as a lost lock. The
    tracker carries a velocity state, so it extrapolates -- which is what turns
    a dropped frame into a small prediction error instead of an abort.
    """
    tracker = AlphaBetaTracker()
    dt_s = 0.1

    # A target closing at a steady 0.5 m/s along the first axis.
    for i in range(40):
        tracker.update((1.0 - 0.5 * i * dt_s, 0.0), dt_s)

    assert tracker.velocity[0] < -0.3, (
        f"the tracker should have learned the closing rate, got {tracker.velocity[0]:.3f} m/s"
    )

    before = tracker.position[0]
    coasted = tracker.predict(dt_s)
    assert coasted is not None
    assert coasted[0] < before, "predict() must advance the estimate along its velocity"
    assert abs(coasted[0] - (before - 0.5 * dt_s)) < 0.02, (
        "the extrapolation should land within 2 cm of where the target actually went"
    )


def test_tracker_reset_discards_a_stale_lock():
    """A reacquired pad must not be averaged with where it was seconds ago."""
    tracker = AlphaBetaTracker()
    tracker.update((3.0, -2.0), 0.1)
    tracker.update((3.1, -2.1), 0.1)
    assert tracker.position is not None

    tracker.reset()
    assert tracker.position is None
    assert tracker.velocity == (0.0, 0.0)

    first = tracker.update((0.2, 0.1), 0.1)
    assert first == (0.2, 0.1), "the first sample after a reset is the estimate"


def test_select_target_missing_marker():
    """
    Test that select_target returns None when marker is not in detections.
    """
    detections = [{"id": 0, "err_x": 10, "err_y": 15}, {"id": 1, "err_x": -20, "err_y": 5}]

    result = select_target(detections, expected_id=2)
    assert result is None, "Should return None when marker not found"

    result = select_target([], expected_id=0)
    assert result is None, "Should return None for empty detections"

    print("✅ Missing marker test passed")


if __name__ == "__main__":
    test_marker_disambiguation()
    test_tracker_reduces_variance()
    test_tracker_coasts_through_a_dropped_frame()
    test_tracker_reset_discards_a_stale_lock()
    test_select_target_missing_marker()
    print("\n🎉 All landing module tests passed!")
