"""
Unit tests for landing module - marker disambiguation.
"""

from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_agent.landing import select_target, EMAFilter


def test_marker_disambiguation():
    """
    Acceptance test: two markers in frame, expected_id set to one of them.
    select_target should return only the correct one across multiple frames.
    """
    # Simulate detections from vision system
    detections_frame1 = [
        {"id": 0, "err_x": 10, "err_y": 15},
        {"id": 1, "err_x": -20, "err_y": 5}
    ]
    
    detections_frame2 = [
        {"id": 1, "err_x": -18, "err_y": 7},
        {"id": 0, "err_x": 12, "err_y": 14}
    ]
    
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


def test_ema_filter_reduces_variance():
    """
    Test that EMA filter reduces variance in noisy input.
    """
    import random
    
    # Generate noisy signal (constant + noise)
    true_value = (5.0, 3.0, 2.0)
    noise_amplitude = 1.0
    
    samples = []
    for _ in range(100):
        noisy_sample = tuple(
            v + random.uniform(-noise_amplitude, noise_amplitude)
            for v in true_value
        )
        samples.append(noisy_sample)
    
    # Apply filter
    filter = EMAFilter(alpha=0.3)
    filtered_samples = [filter.update(s) for s in samples]
    
    # Compute variance of last 50 samples (after filter settles)
    raw_variance = sum(
        (s[0] - true_value[0])**2 for s in samples[-50:]
    ) / 50
    
    filtered_variance = sum(
        (s[0] - true_value[0])**2 for s in filtered_samples[-50:]
    ) / 50
    
    assert filtered_variance < raw_variance, \
        f"Filter should reduce variance: raw={raw_variance:.3f}, filtered={filtered_variance:.3f}"
    
    print(f"✅ EMA filter test passed: variance reduced from {raw_variance:.3f} to {filtered_variance:.3f}")


def test_select_target_missing_marker():
    """
    Test that select_target returns None when marker is not in detections.
    """
    detections = [
        {"id": 0, "err_x": 10, "err_y": 15},
        {"id": 1, "err_x": -20, "err_y": 5}
    ]
    
    result = select_target(detections, expected_id=2)
    assert result is None, "Should return None when marker not found"
    
    result = select_target([], expected_id=0)
    assert result is None, "Should return None for empty detections"
    
    print("✅ Missing marker test passed")


if __name__ == "__main__":
    test_marker_disambiguation()
    test_ema_filter_reduces_variance()
    test_select_target_missing_marker()
    print("\n🎉 All landing module tests passed!")
