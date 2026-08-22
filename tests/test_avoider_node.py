"""
Unit tests for avoider_node.py - tests the fix for UnboundLocalError.
"""

from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from avoider_node import decide_action
import config


def test_avoider_logic():
    """
    Test the avoider state machine logic directly without ROS dependencies.
    This verifies the UnboundLocalError fix by using the real decide_action function.
    """
    import time
    
    # Use real constants from config
    SAFE_DIST = config.SAFE_DIST
    CLEAR_DIST = config.CLEAR_DIST
    CLEAR_CONFIRM_S = config.CLEAR_CONFIRM_S
    MIN_LOCK_S = config.MIN_LOCK_S
    ESCALATION_LOCK_S = config.ESCALATION_LOCK_S
    
    # ── Test 1: First obstacle detection ──────────────────────────────────
    dodge_dir = None
    dodge_start_t = 0.0
    clear_first_seen = 0.0
    eff_front = 5.0  # Obstacle detected (< SAFE_DIST)
    now = time.time()
    med_left = 10.0
    med_right = 8.0
    
    action, dodge_dir, dodge_start_t, clear_first_seen = decide_action(
        eff_front, med_left, med_right, now,
        dodge_dir, dodge_start_t, clear_first_seen,
        SAFE_DIST, CLEAR_DIST, CLEAR_CONFIRM_S, MIN_LOCK_S, ESCALATION_LOCK_S
    )
    
    # Verify action was set (this is where the bug was)
    assert action == "DODGE_LEFT", f"Expected DODGE_LEFT, got {action}"
    assert dodge_dir == "DODGE_LEFT", f"Expected dodge_dir DODGE_LEFT, got {dodge_dir}"
    print("✅ First obstacle detection logic works correctly")
    
    # ── Test 2: Continuing dodge ──────────────────────────────────────────
    now2 = now + 1.0  # 1 second later
    eff_front = 5.0  # Still blocked
    
    action, dodge_dir, dodge_start_t, clear_first_seen = decide_action(
        eff_front, med_left, med_right, now2,
        dodge_dir, dodge_start_t, clear_first_seen,
        SAFE_DIST, CLEAR_DIST, CLEAR_CONFIRM_S, MIN_LOCK_S, ESCALATION_LOCK_S
    )
    
    assert action == "DODGE_LEFT", f"Expected DODGE_LEFT on second call, got {action}"
    print("✅ Continued dodge logic works correctly")
    
    # ── Test 3: Escalation after prolonged dodge ──────────────────────────
    now3 = now + ESCALATION_LOCK_S + 1.0  # Past escalation threshold
    
    action, dodge_dir, dodge_start_t, clear_first_seen = decide_action(
        eff_front, med_left, med_right, now3,
        dodge_dir, dodge_start_t, clear_first_seen,
        SAFE_DIST, CLEAR_DIST, CLEAR_CONFIRM_S, MIN_LOCK_S, ESCALATION_LOCK_S
    )
    
    assert action == "ESCALATE", f"Expected ESCALATE after {ESCALATION_LOCK_S}s, got {action}"
    print("✅ Escalation logic works correctly")
    
    # ── Test 4: Clear path when no obstacle ───────────────────────────────
    dodge_dir = None
    dodge_start_t = 0.0
    clear_first_seen = 0.0
    eff_front = 15.0  # No obstacle
    now4 = time.time()
    
    action, dodge_dir, dodge_start_t, clear_first_seen = decide_action(
        eff_front, med_left, med_right, now4,
        dodge_dir, dodge_start_t, clear_first_seen,
        SAFE_DIST, CLEAR_DIST, CLEAR_CONFIRM_S, MIN_LOCK_S, ESCALATION_LOCK_S
    )
    
    assert action == "CLEAR", f"Expected CLEAR with no obstacle, got {action}"
    print("✅ Clear path logic works correctly")
    
    return True


if __name__ == "__main__":
    try:
        result = test_avoider_logic()
        print("\n🎉 All avoider tests passed!")
        sys.exit(0 if result else 1)
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
