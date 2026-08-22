"""
Unit tests for Mission FSM.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_agent.mission_fsm import MissionFSM, MissionState


def test_acceptance_sequence():
    """
    Acceptance test from spec: firing start, altitude_reached, arrived,
    search_exhausted in sequence should land the FSM in HOVER_AND_ALERT.
    """
    fsm = MissionFSM()

    # Initial state
    assert fsm.state == MissionState.IDLE, f"Expected IDLE, got {fsm.state}"

    # Fire: start
    result = fsm.fire("start")
    assert result is True, "start event should succeed"
    assert fsm.state == MissionState.TAKEOFF, f"Expected TAKEOFF, got {fsm.state}"

    # Fire: altitude_reached
    result = fsm.fire("altitude_reached")
    assert result is True, "altitude_reached event should succeed"
    assert fsm.state == MissionState.ENROUTE, f"Expected ENROUTE, got {fsm.state}"

    # Fire: arrived
    result = fsm.fire("arrived")
    assert result is True, "arrived event should succeed"
    assert fsm.state == MissionState.SEARCHING, f"Expected SEARCHING, got {fsm.state}"

    # Fire: search_exhausted
    result = fsm.fire("search_exhausted")
    assert result is True, "search_exhausted event should succeed"
    assert fsm.state == MissionState.HOVER_AND_ALERT, f"Expected HOVER_AND_ALERT, got {fsm.state}"

    print("✅ Acceptance test passed: FSM reached HOVER_AND_ALERT")


def test_invalid_transition():
    """
    Invalid events should return False without raising exceptions.
    """
    fsm = MissionFSM()

    # Try invalid event from IDLE
    result = fsm.fire("invalid_event")
    assert result is False, "Invalid event should return False"
    assert fsm.state == MissionState.IDLE, "State should remain IDLE after invalid event"

    print("✅ Invalid transition test passed: returned False without raising")


def test_on_enter_callback():
    """
    Test that on_enter callbacks are triggered.
    """
    fsm = MissionFSM()
    callback_triggered = []

    def on_takeoff():
        callback_triggered.append("TAKEOFF")

    fsm.on_enter(MissionState.TAKEOFF, on_takeoff)
    fsm.fire("start")

    assert "TAKEOFF" in callback_triggered, "on_enter callback should be triggered"
    print("✅ on_enter callback test passed")


if __name__ == "__main__":
    test_acceptance_sequence()
    test_invalid_transition()
    test_on_enter_callback()
    print("\n🎉 All Mission FSM tests passed!")
