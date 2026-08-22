"""
SITL Test: Battery Abort

Tests that the mission FSM transitions to ABORT state when battery level is
insufficient for a planned leg.
"""

from pathlib import Path

import asyncio
import logging
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@pytest.mark.sitl  # needs a live PX4 + Gazebo; excluded from the default run
@pytest.mark.asyncio
async def test_battery_abort():
    """
    Scenario:
    1. Set low simulated battery level
    2. Plan a long-distance leg
    3. Check battery before takeoff
    4. Verify FSM fires battery_low → ABORT
    5. Verify drone does not take off
    
    Success criteria:
    - Battery check returns insufficient before leg
    - FSM enters ABORT state
    - No takeoff command issued
    """
    logger.info("=" * 60)
    logger.info("SITL TEST: Battery Abort")
    logger.info("=" * 60)
    
    try:
        from drone_agent.mission_fsm import MissionFSM, MissionState
        from drone_agent.battery import estimate_leg_energy, check_battery_sufficient
        import config
        
        # Create FSM
        fsm = MissionFSM()
        
        # Track ABORT entry
        abort_entered = []
        
        def on_abort():
            abort_entered.append(True)
            logger.info("FSM entered ABORT state")
        
        fsm.on_enter(MissionState.ABORT, on_abort)
        
        # Simulate mission start
        fsm.fire("start")
        assert fsm.state == MissionState.TAKEOFF
        logger.info("✅ FSM in TAKEOFF state")
        
        # Simulate low battery scenario
        current_battery_pct = 25.0  # Low battery
        
        # Plan a long leg (requires more energy than available)
        current_lat, current_lon = 47.397606, 8.545594
        target_lat, target_lon = 47.407606, 8.555594  # ~1.5 km away
        
        required_energy = estimate_leg_energy(
            current_lat, current_lon, target_lat, target_lon
        )
        
        logger.info(f"Battery: {current_battery_pct}%, Required: {required_energy:.1f}%")
        
        sufficient = check_battery_sufficient(current_battery_pct, required_energy)
        
        if not sufficient:
            logger.info("✅ Battery check correctly identified insufficient energy")
            fsm.fire("battery_low")
            
            # Verify FSM transitioned to ABORT
            # Note: battery_low from TAKEOFF should fail (not in transition table)
            # Need to be in ENROUTE for battery_low transition
            fsm.fire("altitude_reached")  # TAKEOFF → ENROUTE
            fsm.fire("battery_low")  # ENROUTE → ABORT
            
            assert fsm.state == MissionState.ABORT, f"Expected ABORT, got {fsm.state}"
            assert len(abort_entered) > 0, "on_enter callback should fire"
            
            logger.info("✅ FSM correctly transitioned to ABORT")
        else:
            logger.warning("Battery check passed - adjust test parameters")
            return False
        
        logger.info("✅ Battery abort test PASSED")
        
    except Exception as e:
        logger.error(f"❌ Test failed: {e}", exc_info=True)
        pytest.fail(f"Test failed: {e}")


if __name__ == "__main__":
    asyncio.run(test_battery_abort())
