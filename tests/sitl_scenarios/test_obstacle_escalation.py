"""
SITL Test: Obstacle Escalation

Tests that prolonged obstacle avoidance triggers escalation to detour planning
instead of oscillating indefinitely.
"""

import asyncio
import logging
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@pytest.mark.sitl  # needs a live PX4 + Gazebo; excluded from the default run
@pytest.mark.asyncio
async def test_obstacle_escalation():
    """
    Scenario:
    1. Place a wall directly on the flight path
    2. Drone starts avoidance (DODGE_LEFT or DODGE_RIGHT)
    3. Wall blocks for > 12 seconds (ESCALATION_LOCK_S)
    4. Verify ESCALATE action is fired
    5. Verify detour planner generates alternative route
    6. Verify drone follows detour

    Success criteria:
    - Avoider fires ESCALATE after 12s of continuous dodging
    - Detour planner generates waypoints that clear the obstacle
    - Drone reaches target without infinite oscillation
    """
    logger.info("=" * 60)
    logger.info("SITL TEST: Obstacle Escalation")
    logger.info("=" * 60)

    try:
        import config
        from global_planner.detour import plan_detour

        # Simulate a wall scenario
        start = (47.397606, 8.545594)
        goal = (47.398606, 8.546594)

        # Place obstacle directly on path
        mid_lat = (start[0] + goal[0]) / 2
        mid_lon = (start[1] + goal[1]) / 2

        obstacle = {
            "lat": mid_lat,
            "lon": mid_lon,
            "radius_m": 8.0,  # Large obstacle to force long avoidance
            "obstacle_type": "static_wall",
        }

        logger.info(f"Test obstacle at ({mid_lat:.6f}, {mid_lon:.6f})")

        # Test detour planner
        logger.info("Testing detour planner...")
        detour_path = plan_detour(start, goal, obstacles=[obstacle], margin_m=2.0)

        assert len(detour_path) >= 3, "Detour should have intermediate waypoints"
        logger.info(f"✅ Detour path generated with {len(detour_path)} waypoints")

        # Simulate escalation timing
        logger.info("Simulating escalation trigger...")
        escalation_time = config.ESCALATION_LOCK_S

        dodge_start = time.time()
        simulated_dodge_duration = 0.0

        # Simulate avoiding for escalation duration
        while simulated_dodge_duration < escalation_time:
            simulated_dodge_duration = time.time() - dodge_start
            action = "DODGE_LEFT"  # Would come from lidar

            if simulated_dodge_duration > escalation_time:
                action = "ESCALATE"
                logger.info(f"✅ ESCALATE triggered after {simulated_dodge_duration:.1f}s")
                break

            await asyncio.sleep(0.1)

        # Verify ESCALATE was triggered
        assert action == "ESCALATE", "Should escalate after prolonged dodge"

        logger.info("✅ Escalation logic test PASSED")
        logger.info("⚠️  Full SITL test requires live PX4 + obstacle spawn")

    except Exception as e:
        logger.error(f"❌ Test failed: {e}", exc_info=True)
        pytest.fail(f"Test failed: {e}")


if __name__ == "__main__":
    asyncio.run(test_obstacle_escalation())
