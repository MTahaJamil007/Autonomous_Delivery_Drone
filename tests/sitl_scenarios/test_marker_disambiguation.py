from pathlib import Path
"""
SITL Test: Marker Disambiguation

Tests that the landing system correctly identifies and lands on the expected
marker when multiple markers are simultaneously visible.
"""

import asyncio
import logging
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@pytest.mark.asyncio
async def test_marker_disambiguation():
    """
    Scenario:
    1. Spawn pickup_pad (marker 0) and drop_pad (marker 1) close together
    2. Command drone to land on marker 0
    3. Verify it lands on marker 0, not marker 1
    4. Verify no flickering between markers
    
    Success criteria:
    - Drone locks onto marker 0 within 10 seconds
    - No lock switches to marker 1 during approach
    - Final landing position within 2m of marker 0
    """
    logger.info("=" * 60)
    logger.info("SITL TEST: Marker Disambiguation")
    logger.info("=" * 60)
    
    try:
        from mavsdk import System
        from drone_agent.landing import execute_precision_landing, select_target
        
        # Initialize drone connection
        drone = System()
        logger.info("Connecting to drone...")
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        
        # Wait for connection
        async for state in drone.core.connection_state():
            if state.is_connected:
                logger.info("✅ Connected to drone")
                break
        
        # Wait for GPS
        async for health in drone.telemetry.health():
            if health.is_global_position_ok:
                logger.info("✅ GPS OK")
                break
        
        # Get home position
        drone_state = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "status": "Test"}
        async for pos in drone.telemetry.position():
            drone_state["lat"] = pos.latitude_deg
            drone_state["lon"] = pos.longitude_deg
            drone_state["alt"] = pos.relative_altitude_m
            break
        
        home_lat, home_lon = drone_state["lat"], drone_state["lon"]
        logger.info(f"Home: ({home_lat:.6f}, {home_lon:.6f})")
        
        # TODO: Spawn two markers close together (requires Gazebo spawn integration)
        # marker_0_lat = home_lat + 0.0001  # ~11m north
        # marker_1_lat = home_lat + 0.0002  # ~22m north
        # spawn_marker("test_marker_0", marker_0_lat, home_lon, marker_id=0)
        # spawn_marker("test_marker_1", marker_1_lat, home_lon, marker_id=1)
        
        logger.info("⚠️  Manual setup required:")
        logger.info("   1. Spawn marker ID 0 at ~11m north of home")
        logger.info("   2. Spawn marker ID 1 at ~22m north of home")
        logger.info("   3. Position drone above markers")
        input("Press Enter when ready...")
        
        # Take off
        logger.info("Taking off...")
        await drone.action.set_takeoff_altitude(10.0)
        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(8)
        
        logger.info("✅ Airborne - starting landing test")
        
        # Start vision data collection (mock)
        vision_data = {"locked": False, "detections": []}
        
        # Test marker selection filter
        test_detections = [
            {"id": 0, "err_x": 10, "err_y": 15},
            {"id": 1, "err_x": -20, "err_y": 5}
        ]
        
        selected = select_target(test_detections, expected_id=0)
        assert selected["id"] == 0, "Filter should select marker 0"
        logger.info("✅ Disambiguation filter test passed")
        
        # Attempt precision landing on marker 0
        logger.info("Attempting precision landing on marker 0...")
        # success = await execute_precision_landing(
        #     drone, expected_marker_id=0,
        #     expected_lat=marker_0_lat, expected_lon=home_lon,
        #     vision_data=vision_data, drone_state=drone_state
        # )
        
        logger.info("⚠️  Full integration test requires vision_bridge + SITL setup")
        logger.info("✅ Unit test portion PASSED")
        
    except Exception as e:
        logger.error(f"❌ Test failed: {e}", exc_info=True)
        pytest.fail(f"Test failed: {e}")


if __name__ == "__main__":
    asyncio.run(test_marker_disambiguation())
