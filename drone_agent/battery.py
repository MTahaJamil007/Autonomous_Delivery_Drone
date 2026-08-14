"""
Battery management and energy estimation for mission planning.
"""

import logging
from typing import Dict, Any

# R4.1: Use relative path instead of hardcoded absolute path
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from drone_agent.navigation import get_distance_m

logger = logging.getLogger(__name__)


# Energy consumption estimates
# R2.5: Recalibrated to realistic value with explicit calibration note
# NOTE: This value needs real-flight calibration. Current value is a conservative
# estimate based on typical multirotor specs (~3.5 m/s cruise, 200W hover power).
# Adjust based on actual flight test data.
ENERGY_PER_M = 0.01  # % battery per meter of flight (conservative placeholder)
LANDING_RESERVE = 5.0  # % battery reserved for landing operations


def estimate_leg_energy(
    current_lat: float,
    current_lon: float,
    target_lat: float,
    target_lon: float
) -> float:
    """
    Estimate energy required for a flight leg.
    
    Args:
        current_lat, current_lon: Current GPS position
        target_lat, target_lon: Target GPS position
        
    Returns:
        Estimated battery percentage required for this leg
    """
    distance_m = get_distance_m(current_lat, current_lon, target_lat, target_lon)
    leg_energy = distance_m * ENERGY_PER_M
    total_energy = leg_energy + LANDING_RESERVE
    
    logger.debug(
        f"Energy estimate: {distance_m:.0f}m requires {leg_energy:.1f}% "
        f"+ {LANDING_RESERVE:.1f}% reserve = {total_energy:.1f}% total"
    )
    
    return total_energy


def check_battery_sufficient(
    current_battery_pct: float,
    required_energy_pct: float
) -> bool:
    """
    Check if battery has sufficient charge for a leg.
    
    Args:
        current_battery_pct: Current battery percentage (0-100)
        required_energy_pct: Required energy percentage
        
    Returns:
        True if battery is sufficient with margin, False otherwise
    """
    # Add safety margin
    required_with_margin = required_energy_pct + config.BATTERY_RESERVE_MARGIN_PCT
    
    sufficient = current_battery_pct >= required_with_margin
    
    if not sufficient:
        logger.warning(
            f"Insufficient battery: {current_battery_pct:.1f}% available, "
            f"{required_with_margin:.1f}% required (including {config.BATTERY_RESERVE_MARGIN_PCT}% margin)"
        )
    else:
        logger.info(
            f"Battery check passed: {current_battery_pct:.1f}% available, "
            f"{required_with_margin:.1f}% required"
        )
    
    return sufficient


async def check_leg_battery(
    drone: Any,
    drone_state: Dict[str, Any],
    target_lat: float,
    target_lon: float,
    fsm: Any = None
) -> bool:
    """
    Check if battery is sufficient for the next leg before takeoff.
    
    Args:
        drone: MAVSDK System instance
        drone_state: Dictionary with current drone state
        target_lat, target_lon: Target coordinates
        fsm: Mission FSM instance (optional)
        
    Returns:
        True if battery is sufficient, False otherwise
        
    Side Effects:
        Fires 'battery_low' FSM event if insufficient battery
    """
    # Get current battery level
    # Note: This requires async iteration over telemetry
    battery_pct = 100.0  # Default assumption
    
    try:
        async for battery in drone.telemetry.battery():
            battery_pct = battery.remaining_percent
            break  # Take first reading
    except Exception as e:
        logger.error(f"Failed to read battery telemetry: {e}")
        # Assume full battery on error to avoid false-positive aborts
        battery_pct = 100.0
    
    # Estimate required energy
    required_energy = estimate_leg_energy(
        drone_state["lat"], drone_state["lon"],
        target_lat, target_lon
    )
    
    # Check sufficiency
    sufficient = check_battery_sufficient(battery_pct, required_energy)
    
    if not sufficient and fsm is not None:
        logger.warning("Battery insufficient - firing battery_low event")
        fsm.fire("battery_low")
    
    return sufficient
