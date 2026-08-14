"""
Navigation module - NED-frame velocity control for waypoint navigation with avoidance.

Migrated from drone_logic.py V3 block with FSM event integration.
"""

import asyncio
import math
import time
import logging
from typing import Tuple, Dict, Any, Callable
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# R4.1: Use relative path instead of hardcoded absolute path
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config

logger = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════════
#  FIXED-RATE CONTROL LOOP
# ════════════════════════════════════════════════════════════════════════════

async def fixed_rate_loop(hz: float, tick_fn: Callable) -> None:
    """
    Execute tick_fn at a fixed rate with drift compensation.
    
    Unlike asyncio.sleep(dt) which accumulates drift, this scheduler uses
    monotonic time to maintain consistent loop timing over long durations.
    
    Args:
        hz: Target frequency in Hertz
        tick_fn: Async function to call each tick (should return False to stop loop)
    """
    period = 1.0 / hz
    next_tick = time.monotonic()
    
    while True:
        # Execute the tick function
        should_continue = await tick_fn()
        if should_continue is False:
            break
        
        # Calculate next tick time
        next_tick += period
        
        # Sleep until next tick (compensates for processing time and drift)
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            # We're running behind schedule - log warning and catch up
            if sleep_time < -period:
                logger.warning(f"Control loop behind schedule by {-sleep_time:.3f}s")


# ════════════════════════════════════════════════════════════════════════════
#  GEOMETRY HELPERS
# ════════════════════════════════════════════════════════════════════════════

def get_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth distance in metres (accurate within ~10 km)."""
    d_lat = (lat2 - lat1) * 111_320.0
    d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
    return math.hypot(d_lat, d_lon)


def get_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compass bearing in degrees (0 = North, 90 = East)."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def bearing_to_ned(bearing_deg: float, speed_ms: float) -> Tuple[float, float]:
    """Convert compass bearing + speed → (north_m_s, east_m_s)."""
    r = math.radians(bearing_deg)
    return speed_ms * math.cos(r), speed_ms * math.sin(r)


def dodge_ned(bearing_deg: float, action: str) -> Tuple[float, float]:
    """
    Return (north, east) velocity for a sideways dodge, PLUS a tiny backward
    component to maintain separation from the wall.

    LEFT/RIGHT are from the drone's perspective (i.e. relative to its bearing).
    """
    fwd_r = math.radians(bearing_deg)

    if action == "DODGE_LEFT":
        side_r = math.radians(bearing_deg - 90.0)
    else:  # DODGE_RIGHT
        side_r = math.radians(bearing_deg + 90.0)

    # sideways
    n = config.DODGE_SPEED * math.cos(side_r)
    e = config.DODGE_SPEED * math.sin(side_r)

    # tiny rearward component
    n -= config.BACK_SPEED * math.cos(fwd_r)
    e -= config.BACK_SPEED * math.sin(fwd_r)

    return n, e


def blend_velocity(
    current: Tuple[float, float],
    target: Tuple[float, float],
    t: float
) -> Tuple[float, float]:
    """
    Linear interpolation between current and target velocities.
    
    Used for smooth transitions when avoidance state changes (e.g., CLEAR to
    DODGE_LEFT or vice versa). Prevents single-tick velocity discontinuities
    that can stress the attitude controller.
    
    Args:
        current: Current (north, east) velocity in m/s
        target: Target (north, east) velocity in m/s
        t: Blend factor in [0, 1]. 0 = fully current, 1 = fully target
        
    Returns:
        Blended (north, east) velocity
    """
    return tuple(c + (g - c) * t for c, g in zip(current, target))


def altitude_correction(drone_state: Dict[str, Any]) -> float:
    """Small down-velocity correction to hold TARGET_ALT (NED: negative = up)."""
    err = config.TARGET_ALT - drone_state["alt"]  # positive → drone too low
    raw = -err * config.ALT_GAIN  # negative → ascend
    return max(-config.ALT_MAX_VEL, min(config.ALT_MAX_VEL, raw))


# ════════════════════════════════════════════════════════════════════════════
#  CORE NAVIGATION WITH AVOIDANCE
# ════════════════════════════════════════════════════════════════════════════

async def navigate_with_avoidance(
    drone: System,
    target_lat: float,
    target_lon: float,
    drone_state: Dict[str, Any],
    lidar_data: Dict[str, str],
    fsm: Any = None
) -> None:
    """
    Fly to (target_lat, target_lon) in OFFBOARD mode using world-frame
    (NED) velocity setpoints with fixed-rate control loop.

    Why NED frame?
    ──────────────
    Body-frame commands (VelocityBodyYawspeed) are relative to where the
    drone's NOSE is pointing. When PX4 brakes, the nose pitches up.
    A 2-D LiDAR mounted flat then stares at the sky, returns ∞, and the
    avoider incorrectly declares the path clear.

    NED commands are always in the WORLD frame. A pitching nose has zero
    effect on a north/east/down velocity command – PX4's attitude controller
    works out the required tilt internally. This completely eliminates
    pitch-blindness.

    Control law
    ───────────
    • Proportional speed (slow down smoothly near target).
    • When avoider says DODGE_*: replace the forward NED component with a
      perpendicular one – no mode switch, no braking impulse.
    • Altitude held via a small P-controller on the down axis.
    • Fixed-rate 10 Hz loop (eliminates drift).
    
    FSM Integration
    ───────────────
    • Fires 'arrived' event when within ARRIVAL_M of target
    • Fires 'battery_low' event if battery check fails (future implementation)
    """
    logger.info(f"Navigating to ({target_lat:.6f}, {target_lon:.6f})")
    drone_state["status"] = "Navigating"

    # Prime offboard with a zero setpoint, then start it
    zero = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_ned(zero)
    try:
        await drone.offboard.start()
    except OffboardError as e:
        logger.warning(f"Offboard start error (may already be running): {e}")

    # Blending state
    current_ned_vel = (0.0, 0.0)  # Track current NED velocity for blending
    blend_duration_s = 0.4  # Blend over 400ms
    blend_ticks = int(blend_duration_s * config.NAV_HZ)
    blend_counter = 0
    last_action = "CLEAR"

    # Navigation tick function
    async def nav_tick() -> bool:
        """Single navigation control tick. Returns False to stop loop."""
        
        # R5 BUG 2 FIX: Check lidar staleness (fail-closed behavior)
        LIDAR_STALE_TIMEOUT_S = 1.0  # avoider_node publishes far faster than this in normal operation
        
        last_lidar_ts = lidar_data.get("last_lidar_ts", 0)
        lidar_is_stale = (time.time() - last_lidar_ts) > LIDAR_STALE_TIMEOUT_S
        
        if lidar_is_stale:
            # FAIL CLOSED: do not assume CLEAR if we have never heard from the avoider,
            # or haven't heard from it recently. Hold position and raise the alarm.
            drone_state["status"] = "⛔ OBSTACLE FEED DEAD — holding position"
            if not hasattr(nav_tick, "_warned_stale"):
                logger.error(
                    "[NAV] No lidar/avoider data received — is avoider_node.py and the "
                    "ros_gz_bridge running? Holding position instead of flying blind. "
                    "See HOW_TO_RUN.md Terminal 4 and 5."
                )
                nav_tick._warned_stale = True
            
            # Hold position: zero velocity
            bearing = get_bearing(
                drone_state["lat"], drone_state["lon"],
                target_lat, target_lon
            )
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, bearing)
            )
            return True  # keep looping, keep holding, do not advance toward target
        
        # ── arrival check ────────────────────────────────────────────────────
        dist_m = get_distance_m(
            drone_state["lat"], drone_state["lon"],
            target_lat, target_lon
        )

        if dist_m <= config.ARRIVAL_M:
            # Hover in place to stabilise before landing
            await drone.offboard.set_velocity_ned(zero)
            await asyncio.sleep(0.5)
            logger.info(f"Arrived at target ({dist_m:.1f} m)")
            await drone.offboard.stop()
            
            # Fire FSM event if FSM is provided
            if fsm is not None:
                fsm.fire("arrived")
            
            return False  # Stop the loop

        # ── compute bearing & proportional cruise speed ─────────────────────
        bearing = get_bearing(
            drone_state["lat"], drone_state["lon"],
            target_lat, target_lon
        )
        speed = min(
            config.CRUISE_SPEED,
            max(config.MIN_SPEED, config.CRUISE_SPEED * (dist_m / config.SLOW_RADIUS_M))
        )

        # ── pick NED velocity based on lidar state ───────────────────────────
        nonlocal current_ned_vel, blend_counter, last_action
        
        action = lidar_data["action"]
        
        # Detect state transition
        if action != last_action:
            blend_counter = blend_ticks  # Start blending
            last_action = action

        # Report obstacle on first dodge lock
        if action != last_action and action in ["DODGE_LEFT", "DODGE_RIGHT"]:
            # New dodge - report obstacle
            try:
                from drone_agent.obstacle_client import report_obstacle
                import asyncio
                asyncio.create_task(report_obstacle(
                    lat=drone_state["lat"],
                    lon=drone_state["lon"],
                    source_drone=getattr(navigate_with_avoidance, "_drone_id", "drone-0"),
                    radius_m=3.0,
                    obstacle_type="static_wall",
                    confidence=0.5
                ))
                logger.info(f"🚨 Obstacle detected - reporting to fleet memory at ({drone_state['lat']:.6f}, {drone_state['lon']:.6f})")
            except Exception as e:
                logger.warning(f"Failed to report obstacle: {e}")
        
        # Compute target velocity (R2.7: Handle ESCALATE explicitly)
        if action == "ESCALATE":
            # ── ESCALATION: Detour planning required ────────────────────────
            # TODO: Integrate global_planner.detour.plan_detour here
            # For now, treat as CLEAR and log warning
            drone_state["status"] = "⚠️  ESCALATION TRIGGERED"
            logger.warning(
                "[NAV] ESCALATE action received - detour planning not yet implemented. "
                "Continuing with current velocity."
            )
            target_vel = bearing_to_ned(bearing, speed)
        elif action != "CLEAR":
            # ── AVOIDANCE ───────────────────────────────────────────────────
            drone_state["status"] = f"EVADING ({action})"
            target_vel = dodge_ned(bearing, action)
        else:
            # ── CRUISE toward target ────────────────────────────────────────
            drone_state["status"] = f"Navigating  dist={dist_m:.0f} m"
            target_vel = bearing_to_ned(bearing, speed)
        
        # Apply blending if transitioning
        if blend_counter > 0:
            blend_factor = 1.0 - (blend_counter / blend_ticks)
            n_vel, e_vel = blend_velocity(current_ned_vel, target_vel, blend_factor)
            blend_counter -= 1
        else:
            n_vel, e_vel = target_vel
        
        # Update current velocity state
        current_ned_vel = (n_vel, e_vel)
        
        # Add altitude correction
        d_vel = altitude_correction(drone_state)
        
        # Send velocity command
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(n_vel, e_vel, d_vel, bearing)
        )

        return True  # Continue the loop
    
    # Run fixed-rate control loop
    await fixed_rate_loop(config.NAV_HZ, nav_tick)


async def arm_and_takeoff(drone: System, drone_state: Dict[str, Any], fsm: Any = None) -> None:
    """
    Arm, command auto-takeoff to TARGET_ALT, then wait until the drone
    actually reaches that altitude before returning.
    
    R4.3 FIX: Added timeout to altitude wait to prevent indefinite hangs.
    R5 SECONDARY FIX 1: Wait for EKF/heading convergence before arming.
    
    FSM Integration
    ───────────────
    • Fires 'altitude_reached' event when takeoff altitude is reached
    • Fires 'fail_x2' event if takeoff fails after retries (future implementation)
    
    Raises:
        TimeoutError: If altitude not reached within TAKEOFF_TIMEOUT_S
    """
    drone_state["status"] = "Waiting for EKF convergence"
    logger.info("Waiting for EKF/heading/GPS convergence before arming...")
    
    # R5 SECONDARY FIX 1: Wait for all health checks before arming
    EKF_TIMEOUT_S = 30.0
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed > EKF_TIMEOUT_S:
            raise TimeoutError(
                f"EKF/heading/GPS did not converge after {EKF_TIMEOUT_S}s. "
                f"Check PX4 console for 'Preflight Fail' messages and ensure simulation is running properly."
            )
        
        async for health in drone.telemetry.health():
            if (health.is_global_position_ok and 
                health.is_home_position_ok and
                health.is_armable):
                logger.info("EKF converged, GPS OK, heading valid - ready to arm")
                drone_state["status"] = "Arming"
                break
            else:
                logger.debug(
                    f"Waiting for health: gps={health.is_global_position_ok}, "
                    f"home={health.is_home_position_ok}, armable={health.is_armable}"
                )
                await asyncio.sleep(0.5)
        else:
            continue  # Inner loop didn't break, continue outer loop
        break  # Inner loop broke, exit outer loop
    
    logger.info("Arming...")
    await drone.action.set_takeoff_altitude(config.TARGET_ALT)
    await drone.action.arm()
    await asyncio.sleep(1.0)

    logger.info(f"Taking off to {config.TARGET_ALT} m...")
    await drone.action.takeoff()

    # Wait for altitude with timeout (R4.3)
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        
        # Check timeout
        if elapsed > config.TAKEOFF_TIMEOUT_S:
            raise TimeoutError(
                f"Takeoff altitude not reached after {config.TAKEOFF_TIMEOUT_S}s. "
                f"Current altitude: {drone_state['alt']:.1f} m, target: {config.TARGET_ALT} m. "
                f"Check: (1) Takeoff command accepted (no preflight failures in PX4 console), "
                f"(2) EKF2 status is healthy, (3) No simulated physics issues in Gazebo."
            )
        
        alt = drone_state["alt"]
        if alt >= config.TARGET_ALT - 1.5:
            break
        logger.info(f"Altitude: {alt:.1f} m  (target {config.TARGET_ALT} m)")
        await asyncio.sleep(0.5)

    # Brief stabilisation pause
    await asyncio.sleep(2.0)
    logger.info("Altitude reached. Ready for offboard control.")
    
    # Fire FSM event if FSM is provided
    if fsm is not None:
        fsm.fire("altitude_reached")
