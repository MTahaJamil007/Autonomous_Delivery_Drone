# config.py
"""
Central configuration file for the autonomous delivery drone fleet.
All tunable constants live here to avoid scattered hardcoded values.
"""

# ── Fleet Configuration ──────────────────────────────────────────────────────
FLEET_SIZE = 1                       # Single drone for testing (change to 3 for multi-drone with spawn_fleet.sh)
DRONE_IDS = [f"drone-{i}" for i in range(FLEET_SIZE)]
MAVSDK_PORT_BASE = 14540             # drone-i → udpin://0.0.0.0:14540+i (MAVSDK receives on this port from PX4)

# ── Per-Drone Port Configuration (R4.2) ──────────────────────────────────────
VISION_PORT_BASE = 5005              # drone-0 vision port (unchanged for backward compat)
LIDAR_PORT_BASE = 5006               # drone-0 lidar port (unchanged for backward compat)
PORT_STRIDE = 10                     # Port spacing between drones (prevents collisions)

def drone_index(drone_id: str) -> int:
    """
    Extract numeric index from drone_id (e.g., 'drone-0' → 0, 'drone-1' → 1).
    
    Args:
        drone_id: Drone identifier in format 'drone-N'
        
    Returns:
        Integer index N
        
    Raises:
        ValueError: If drone_id format is invalid
    """
    if not drone_id.startswith("drone-"):
        raise ValueError(f"Invalid drone_id format: {drone_id} (expected 'drone-N')")
    try:
        return int(drone_id.split("-")[1])
    except (IndexError, ValueError) as e:
        raise ValueError(f"Invalid drone_id format: {drone_id} (expected 'drone-N')") from e

# ── Avoider (existing values, centralized) ───────────────────────────────────
SAFE_DIST = 6.5                      # m – trigger avoidance
CLEAR_DIST = 9.0                     # m – front must exceed this to START clear timer
CLEAR_CONFIRM_S = 2.5                # s – how long front must stay > CLEAR_DIST to unlock
MIN_LOCK_S = 2.0                     # s – minimum dodge duration before checking clear
ESCALATION_LOCK_S = 12.0             # NEW — Phase 3 trigger for detour planning

# ── Obstacle Memory ──────────────────────────────────────────────────────────
OBSTACLE_MERGE_RADIUS_M = 5.0        # m – merge obstacles within this radius
OBSTACLE_DECAY_TAU_DAYS = 14         # days – confidence decay time constant
OBSTACLE_MEMORY_URL = "http://127.0.0.1:5050"

# ── Landing / Vision ─────────────────────────────────────────────────────────
CENTER_THRESH_PX = 20                # px – marker considered centered within this
SEARCH_SPIRAL_STEP_M = 2.0           # m – spiral search step size
SEARCH_SPIRAL_MAX_RADIUS_M = 10.0    # m – maximum spiral search radius
LANDING_TIMEOUT_S = 200              # s – landing search timeout (R5: increased from 45 to accommodate full spiral at 1.0 m/s)

# ── Connection & Takeoff Timeouts (R4.3) ─────────────────────────────────────
MAVSDK_CONNECT_TIMEOUT_S = 30        # s – timeout for MAVSDK connection establishment
GPS_FIX_TIMEOUT_S = 60               # s – timeout for GPS fix acquisition
TAKEOFF_TIMEOUT_S = 30               # s – timeout for reaching takeoff altitude

# ── Battery ──────────────────────────────────────────────────────────────────
BATTERY_CRITICAL_PCT = 20            # % – critical battery threshold
BATTERY_RESERVE_MARGIN_PCT = 15      # % – reserve margin for leg planning

# ── Payload ──────────────────────────────────────────────────────────────────
BAY_OFFSET_NED_M = (0.0, 0.0, 0.35)  # 35cm below drone center in NED frame

# ── Fleet Dispatch ───────────────────────────────────────────────────────────
FLEET_DISPATCH_URL = "http://127.0.0.1:5000"

# ── Navigation (migrated from drone_logic.py V3) ─────────────────────────────
TARGET_ALT = 10.0                    # m – cruise / takeoff altitude
CRUISE_SPEED = 3.5                   # m/s – maximum forward speed
SLOW_RADIUS_M = 12.0                 # m – start slowing when this close to target
MIN_SPEED = 0.8                      # m/s – minimum approach speed
ARRIVAL_M = 2.0                      # m – arrival tolerance
DODGE_SPEED = 2.5                    # m/s – sideways dodge speed
BACK_SPEED = 0.4                     # m/s – rearward component while dodging
ALT_GAIN = 0.25                      # altitude correction P-gain
ALT_MAX_VEL = 0.5                    # m/s – max vertical correction
NAV_HZ = 10                          # Hz – navigation control loop rate

# ── PX4 Tuning Parameters (Phase 4) ──────────────────────────────────────────
# These values should be applied via PX4 parameter set commands
# Documentation: https://docs.px4.io/main/en/advanced_config/parameter_reference.html
#
# Recommended values for smooth autonomous flight with obstacle avoidance:
# 
# MPC_ACC_HOR = 3.0        # Horizontal acceleration limit (m/s²)
# MPC_ACC_HOR_MAX = 5.0    # Maximum horizontal acceleration (m/s²)
# MPC_JERK_MAX = 8.0       # Maximum jerk limit (m/s³)
# MPC_XY_VEL_MAX = 4.0     # Maximum XY velocity (m/s) - matches CRUISE_SPEED
# MPC_XY_P = 0.95          # Position control P gain
# MPC_Z_VEL_MAX_DN = 1.0   # Maximum descent rate (m/s)
# MPC_Z_VEL_MAX_UP = 3.0   # Maximum ascent rate (m/s)
#
# To apply these parameters:
# 1. Connect to PX4 console: `make px4_sitl gz_x500`
# 2. In PX4 shell, run:
#    param set MPC_ACC_HOR 3.0
#    param set MPC_ACC_HOR_MAX 5.0
#    param set MPC_JERK_MAX 8.0
#    param set MPC_XY_VEL_MAX 4.0
#    param set MPC_XY_P 0.95
#    param save
# 3. Restart simulation to apply
#
# Tuning guidelines:
# - Lower MPC_ACC_HOR for smoother flight (reduce jerkiness)
# - Higher MPC_XY_P for tighter position holding (but may oscillate)
# - Match MPC_XY_VEL_MAX to CRUISE_SPEED for consistency
# - Lower MPC_JERK_MAX reduces stress on airframe and EKF
