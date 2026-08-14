"""
Precision landing module with vision-based marker tracking.

Provides marker disambiguation, metric offset calculation via solvePnP,
smoothing filters, and expanding search patterns for robust landing.
"""

import asyncio
import time
import logging
import math
from typing import Dict, Any, Optional, Tuple, List
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None
    logging.warning("OpenCV not available - solvePnP features disabled")

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw

# R4.1: Use relative path instead of hardcoded absolute path
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from drone_agent import navigation

logger = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════════
#  MARKER DISAMBIGUATION
# ════════════════════════════════════════════════════════════════════════════

def select_target(detections: List[Dict], expected_id: int) -> Optional[Dict]:
    """
    Returns the detection matching expected_id, or None if absent.
    
    Multiple simultaneous markers in frame are a non-issue — the filter
    happens here, not in perception.
    
    Args:
        detections: List of detection dicts with 'id', 'err_x', 'err_y' keys
        expected_id: The marker ID we're looking for
        
    Returns:
        Detection dict if found, None otherwise
    """
    return next((d for d in detections if d.get("id") == expected_id), None)


# ════════════════════════════════════════════════════════════════════════════
#  EXPONENTIAL MOVING AVERAGE FILTER
# ════════════════════════════════════════════════════════════════════════════

class EMAFilter:
    """
    Exponential Moving Average filter for smoothing noisy sensor data.
    """
    
    def __init__(self, alpha: float = 0.3):
        """
        Initialize EMA filter.
        
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive, less smooth.
        """
        self._alpha = alpha
        self._state: Optional[Tuple[float, float, float]] = None
    
    def update(self, sample: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Update filter with new sample.
        
        Args:
            sample: (x, y, z) tuple
            
        Returns:
            Filtered (x, y, z) tuple
        """
        if self._state is None:
            self._state = sample
        else:
            self._state = tuple(
                self._alpha * s + (1 - self._alpha) * p
                for s, p in zip(sample, self._state)
            )
        return self._state
    
    def reset(self) -> None:
        """Reset filter state."""
        self._state = None


# ════════════════════════════════════════════════════════════════════════════
#  METRIC OFFSET VIA solvePnP (Task 2.2 - placeholder)
# ════════════════════════════════════════════════════════════════════════════

def compute_metric_offset(
    corners: np.ndarray,
    marker_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray
) -> Optional[Tuple[float, float, float]]:
    """
    Compute metric (x, y, z) offset to marker using cv2.solvePnP.
    
    Args:
        corners: ArUco corner points in image (4x2 array)
        marker_size_m: Physical marker size in meters
        camera_matrix: Camera intrinsic matrix (3x3)
        dist_coeffs: Lens distortion coefficients
        
    Returns:
        (x, y, z) offset in meters, or None if solvePnP fails
    """
    if cv2 is None:
        return None
    
    # 3D object points for the marker (in marker's coordinate frame)
    # Assuming marker is flat and square
    half_size = marker_size_m / 2.0
    obj_points = np.array([
        [-half_size, -half_size, 0],
        [half_size, -half_size, 0],
        [half_size, half_size, 0],
        [-half_size, half_size, 0]
    ], dtype=np.float32)
    
    try:
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if success:
            # tvec is the translation vector (x, y, z) in camera frame
            return float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
    except Exception as e:
        logger.error(f"solvePnP failed: {e}")
    
    return None


# ════════════════════════════════════════════════════════════════════════════
#  EXPANDING SPIRAL SEARCH PATTERN
# ════════════════════════════════════════════════════════════════════════════

def generate_spiral_waypoints(
    center_lat: float,
    center_lon: float,
    step_m: float,
    max_radius_m: float
) -> List[Tuple[float, float]]:
    """
    Generate GPS waypoints in an expanding spiral pattern.
    
    Args:
        center_lat, center_lon: Search center coordinates
        step_m: Step size per ring
        max_radius_m: Maximum search radius
        
    Returns:
        List of (lat, lon) waypoints
    """
    waypoints = [(center_lat, center_lon)]  # Start at center
    
    num_rings = int(max_radius_m / step_m)
    
    for ring in range(1, num_rings + 1):
        radius_m = ring * step_m
        
        # Points per ring (more points for larger rings)
        num_points = max(8, ring * 4)
        
        for i in range(num_points):
            angle_rad = (2 * math.pi * i) / num_points
            
            # Convert polar to Cartesian offset
            dx_m = radius_m * math.cos(angle_rad)
            dy_m = radius_m * math.sin(angle_rad)
            
            # Convert meter offset to lat/lon (flat-earth approximation)
            d_lat = dy_m / 111_320.0
            d_lon = dx_m / (111_320.0 * math.cos(math.radians(center_lat)))
            
            waypoint_lat = center_lat + d_lat
            waypoint_lon = center_lon + d_lon
            waypoints.append((waypoint_lat, waypoint_lon))
    
    return waypoints


# ════════════════════════════════════════════════════════════════════════════
#  PRECISION LANDING EXECUTION
# ════════════════════════════════════════════════════════════════════════════

async def execute_precision_landing(
    drone: System,
    expected_marker_id: int,
    expected_lat: float,
    expected_lon: float,
    vision_data: Dict[str, Any],
    drone_state: Dict[str, Any],
    fsm: Any = None
) -> bool:
    """
    Descend onto an ArUco marker using body-frame velocity offboard control.
    
    Features:
    - Marker disambiguation (filters by expected_id)
    - Expanding spiral search if marker not immediately visible
    - Smooth descent with vision feedback
    - FSM event integration
    
    Args:
        drone: MAVSDK System instance
        expected_marker_id: The marker ID to land on
        expected_lat, expected_lon: Expected marker GPS coordinates
        vision_data: Shared vision data dictionary
        drone_state: Shared drone state dictionary
        fsm: Mission FSM instance (optional)
        
    Returns:
        True if landing succeeded, False if timeout/failed
        
    Side Effects:
        Fires FSM events: marker_locked, lock_lost, search_exhausted, touchdown
    """
    drone_state["status"] = f"Landing (Marker {expected_marker_id})"
    logger.info(f"Precision landing - searching for Marker ID {expected_marker_id}")
    
    K_p = 0.015  # proportional gain (pixels → m/s)
    start_t = time.time()
    
    # Enter offboard hover
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await drone.offboard.start()
    
    # Smoothing filter for vision data
    position_filter = EMAFilter(alpha=0.3)
    
    marker_locked = False
    search_waypoint_idx = 0
    
    # R6 FIX D: stall/no-progress watchdog
    last_alt_check_t = time.time()
    last_alt_value = drone_state.get("alt", 0.0)
    STALL_CHECK_INTERVAL_S = 5.0
    STALL_MIN_PROGRESS_M = 0.3
    consecutive_stall_windows = 0
    MAX_CONSECUTIVE_STALL_WINDOWS = 3
    spiral_waypoints = generate_spiral_waypoints(
        expected_lat,
        expected_lon,
        config.SEARCH_SPIRAL_STEP_M,
        config.SEARCH_SPIRAL_MAX_RADIUS_M
    )
    
    # R5 BUG 1 FIX A/C: Compute dynamic timeout based on actual search path length
    SEARCH_SPEED_MS = 1.0  # m/s - the speed used in the search pattern below
    path_len_m = sum(
        navigation.get_distance_m(spiral_waypoints[i][0], spiral_waypoints[i][1], 
                                  spiral_waypoints[i + 1][0], spiral_waypoints[i + 1][1])
        for i in range(len(spiral_waypoints) - 1)
    )
    dynamic_timeout_s = path_len_m / SEARCH_SPEED_MS + 15.0  # +15s margin for centering/settling
    effective_timeout_s = max(config.LANDING_TIMEOUT_S, dynamic_timeout_s)
    
    logger.info(
        f"Search pattern: {len(spiral_waypoints)} waypoints, ~{path_len_m:.0f}m path length, "
        f"requires ~{dynamic_timeout_s:.0f}s at {SEARCH_SPEED_MS} m/s, "
        f"using timeout={effective_timeout_s:.0f}s"
    )
    
    while True:
        elapsed = time.time() - start_t
        
        # ── timeout failsafe ─────────────────────────────────────────────────
        if elapsed > effective_timeout_s:  # R5 BUG 1 FIX A/C: Use computed timeout
            logger.warning(
                f"Landing timeout - Marker {expected_marker_id} not found after "
                f"{effective_timeout_s:.0f}s"
            )
            await drone.offboard.stop()
            
            if fsm is not None:
                fsm.fire("search_exhausted")
            
            return False
        
        # ── parse vision data (JSON array) ───────────────────────────────────
        detections = []
        if vision_data.get("locked", False):
            # Vision data contains JSON-parsed detections list
            raw_detections = vision_data.get("detections", [])
            if isinstance(raw_detections, list):
                detections = raw_detections
        
        # ── marker disambiguation ─────────────────────────────────────────────
        target_detection = select_target(detections, expected_marker_id)
        
        if target_detection is not None:
            # Marker found!
            if not marker_locked:
                marker_locked = True
                logger.info(f"Marker {expected_marker_id} locked!")
                if fsm is not None:
                    fsm.fire("marker_locked")
            
            # Extract pixel errors
            ex = target_detection.get("err_x", 0)
            ey = target_detection.get("err_y", 0)
            
            # Convert to body-frame velocities (simple P-controller on pixels)
            # TODO Task 2.2: Replace with metric offset from solvePnP
            vx = -ey * K_p  # forward/backward
            vy = ex * K_p   # left/right
            
            # Apply smoothing filter
            filtered = position_filter.update((vx, vy, 0.0))
            vx, vy, _ = filtered
            
            # Determine descent rate
            centered = abs(ex) < config.CENTER_THRESH_PX and abs(ey) < config.CENTER_THRESH_PX
            
            # Fire FSM event when centered (APPROACH → DESCEND)
            if centered and fsm is not None and fsm.state.value == "APPROACH":
                fsm.fire("centered_stable")
            
            vz = 0.8 if centered else 0.4  # faster when centered
            
            # Clamp horizontal speed
            vx = max(-1.5, min(1.5, vx))
            vy = max(-1.5, min(1.5, vy))
            
            # R6 FIX D: periodic diagnostic + stall detection
            now = time.time()
            if now - last_alt_check_t >= STALL_CHECK_INTERVAL_S:
                current_alt = drone_state.get("alt", 0.0)
                descended = last_alt_value - current_alt
                logger.info(
                    f"[LAND-DIAG] alt={current_alt:.2f}m  descended_last_{STALL_CHECK_INTERVAL_S:.0f}s={descended:.2f}m  "
                    f"centered={centered}  ex={ex:.0f}px  ey={ey:.0f}px  vz_cmd={vz:.2f}m/s"
                )
                if descended < STALL_MIN_PROGRESS_M:
                    consecutive_stall_windows += 1
                    logger.warning(
                        f"[LAND] No-progress window {consecutive_stall_windows}/{MAX_CONSECUTIVE_STALL_WINDOWS} "
                        f"({descended:.2f}m descended, vz_cmd={vz:.2f})"
                    )
                    if consecutive_stall_windows >= MAX_CONSECUTIVE_STALL_WINDOWS:
                        logger.error(
                            f"[LAND] CONFIRMED STALL — no descent progress for "
                            f"{consecutive_stall_windows * STALL_CHECK_INTERVAL_S:.0f}s. Aborting landing attempt, "
                            f"climbing back to cruise altitude instead of hanging indefinitely."
                        )
                        await drone.offboard.stop()
                        if fsm is not None:
                            fsm.fire("search_exhausted")
                        return False
                else:
                    consecutive_stall_windows = 0
                last_alt_check_t = now
                last_alt_value = current_alt
            
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, vy, vz, 0.0)
            )
            
            # ── touchdown detection (S1.3: only when locked on marker) ──────
            if drone_state["alt"] < 0.30:
                logger.info("Touchdown detected - cutting motors")
                await drone.offboard.stop()
                
                try:
                    await asyncio.wait_for(drone.action.land(), timeout=3.0)
                except Exception as e:
                    logger.warning(f"Land command timeout: {e}")
                
                # Wait for disarm
                async for armed in drone.telemetry.armed():
                    if not armed:
                        break
                
                logger.info(f"✅ Marker {expected_marker_id} secured. Disarmed.")
                
                if fsm is not None:
                    fsm.fire("touchdown")
                
                return True
            
        else:
            # Marker not visible
            if marker_locked:
                logger.warning(f"Marker {expected_marker_id} lock lost!")
                marker_locked = False
                position_filter.reset()  # Reset filter on lock loss
                if fsm is not None:
                    fsm.fire("lock_lost")
            
            # ── Execute spiral search (S1.4: actually drive the spiral) ─────
            if search_waypoint_idx < len(spiral_waypoints):
                wp_lat, wp_lon = spiral_waypoints[search_waypoint_idx]
                
                dist_m = navigation.get_distance_m(
                    drone_state["lat"], drone_state["lon"], 
                    wp_lat, wp_lon
                )
                
                if dist_m < 1.0:
                    # Reached waypoint, move to next
                    search_waypoint_idx += 1
                    logger.info(f"Search waypoint {search_waypoint_idx}/{len(spiral_waypoints)}")
                else:
                    # Navigate towards waypoint
                    bearing = navigation.get_bearing(
                        drone_state["lat"], drone_state["lon"],
                        wp_lat, wp_lon
                    )
                    n_vel, e_vel = navigation.bearing_to_ned(bearing, 1.0)  # slow search speed
                    
                    # Use NED velocity for search pattern
                    await drone.offboard.set_velocity_ned(
                        VelocityNedYaw(n_vel, e_vel, 0.0, bearing)
                    )
            else:
                # Search exhausted
                logger.warning("Spiral search exhausted - marker not found")
                await drone.offboard.stop()
                if fsm is not None:
                    fsm.fire("search_exhausted")
                return False
        
        await asyncio.sleep(0.1)  # 10 Hz loop
