"""
Geometric tangent-bypass detour planner.

Chosen over full grid/A* for lightweight implementation suitable for discrete
known obstacles in a small fleet environment.
"""

import math
import logging
from typing import List, Tuple, Dict, Any

logger = logging.getLogger(__name__)


def get_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate flat-earth distance in meters."""
    d_lat = (lat2 - lat1) * 111_320.0
    d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
    return math.hypot(d_lat, d_lon)


def point_to_line_distance(
    point: Tuple[float, float],
    line_start: Tuple[float, float],
    line_end: Tuple[float, float]
) -> float:
    """
    Calculate perpendicular distance from point to line segment.
    
    Args:
        point: (lat, lon) of the point
        line_start, line_end: (lat, lon) of line endpoints
        
    Returns:
        Distance in meters
    """
    # Convert to local meters (flat-earth approximation)
    ref_lat = line_start[0]
    
    px = (point[1] - line_start[1]) * (111_320.0 * math.cos(math.radians(ref_lat)))
    py = (point[0] - line_start[0]) * 111_320.0
    
    lx = (line_end[1] - line_start[1]) * (111_320.0 * math.cos(math.radians(ref_lat)))
    ly = (line_end[0] - line_start[0]) * 111_320.0
    
    line_length_sq = lx * lx + ly * ly
    
    if line_length_sq < 1e-6:
        # Line start and end are same point
        return math.hypot(px, py)
    
    # Parameter t representing projection onto line
    t = max(0.0, min(1.0, (px * lx + py * ly) / line_length_sq))
    
    # Closest point on line
    closest_x = t * lx
    closest_y = t * ly
    
    # Distance from point to closest point
    dx = px - closest_x
    dy = py - closest_y
    
    return math.hypot(dx, dy)


def compute_tangent_points(
    obstacle_lat: float,
    obstacle_lon: float,
    obstacle_radius: float,
    start: Tuple[float, float],
    goal: Tuple[float, float],
    margin_m: float
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Compute two tangent bypass points perpendicular to the start→goal line.
    
    Args:
        obstacle_lat, obstacle_lon: Obstacle center
        obstacle_radius: Obstacle radius in meters
        start, goal: (lat, lon) of path endpoints
        margin_m: Additional clearance margin
        
    Returns:
        (left_point, right_point) as (lat, lon) tuples
    """
    # Convert to local meters
    ref_lat = start[0]
    
    # Start point in meters
    sx = 0.0
    sy = 0.0
    
    # Goal in meters relative to start
    gx = (goal[1] - start[1]) * (111_320.0 * math.cos(math.radians(ref_lat)))
    gy = (goal[0] - start[0]) * 111_320.0
    
    # Obstacle in meters relative to start
    ox = (obstacle_lon - start[1]) * (111_320.0 * math.cos(math.radians(ref_lat)))
    oy = (obstacle_lat - start[0]) * 111_320.0
    
    # Direction vector from start to goal
    dx = gx - sx
    dy = gy - sy
    length = math.hypot(dx, dy)
    
    if length < 1e-6:
        # Start and goal are same - return offset points
        return (obstacle_lat, obstacle_lon), (obstacle_lat, obstacle_lon)
    
    # Normalized direction
    dir_x = dx / length
    dir_y = dy / length
    
    # Perpendicular direction (90 degrees left)
    perp_x = -dir_y
    perp_y = dir_x
    
    # Clearance distance
    clear_dist = obstacle_radius + margin_m
    
    # Tangent points (left and right of line)
    left_x = ox + perp_x * clear_dist
    left_y = oy + perp_y * clear_dist
    
    right_x = ox - perp_x * clear_dist
    right_y = oy - perp_y * clear_dist
    
    # Convert back to lat/lon
    left_lon = start[1] + left_x / (111_320.0 * math.cos(math.radians(ref_lat)))
    left_lat = start[0] + left_y / 111_320.0
    
    right_lon = start[1] + right_x / (111_320.0 * math.cos(math.radians(ref_lat)))
    right_lat = start[0] + right_y / 111_320.0
    
    return (left_lat, left_lon), (right_lat, right_lon)


def plan_detour(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Dict[str, Any]],
    margin_m: float = 2.0,
    max_iterations: int = 3
) -> List[Tuple[float, float]]:
    """
    Plan a detour route around known obstacles using tangent-bypass geometry.
    
    Algorithm:
    1. If the start→goal line clears every obstacle by >= radius_m + margin_m,
       return [start, goal] unchanged.
    2. Otherwise, for the nearest blocking obstacle, compute two tangent
       waypoints perpendicular to the start→goal line at (radius_m + margin_m)
       from its center. Prefer the side with greater lateral clearance.
    3. Recurse from the chosen tangent point to goal (bounded to max_iterations).
       If no clean bypass resolves, return [start, goal] and log a warning —
       the local LIDAR avoider remains the fallback safety net.
    
    Args:
        start: (lat, lon) starting point
        goal: (lat, lon) goal point
        obstacles: List of obstacle dicts with 'lat', 'lon', 'radius_m'
        margin_m: Additional clearance margin
        max_iterations: Maximum recursion depth
        
    Returns:
        List of (lat, lon) waypoints from start to goal
    """
    if max_iterations <= 0:
        logger.warning("Detour planning max iterations reached - using direct path")
        return [start, goal]
    
    if not obstacles:
        return [start, goal]
    
    # Check if direct path is clear
    blocking_obstacle = None
    min_clearance = float('inf')
    
    for obs in obstacles:
        obs_lat = obs.get('lat')
        obs_lon = obs.get('lon')
        obs_radius = obs.get('radius_m', 3.0)
        
        if obs_lat is None or obs_lon is None:
            continue
        
        clearance = point_to_line_distance(
            (obs_lat, obs_lon),
            start,
            goal
        )
        
        required_clearance = obs_radius + margin_m
        
        if clearance < required_clearance:
            # This obstacle blocks the path
            if clearance < min_clearance:
                min_clearance = clearance
                blocking_obstacle = obs
    
    # If no blocking obstacle, path is clear
    if blocking_obstacle is None:
        logger.debug("Direct path is clear")
        return [start, goal]
    
    # Compute tangent bypass points
    logger.info(
        f"Planning detour around obstacle at "
        f"({blocking_obstacle['lat']:.6f}, {blocking_obstacle['lon']:.6f})"
    )
    
    left_point, right_point = compute_tangent_points(
        blocking_obstacle['lat'],
        blocking_obstacle['lon'],
        blocking_obstacle['radius_m'],
        start,
        goal,
        margin_m
    )
    
    # Choose the side with better clearance to other obstacles
    # (Simplified: choose based on which has fewer nearby obstacles)
    left_obstacles_nearby = sum(
        1 for obs in obstacles
        if get_distance_m(left_point[0], left_point[1], obs['lat'], obs['lon'])
        < (obs.get('radius_m', 3.0) + margin_m + 5.0)
    )
    
    right_obstacles_nearby = sum(
        1 for obs in obstacles
        if get_distance_m(right_point[0], right_point[1], obs['lat'], obs['lon'])
        < (obs.get('radius_m', 3.0) + margin_m + 5.0)
    )
    
    if left_obstacles_nearby <= right_obstacles_nearby:
        waypoint = left_point
        logger.debug("Chose left tangent bypass")
    else:
        waypoint = right_point
        logger.debug("Chose right tangent bypass")
    
    # Recurse: plan from waypoint to goal
    remaining_path = plan_detour(
        waypoint,
        goal,
        obstacles,
        margin_m,
        max_iterations - 1
    )
    
    return [start] + remaining_path
