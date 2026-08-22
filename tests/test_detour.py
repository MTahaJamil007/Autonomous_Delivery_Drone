"""
Unit tests for detour planner.
"""

from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from global_planner.detour import plan_detour, get_distance_m, point_to_line_distance


def test_direct_path_with_no_obstacles():
    """Test that empty obstacle list returns direct path."""
    start = (47.397606, 8.545594)
    goal = (47.398606, 8.546594)
    
    path = plan_detour(start, goal, obstacles=[], margin_m=2.0)
    
    assert len(path) == 2, f"Expected 2 waypoints, got {len(path)}"
    assert path[0] == start, "First waypoint should be start"
    assert path[1] == goal, "Last waypoint should be goal"
    
    print("✅ Direct path with no obstacles test passed")


def test_obstacle_on_path_creates_detour():
    """
    Acceptance test: synthetic obstacle placed directly on the straight-line
    path should produce a 3+ waypoint route that clears the obstacle.
    """
    start = (47.397606, 8.545594)
    goal = (47.398606, 8.546594)
    
    # Place obstacle at midpoint of path
    mid_lat = (start[0] + goal[0]) / 2
    mid_lon = (start[1] + goal[1]) / 2
    
    obstacle = {
        "lat": mid_lat,
        "lon": mid_lon,
        "radius_m": 5.0,
        "obstacle_type": "static_wall"
    }
    
    path = plan_detour(start, goal, obstacles=[obstacle], margin_m=2.0)
    
    assert len(path) >= 3, f"Expected 3+ waypoints for detour, got {len(path)}"
    assert path[0] == start, "First waypoint should be start"
    assert path[-1] == goal, "Last waypoint should be goal"
    
    # Check that intermediate waypoints clear the obstacle
    for waypoint in path[1:-1]:
        dist_to_obstacle = get_distance_m(
            waypoint[0], waypoint[1],
            obstacle['lat'], obstacle['lon']
        )
        required_clearance = obstacle['radius_m'] + 2.0  # margin
        
        assert dist_to_obstacle >= required_clearance - 0.1, \
            f"Waypoint {waypoint} too close to obstacle: {dist_to_obstacle:.1f}m < {required_clearance:.1f}m"
    
    print(f"✅ Obstacle on path creates detour: {len(path)} waypoints with proper clearance")


def test_obstacle_off_path_no_detour():
    """Test that obstacle off to the side doesn't trigger detour."""
    start = (47.397606, 8.545594)
    goal = (47.398606, 8.546594)
    
    # Place obstacle 20 meters to the side (well clear)
    obstacle = {
        "lat": start[0] + 0.0001,  # ~11m north
        "lon": start[1] - 0.0002,  # ~15m west  
        "radius_m": 3.0
    }
    
    path = plan_detour(start, goal, obstacles=[obstacle], margin_m=2.0)
    
    # Should return direct path since obstacle doesn't block
    assert len(path) == 2, f"Expected direct path (2 waypoints), got {len(path)}"
    
    print("✅ Obstacle off path produces direct route")


def test_point_to_line_distance():
    """Test the point-to-line distance calculation."""
    # Simple test: point perpendicular to line
    line_start = (47.0, 8.0)
    line_end = (47.0, 8.001)
    
    # Point 111m north of line midpoint
    point = (47.001, 8.0005)
    
    dist = point_to_line_distance(point, line_start, line_end)
    
    # Should be approximately 111 meters (1 degree lat ≈ 111km)
    assert 100 < dist < 120, f"Expected ~111m, got {dist:.1f}m"
    
    print(f"✅ Point-to-line distance test passed: {dist:.1f}m")


if __name__ == "__main__":
    test_direct_path_with_no_obstacles()
    test_obstacle_on_path_creates_detour()
    test_obstacle_off_path_no_detour()
    test_point_to_line_distance()
    print("\n🎉 All detour planner tests passed!")
