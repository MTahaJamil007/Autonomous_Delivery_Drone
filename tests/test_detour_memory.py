"""P5 acceptance: escalation -> detour -> memory, and memory changing behaviour.

The two-run SITL criterion (run 1 dodges and escalates, run 2 routes around the
wall from departure with zero dodges) is a flight procedure in
docs/ACCEPTANCE.md. What is proved here is the loop it depends on: that a report
is placed where the obstacle actually is, deduplicated, persisted across a
restart, and that a prefetched obstacle genuinely changes the planned route.
"""

import asyncio
from pathlib import Path

import pytest

import config
from drone_agent import geo
from drone_agent.navigation import ObstacleReporter
from global_planner.detour import plan_detour, point_to_line_distance

# ─────────────────────────────────────────────────────────────────────────────
#  P5.2 where the obstacle is recorded
# ─────────────────────────────────────────────────────────────────────────────


def test_the_report_is_placed_ahead_of_the_drone_not_at_it():
    """The LiDAR measured clear space; the obstacle is beyond it.

    Recording the drone's own coordinates -- the previous behaviour -- puts every
    obstacle up to SAFE_DIST metres short of the truth, systematically on the
    approach side. A planner using those positions routes around empty air and
    still clips the wall.
    """
    drone_lat, drone_lon = 30.0315, 72.3140
    heading_deg = 0.0  # due north
    clear_distance_m = 6.0

    obstacle_lat, obstacle_lon = geo.offset_bearing(
        drone_lat, drone_lon, heading_deg, clear_distance_m
    )

    assert obstacle_lat > drone_lat, "north of the drone, not at it"
    assert obstacle_lon == pytest.approx(drone_lon, abs=1e-9)
    assert geo.get_distance_m(drone_lat, drone_lon, obstacle_lat, obstacle_lon) == pytest.approx(
        clear_distance_m, rel=1e-6
    )


def test_a_long_wall_is_reported_once_not_fifty_times():
    """P5.2 dedupe: one dodge along a 30 m wall is one obstacle.

    Without this, every tick of a multi-second dodge posts a row, and one wall
    becomes fifty low-confidence obstacles that no planner can use.
    """
    reporter = ObstacleReporter("drone-0", dedupe_m=10.0)
    base_lat, base_lon = 30.0315, 72.3140

    assert reporter.should_report(base_lat, base_lon)
    reporter._reported.append((base_lat, base_lon))

    # Points along the same wall, 2 m apart, all inside the dedupe radius.
    for step in range(1, 5):
        near_lat, near_lon = geo.offset_bearing(base_lat, base_lon, 90.0, 2.0 * step)
        assert not reporter.should_report(near_lat, near_lon), (
            f"{2 * step} m away is inside the {reporter._dedupe_m} m dedupe radius"
        )

    # A genuinely different obstacle, well outside the radius, still reports.
    far_lat, far_lon = geo.offset_bearing(base_lat, base_lon, 90.0, 40.0)
    assert reporter.should_report(far_lat, far_lon)
    assert reporter.reports_suppressed == 4


def test_dedupe_is_per_mission_not_global():
    """A second mission must re-report, because that raises stored confidence."""
    first = ObstacleReporter("drone-0")
    second = ObstacleReporter("drone-0")
    first._reported.append((30.0315, 72.3140))
    assert second.should_report(30.0315, 72.3140), (
        "a fresh mission's reporter must not inherit the previous one's memory"
    )


async def test_report_task_exceptions_are_logged_not_swallowed():
    """The old fire-and-forget task could be collected before it ran.

    An unreferenced asyncio task can be garbage-collected mid-flight, and its
    exception disappears with it.
    """
    reporter = ObstacleReporter("drone-0")

    async def boom():
        raise RuntimeError("service down")

    task = asyncio.create_task(boom())
    reporter._tasks.add(task)
    task.add_done_callback(reporter._on_report_done)

    await asyncio.gather(task, return_exceptions=True)
    await asyncio.sleep(0)
    assert task not in reporter._tasks, "the callback must run and clean up"


# ─────────────────────────────────────────────────────────────────────────────
#  P5.1 / P5.4 the detour actually changes the route
# ─────────────────────────────────────────────────────────────────────────────


def test_a_known_obstacle_changes_the_planned_route():
    """P5.4: this is the observable difference between a database and a log.

    If a prefetched obstacle does not change the route, obstacle memory is
    write-only decoration.
    """
    start = (30.0315, 72.3140)
    goal = geo.local_enu_to_lat_lon(0.0, 60.0, *start)  # 60 m due north

    direct = plan_detour(start, goal, obstacles=[], margin_m=config.DETOUR_MARGIN_M)
    assert direct == [start, goal], "no obstacles means no detour"

    # A wall squarely on the straight line, 30 m along.
    wall_lat, wall_lon = geo.local_enu_to_lat_lon(0.0, 30.0, *start)
    obstacles = [{"lat": wall_lat, "lon": wall_lon, "radius_m": 15.0}]

    routed = plan_detour(start, goal, obstacles, margin_m=config.DETOUR_MARGIN_M)
    assert len(routed) > 2, f"the wall must force a detour, got {routed}"
    assert routed[0] == start and routed[-1] == goal

    # Every intermediate waypoint must actually clear the obstacle.
    for waypoint in routed[1:-1]:
        clearance_m = geo.get_distance_m(*waypoint, wall_lat, wall_lon)
        assert clearance_m >= 15.0, f"detour waypoint only {clearance_m:.1f} m from a 15 m obstacle"


def test_the_detour_margin_is_at_least_the_reactive_trigger_distance():
    """A detour that routes inside SAFE_DIST re-triggers the avoider it replaced."""
    assert config.DETOUR_MARGIN_M >= config.SAFE_DIST


def test_detour_recursion_is_bounded():
    """Unbounded recursion in clutter produces paths the battery cannot fly."""
    start = (30.0315, 72.3140)
    goal = geo.local_enu_to_lat_lon(0.0, 100.0, *start)

    # A dense corridor of obstacles: whatever the planner does, it must return.
    obstacles = [
        {"lat": lat, "lon": lon, "radius_m": 8.0}
        for lat, lon in (
            geo.local_enu_to_lat_lon(east, north, *start)
            for north in range(10, 100, 10)
            for east in (-6, 0, 6)
        )
    ]

    routed = plan_detour(
        start,
        goal,
        obstacles,
        margin_m=config.DETOUR_MARGIN_M,
        max_iterations=config.DETOUR_MAX_ITERATIONS,
    )
    assert isinstance(routed, list) and len(routed) >= 2
    assert len(routed) <= config.DETOUR_MAX_ITERATIONS + 2, (
        f"a bounded planner must not emit {len(routed)} waypoints"
    )


def test_an_obstacle_off_the_path_is_ignored():
    """Routing around something that is not in the way wastes battery."""
    start = (30.0315, 72.3140)
    goal = geo.local_enu_to_lat_lon(0.0, 60.0, *start)
    aside_lat, aside_lon = geo.local_enu_to_lat_lon(80.0, 30.0, *start)

    routed = plan_detour(
        start,
        goal,
        [{"lat": aside_lat, "lon": aside_lon, "radius_m": 5.0}],
        margin_m=config.DETOUR_MARGIN_M,
    )
    assert routed == [start, goal]


def test_point_to_line_distance_handles_the_degenerate_segment():
    """A zero-length segment must not divide by zero."""
    point = (30.0320, 72.3140)
    same = (30.0315, 72.3140)
    distance_m = point_to_line_distance(point, same, same)
    assert distance_m == pytest.approx(geo.get_distance_m(*point, *same), rel=1e-3)


# ─────────────────────────────────────────────────────────────────────────────
#  P5.3 persistence
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def obstacle_db(tmp_path, monkeypatch):
    from obstacle_memory_service import db as obstacle_db_module

    monkeypatch.setattr(obstacle_db_module, "DB_PATH", tmp_path / "obstacles.db")
    return obstacle_db_module


async def test_one_wall_becomes_one_row_with_rising_confidence(obstacle_db):
    """P5 acceptance: exactly one row for the wall, confidence above 0.5.

    The old merge averaged confidences, so re-reporting a 0.5 obstacle at 0.5
    left it at 0.5 forever and "confidence > 0.5" was unreachable. Confirming an
    obstacle is evidence and has to move the number.
    """
    await obstacle_db.init_database()

    wall_lat, wall_lon = 30.03200, 72.31450

    first_id = await obstacle_db.upsert_obstacle(
        wall_lat, wall_lon, radius_m=15.0, source_drone="drone-0", confidence=0.5
    )

    # Three more sightings from slightly different positions along the wall,
    # all inside the merge radius.
    for offset_m in (2.0, 3.5, 1.0):
        near_lat, near_lon = geo.offset_bearing(wall_lat, wall_lon, 90.0, offset_m)
        merged_id = await obstacle_db.upsert_obstacle(
            near_lat,
            near_lon,
            radius_m=15.0,
            source_drone="drone-1",
            confidence=0.5,
        )
        assert merged_id == first_id, "nearby sightings must merge, not multiply"

    rows = await obstacle_db.get_all_obstacles()
    assert len(rows) == 1, f"one wall must be one row, got {len(rows)}"

    wall = rows[0]
    assert wall["confidence"] > 0.5, (
        f"re-confirmation must raise confidence, got {wall['confidence']:.3f}"
    )
    assert wall["confidence"] <= 1.0
    assert wall["source_drone"] == "drone-0", "the original reporter is retained"
    assert wall["radius_m"] == 15.0


async def test_a_distant_obstacle_is_a_separate_row(obstacle_db):
    await obstacle_db.init_database()
    await obstacle_db.upsert_obstacle(30.0320, 72.3145, confidence=0.5)

    far_lat, far_lon = geo.offset_bearing(30.0320, 72.3145, 0.0, config.OBSTACLE_MERGE_RADIUS_M * 5)
    await obstacle_db.upsert_obstacle(far_lat, far_lon, confidence=0.5)

    assert len(await obstacle_db.get_all_obstacles()) == 2


async def test_obstacles_survive_a_restart(obstacle_db):
    """P5 acceptance: restarting the service preserves the row.

    The deleted root-level obstacle_memory_service.py kept obstacles in a Python
    dict, so "persisted between missions" reset on every restart. RUN_GUIDE.md
    launched that one.
    """
    await obstacle_db.init_database()
    obstacle_id = await obstacle_db.upsert_obstacle(
        30.0320, 72.3145, radius_m=15.0, source_drone="drone-0", confidence=0.8
    )

    # "Restart": a fresh init against the same file must not wipe it.
    await obstacle_db.init_database()

    rows = await obstacle_db.get_all_obstacles()
    assert len(rows) == 1
    assert rows[0]["id"] == obstacle_id
    assert rows[0]["confidence"] == pytest.approx(0.8, abs=0.01)


async def test_bbox_query_returns_the_wall_a_mission_would_prefetch(obstacle_db):
    await obstacle_db.init_database()
    wall_lat, wall_lon = 30.0320, 72.3145
    await obstacle_db.upsert_obstacle(
        wall_lat, wall_lon, radius_m=15.0, source_drone="drone-0", confidence=0.7
    )

    bbox = geo.bounding_box(
        [(30.0315, 72.3140), (30.0330, 72.3160)],
        pad_m=config.DETOUR_MARGIN_M * 4.0,
    )
    found = await obstacle_db.query_bbox(*bbox)
    assert len(found) == 1

    # And the shape the planner consumes.
    obstacle = found[0]
    for key in ("lat", "lon", "radius_m", "confidence"):
        assert key in obstacle
    routed = plan_detour(
        (30.0315, 72.3140),
        (30.0330, 72.3160),
        found,
        margin_m=config.DETOUR_MARGIN_M,
    )
    assert len(routed) > 2, "a prefetched obstacle must be usable by the planner"


async def test_low_confidence_obstacles_are_filtered_out(obstacle_db):
    """Decay makes the database self-correcting; the filter is what applies it."""
    await obstacle_db.init_database()
    await obstacle_db.upsert_obstacle(30.0320, 72.3145, confidence=0.05)

    assert await obstacle_db.query_bbox(30.0, 30.1, 72.3, 72.4, min_confidence=0.15) == []
    assert len(await obstacle_db.query_bbox(30.0, 30.1, 72.3, 72.4, min_confidence=0.0)) == 1


async def test_stats_reports_what_the_fleet_has_learned(obstacle_db):
    await obstacle_db.init_database()
    await obstacle_db.upsert_obstacle(30.0320, 72.3145, source_drone="drone-0", confidence=0.9)
    far_lat, far_lon = geo.offset_bearing(30.0320, 72.3145, 0.0, 200.0)
    await obstacle_db.upsert_obstacle(far_lat, far_lon, source_drone="drone-1", confidence=0.2)

    stats = await obstacle_db.get_stats()
    assert stats["total"] == 2
    assert stats["confident"] == 1
    assert stats["by_source_drone"] == {"drone-0": 1, "drone-1": 1}
    assert stats["merge_radius_m"] == config.OBSTACLE_MERGE_RADIUS_M


def test_confidence_merge_is_monotonic_and_bounded():
    from obstacle_memory_service.db import merge_confidence

    confidence = 0.1
    for _ in range(30):
        merged = merge_confidence(confidence, 0.3)
        assert merged > confidence, "each confirmation must raise confidence"
        assert merged <= 1.0, "and never exceed certainty"
        confidence = merged
    assert confidence > 0.99


def test_the_in_memory_duplicate_service_is_gone():
    """P5.3: the root-level service kept obstacles in a dict and read its own
    constants, so fleet-wide sharing reset on every restart."""
    project_root = Path(__file__).resolve().parents[1]
    assert not (project_root / "obstacle_memory_service.py").exists(), (
        "the non-persistent duplicate must stay deleted; two implementations is "
        "how a fix gets applied to the copy that is not running"
    )


def test_the_obstacle_service_imports_from_any_working_directory():
    """`from db import ...` only resolved from inside the package directory."""
    source = (
        Path(__file__).resolve().parents[1] / "obstacle_memory_service" / "app.py"
    ).read_text()
    assert "from obstacle_memory_service.db import" in source
    assert "\nfrom db import" not in source
