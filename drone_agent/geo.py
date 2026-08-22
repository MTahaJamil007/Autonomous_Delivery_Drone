"""Flat-earth geodesy. The single definition of every coordinate conversion.

WHY THIS MODULE EXISTS
----------------------
`get_distance_m` was defined six times across this codebase with identical
bodies: in drone_logic.py, navigation.py, detour.py, obstacle_memory_service/
db.py, and twice more as private methods on SafetySupervisor and PayloadBay.
`get_bearing` and the lat/lon-to-local conversion were each defined twice more.

Six copies of a formula is not a style problem. It means a correction applied
to one copy leaves five wrong, and it means the geofence check and the detour
planner can disagree about how far apart two points are while both look
correct in isolation.

ACCURACY AND ITS LIMITS
-----------------------
These are flat-earth (equirectangular) approximations: they treat one degree of
latitude as a constant 111_320 m and scale longitude by cos(latitude). Error
grows with distance and is roughly 0.1% at 10 km -- about 2 m over the longest
line this system could fly.

That is acceptable here for one specific reason: config.GEOFENCE_RADIUS_M is
500 m, and the autopilot enforces it. Inside a 500 m circle the error is under
10 cm, which is far below GPS noise. If the geofence is ever widened past a few
kilometres, replace these with proper haversine/Vincenty formulas rather than
adjusting the constant -- the approximation's failure mode is a slow drift, not
an obvious break, so it will not announce itself.

CONVENTIONS
-----------
* Latitude/longitude in decimal degrees; distances in metres.
* Bearings in degrees, 0 = North, 90 = East, always normalised to [0, 360).
* NED = North-East-Down. Down is POSITIVE, so a negative d-velocity climbs.
  This trips everyone at least once; it is PX4's convention, not a choice.
* Local Cartesian for Gazebo is ENU: x = East, y = North, z = Up. Note this is
  the opposite handedness to NED, which is why gz pose-setting swaps the axes.
"""

from __future__ import annotations

import math

METRES_PER_DEG_LAT = 111_320.0
"""Metres per degree of latitude. Constant enough for our range."""


def metres_per_deg_lon(latitude_deg: float) -> float:
    """Metres per degree of longitude at a given latitude.

    Collapses to zero at the poles, which is correct and also why a pole-crossing
    mission would need real geodesy.
    """
    return METRES_PER_DEG_LAT * math.cos(math.radians(latitude_deg))


def get_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two points, metres (flat-earth approx)."""
    d_lat_m = (lat2 - lat1) * METRES_PER_DEG_LAT
    d_lon_m = (lon2 - lon1) * metres_per_deg_lon(lat1)
    return math.hypot(d_lat_m, d_lon_m)


def get_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial compass bearing from point 1 to point 2, degrees in [0, 360).

    Uses the spherical formula rather than the flat-earth one: bearing error
    from the flat approximation is largest exactly where it matters most, at
    short range, because the numerator and denominator both go to zero.
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon_rad = math.radians(lon2 - lon1)

    x = math.sin(d_lon_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(d_lon_rad)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def bearing_to_ned(bearing_deg: float, speed_m_s: float) -> tuple[float, float]:
    """Split a compass bearing and speed into (north_m_s, east_m_s)."""
    rad = math.radians(bearing_deg)
    return speed_m_s * math.cos(rad), speed_m_s * math.sin(rad)


def offset_bearing(
    lat: float, lon: float, bearing_deg: float, distance_m: float
) -> tuple[float, float]:
    """Point `distance_m` from (lat, lon) along `bearing_deg`.

    Used to place a reported obstacle where the obstacle actually is -- the
    drone's own position projected forward along its heading by the LiDAR's
    measured clear distance -- rather than at the drone, which is up to
    SAFE_DIST metres short and biases every stored obstacle toward the
    approach path.
    """
    rad = math.radians(bearing_deg)
    d_north_m = distance_m * math.cos(rad)
    d_east_m = distance_m * math.sin(rad)
    return (
        lat + d_north_m / METRES_PER_DEG_LAT,
        lon + d_east_m / metres_per_deg_lon(lat),
    )


def lat_lon_to_local_enu(
    lat: float, lon: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """(lat, lon) -> (x_east_m, y_north_m) relative to an origin.

    ENU, which is Gazebo's world frame: x East, y North. Returned in that order
    because that is the order Gazebo's Pose message expects; getting it backwards
    puts spawned models 90 degrees around the origin from where they belong.
    """
    x_east_m = (lon - origin_lon) * metres_per_deg_lon(origin_lat)
    y_north_m = (lat - origin_lat) * METRES_PER_DEG_LAT
    return x_east_m, y_north_m


def local_enu_to_lat_lon(
    x_east_m: float, y_north_m: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """Inverse of lat_lon_to_local_enu."""
    return (
        origin_lat + y_north_m / METRES_PER_DEG_LAT,
        origin_lon + x_east_m / metres_per_deg_lon(origin_lat),
    )


def bounding_box(
    points: list[tuple[float, float]], pad_m: float = 0.0
) -> tuple[float, float, float, float]:
    """Bounding box over (lat, lon) points as (min_lat, max_lat, min_lon, max_lon).

    Args:
        points: At least one (lat, lon) pair.
        pad_m: Outward padding in metres. The obstacle prefetch pads by roughly
            the detour margin so an obstacle just outside the straight-line
            corridor -- exactly the one a detour would route around -- is still
            returned.

    Raises:
        ValueError: if `points` is empty. A zero-area box would silently match
            no obstacles, which reads identically to "there are none".
    """
    if not points:
        raise ValueError("bounding_box() needs at least one point")

    lats = [lat for lat, _ in points]
    lons = [lon for _, lon in points]

    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    if pad_m:
        d_lat = pad_m / METRES_PER_DEG_LAT
        ref_lat = (min_lat + max_lat) / 2.0
        d_lon = pad_m / metres_per_deg_lon(ref_lat)
        min_lat -= d_lat
        max_lat += d_lat
        min_lon -= d_lon
        max_lon += d_lon

    return min_lat, max_lat, min_lon, max_lon


def clamp(value: float, low: float, high: float) -> float:
    """Constrain a value to [low, high]."""
    return max(low, min(high, value))


def slew_limit(current: float, target: float, max_delta_per_s: float, dt_s: float) -> float:
    """Move `current` toward `target`, capped at `max_delta_per_s * dt_s`.

    This replaces the fixed-tick-count velocity blend the navigation loop used
    to run. Acceleration is the physically meaningful limit, it matches PX4's
    MPC_ACC_HOR, and unlike a countdown it behaves correctly when the target
    changes mid-transition: a counter restarts and produces a discontinuity at
    exactly the moment smoothness matters.
    """
    max_step = abs(max_delta_per_s) * dt_s
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)
