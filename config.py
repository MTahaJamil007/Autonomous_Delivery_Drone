"""Central configuration for the autonomous delivery drone fleet.

This module is the ONLY source of tunables. Nothing else may re-declare a
threshold that lives here.

That rule is not stylistic. Before remediation, `drone_web/drone_logic.py`
re-declared TARGET_ALT, CRUISE_SPEED and eight more of these values at its own
module scope, and `avoider_node.py` re-declared five as class constants. Editing
this file therefore had no effect on what actually flew, which made the file
that claims to centralise tuning the most misleading file in the repository.
If you add a constant here, `make check` greps for shadowed re-declarations.

Naming convention: every quantity carries its unit as a suffix -- `_m`, `_m_s`,
`_m_s2`, `_pct`, `_px`, `_deg`, `_rad`, `_s`, `_hz`. Two of the seven findings
behind this remediation were unit bugs (pixels treated as metres; battery
percent treated as a 0-1 fraction) that a name would have caught.
"""

import math

# ─────────────────────────────────────────────────────────────────────────────
#  Fleet
# ─────────────────────────────────────────────────────────────────────────────

FLEET_SIZE = 1
"""Number of drones. Raise to 3 for the P6 multi-drone acceptance run.

Hard ceiling of 10: PX4's px4-rc.mavlink collapses every instance above 9 onto
port 14549 (`[ "$px4_instance" -gt 9 ] && udp_offboard_port_remote=14549`), so
instances 10+ would share one MAVLink port and fight over it.
"""

MAX_FLEET_SIZE = 10

DRONE_IDS = [f"drone-{i}" for i in range(FLEET_SIZE)]

MAVSDK_PORT_BASE = 14540
"""drone-i is reached at udpin://0.0.0.0:(14540+i).

Matches PX4's `udp_offboard_port_remote=$((14540+px4_instance))`.
"""

VISION_PORT_BASE = 5005
LIDAR_PORT_BASE = 5006
PORT_STRIDE = 10
"""Spacing between per-drone UDP ports, so drone-1 uses 5015/5016, not 5006."""


def drone_index(drone_id: str) -> int:
    """Extract the numeric index from a drone id ('drone-2' -> 2).

    Raises:
        ValueError: if drone_id is not of the form 'drone-N' with N >= 0.
    """
    prefix = "drone-"
    if not drone_id.startswith(prefix):
        raise ValueError(f"Invalid drone_id {drone_id!r} (expected 'drone-N')")
    try:
        index = int(drone_id[len(prefix):])
    except ValueError as exc:
        raise ValueError(f"Invalid drone_id {drone_id!r} (expected 'drone-N')") from exc
    if index < 0:
        raise ValueError(f"Invalid drone_id {drone_id!r} (index must be >= 0)")
    return index


def mavsdk_port(drone_id: str) -> int:
    """MAVLink UDP port for a drone."""
    return MAVSDK_PORT_BASE + drone_index(drone_id)


def mavsdk_url(drone_id: str) -> str:
    """MAVSDK connection URL for a drone."""
    return f"udpin://0.0.0.0:{mavsdk_port(drone_id)}"


def vision_port(drone_id: str) -> int:
    """UDP port carrying the vision bridge's detections for a drone."""
    return VISION_PORT_BASE + PORT_STRIDE * drone_index(drone_id)


def lidar_port(drone_id: str) -> int:
    """UDP port carrying the avoider's actions for a drone."""
    return LIDAR_PORT_BASE + PORT_STRIDE * drone_index(drone_id)


# ─────────────────────────────────────────────────────────────────────────────
#  Avoider thresholds
# ─────────────────────────────────────────────────────────────────────────────
# NAMES FROZEN. tests/test_avoider_node.py reads these five by name and passes
# them positionally into decide_action(). The plan's acceptance criterion for
# P2 is that that test passes *unmodified*, so these keep their original
# unit-less names by deliberate exception to the convention above. Their units
# are metres and seconds as annotated.

SAFE_DIST = 6.5                 # m  - closer than this in front triggers a dodge
CLEAR_DIST = 9.0                # m  - front must exceed this to start the clear timer
CLEAR_CONFIRM_S = 2.5           # s  - how long the front must stay clear to unlock
MIN_LOCK_S = 2.0                # s  - minimum dodge duration before testing for clear
ESCALATION_LOCK_S = 12.0        # s  - dodging this long means reactive avoidance lost

# ─────────────────────────────────────────────────────────────────────────────
#  LiDAR geometry and sector definition
# ─────────────────────────────────────────────────────────────────────────────
# Must match sim/models/x500_delivery/model.sdf. scripts/preflight.py compares
# the live scan against these and refuses to dispatch on a mismatch, which is
# the check whose absence let the sensor suite silently disappear (F2).

LIDAR_SAMPLES = 360
LIDAR_ANGLE_MIN_RAD = -math.pi
LIDAR_ANGLE_MAX_RAD = math.pi
LIDAR_RANGE_MIN_M = 0.15
LIDAR_RANGE_MAX_M = 12.0
LIDAR_HZ = 10.0

MIN_VALID_RANGE_M = 0.5
"""Returns closer than this are treated as self-hits, not obstacles.

The propellers sit about 0.29 m from the airframe centre, and the sensor's own
floor is 0.15 m, so a prop tip is a physically valid return. Without this mask
a prop reads as a permanent 0.29 m wall directly ahead: the avoider locks into
a dodge it can never clear, escalates, and the mission stalls forever. 0.5 m
clears the props with margin while staying far below SAFE_DIST.
"""

INF_REPLACE_M = 15.0
"""Substituted for inf/nan returns. Must exceed CLEAR_DIST or an empty world
would never read as clear."""

FRONT_HALF_DEG = 35            # +/- this defines the 70-degree front cone
SIDE_START_DEG = 40            # side sectors span [SIDE_START_DEG, SIDE_END_DEG]
SIDE_END_DEG = 100

# ─────────────────────────────────────────────────────────────────────────────
#  Camera intrinsics and the marker/altitude budget
# ─────────────────────────────────────────────────────────────────────────────
# Must match sim/models/x500_delivery/model.sdf.

CAMERA_WIDTH_PX = 320
CAMERA_HEIGHT_PX = 240
CAMERA_HFOV_RAD = 1.047
CAMERA_HZ = 10.0

CAMERA_FX_PX = (CAMERA_WIDTH_PX / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
"""Focal length in pixels, 277.1 px. Derived, never hardcoded.

This is the constant that converts a pixel error into a ground offset:
    offset_m = err_px * altitude_m / CAMERA_FX_PX
Landing control runs on that metric offset rather than on raw pixels, because
pixel error for a fixed ground offset grows as 1/altitude -- so a pixel-gain
controller's real loop gain rises tenfold between 10 m and 1 m and oscillates
exactly at touchdown (finding F4).
"""

PAD_SIZE_M = 2.0
"""Side length of the landing pads. Must match sim/models/pad_N/model.sdf.

A 4x4 ArUco tag needs roughly 25-30 px to decode reliably. Apparent size is
    px = CAMERA_FX_PX * PAD_SIZE_M / altitude_m
so with the original 0.5 m pads:
    10 m -> 13.9 px   undecodable  <-- and 10 m is where the drone searched
     5 m -> 27.7 px   marginal
     3 m -> 46.2 px   reliable
The documented flow (arrive at cruise altitude, then search) therefore could
not lock even with a decodable texture. At 2.0 m a pad spans 55 px at 10 m,
which is realistic for a delivery pad and gives about 4x margin (finding F3).
"""


def marker_px_at_altitude(altitude_m: float, pad_size_m: float = PAD_SIZE_M) -> float:
    """Apparent marker width in pixels at a given altitude."""
    if altitude_m <= 0.0:
        return float("inf")
    return CAMERA_FX_PX * pad_size_m / altitude_m


MARKER_MIN_DECODE_PX = 25.0
"""Below this apparent size, treat a marker as undecodable rather than absent."""

SEARCH_ALT_M = 6.0
"""Altitude at which marker search happens.

Descend here from TARGET_ALT_M before searching. At 6 m a 2 m pad spans 92 px,
comfortably above MARKER_MIN_DECODE_PX, while staying high enough that the
ground footprint (2 * 6 * tan(1.047/2)) is about 7 m and the spiral does not
need many rings. Belt and braces with PAD_SIZE_M: either lever alone is
fragile, together they give roughly 4x headroom (finding F3).
"""

# ─────────────────────────────────────────────────────────────────────────────
#  Navigation
# ─────────────────────────────────────────────────────────────────────────────

TARGET_ALT_M = 10.0             # cruise / takeoff altitude
CRUISE_SPEED_M_S = 3.5          # maximum forward speed
SLOW_RADIUS_M = 12.0            # start decelerating within this range of the target
MIN_SPEED_M_S = 0.8             # never stall completely on approach
ARRIVAL_M = 2.0                 # arrival tolerance
DODGE_SPEED_M_S = 2.5           # sideways dodge speed
BACK_SPEED_M_S = 0.4            # small rearward component while dodging
ALT_GAIN = 0.25                 # altitude-hold P gain (1/s)
ALT_MAX_VEL_M_S = 0.5           # clamp on vertical correction
NAV_HZ = 10                     # navigation decision rate

MAX_ACCEL_M_S2 = 2.0
"""Per-axis slew limit on commanded horizontal velocity.

Replaces the old fixed-tick-count velocity blend. Acceleration is the
physically meaningful constraint, it matches PX4's MPC_ACC_HOR, and unlike a
countdown it behaves correctly when the avoider's action changes mid-blend --
the counter would restart and produce a discontinuity at exactly the moment
smoothness matters most.
"""

SETPOINT_HZ = 20
"""Rate of the offboard setpoint publisher.

PX4 drops OFFBOARD mode when the setpoint stream gaps for longer than about
0.5 s. Publishing at 20 Hz from one dedicated task -- rather than from whatever
mission code happens to be running -- makes stream continuity a structural
invariant instead of a property that every code path has to remember.
"""

STALE_SENSOR_TIMEOUT_S = 1.0
"""A sensor feed silent this long is treated as dead: hold position, never
assume the path is clear."""

# ─────────────────────────────────────────────────────────────────────────────
#  Landing
# ─────────────────────────────────────────────────────────────────────────────

LANDING_K_P = 0.8
"""Metric position gain, 1/s: commanded velocity per metre of ground offset.

Altitude-invariant by construction, unlike the pixel gain it replaces.
"""

LANDING_MAX_VEL_M_S = 1.5       # clamp on horizontal landing velocity
LANDING_DEADBAND_M = 0.05       # ignore offsets smaller than this
LANDING_EMA_ALPHA = 0.3
"""Smoothing applied to the measured metric offset.

Filter the measurement, not the output. Filtering the command (as the previous
implementation did) adds lag to the actuator without removing noise from the
sensor.
"""

CENTER_THRESH_PX = 20           # legacy pixel centring threshold, kept for tests
CENTERED_M = 0.25               # ground offset below which the pad counts as centred

DESCENT_CONE_SLOPE = 0.35
DESCENT_CONE_INTERCEPT_M = 0.15
"""Descend only while |offset| < DESCENT_CONE_SLOPE * altitude + intercept.

A cone that narrows with altitude. Descending at a fixed rate regardless of
centring converts a tolerable lateral error at 8 m into a missed pad at 0 m.
"""

DESCENT_VZ_HIGH_M_S = 0.8       # above DESCENT_SLOW_ALT_M
DESCENT_VZ_MID_M_S = 0.35       # below DESCENT_SLOW_ALT_M
DESCENT_VZ_FINAL_M_S = 0.15     # below DESCENT_FINAL_ALT_M
DESCENT_SLOW_ALT_M = 1.5
DESCENT_FINAL_ALT_M = 0.6
CLIMB_RECENTER_VZ_M_S = 0.3     # climb rate while re-centring outside the cone

TOUCHDOWN_ALT_M = 0.30
"""Fallback touchdown threshold. LandedState.ON_GROUND plus disarm is the
primary signal; this only covers the case where LandedState never arrives."""

SEARCH_SPIRAL_MAX_RADIUS_M = 10.0
SEARCH_SPEED_M_S = 1.0
SEARCH_RING_OVERLAP = 0.6
"""Ring spacing as a fraction of the camera's ground footprint.

Spacing = SEARCH_RING_OVERLAP * 2 * altitude * tan(hfov/2). Below 1.0 the
swaths overlap; at or above 1.0 they stripe and the pad can sit in a gap.
"""

LANDING_TIMEOUT_S = 200
LANDING_TIMEOUT_MAX_S = 240
"""Hard cap on the search phase.

The previous code took max(LANDING_TIMEOUT_S, path_length / speed) over a
100+ waypoint spiral, which evaluated to a multi-minute hover on a drone that
had already failed to find its pad.
"""


def search_ring_step_m(altitude_m: float) -> float:
    """Spiral ring spacing that overlaps the camera footprint at this altitude."""
    footprint_m = 2.0 * altitude_m * math.tan(CAMERA_HFOV_RAD / 2.0)
    return max(0.5, SEARCH_RING_OVERLAP * footprint_m)


# ─────────────────────────────────────────────────────────────────────────────
#  Timeouts
# ─────────────────────────────────────────────────────────────────────────────

MAVSDK_CONNECT_TIMEOUT_S = 30
GPS_FIX_TIMEOUT_S = 60
TAKEOFF_TIMEOUT_S = 30
EKF_CONVERGE_TIMEOUT_S = 30
LEG_TIMEOUT_S = 600
"""Ceiling on a single leg. Without it a leg that neither arrives nor fails
holds the drone BUSY indefinitely."""

# ─────────────────────────────────────────────────────────────────────────────
#  Battery
# ─────────────────────────────────────────────────────────────────────────────
# MAVSDK 2.x reports Battery.remaining_percent on 0-100. It was 0-1 in 1.x.
# requirements.txt pins mavsdk>=2.0,<3 for exactly this reason: the old
# >=1.4.0 spanned the break, so the same code silently read 87% as 0.87% and
# aborted every mission, or read 0.87 as 0.87% and never aborted at all.

BATTERY_CRITICAL_PCT = 20
BATTERY_RESERVE_MARGIN_PCT = 15
BATTERY_ENERGY_PER_M_PCT = 0.01
"""Percent of battery per metre of flight. PLACEHOLDER -- needs real-flight
calibration; see docs/CALIBRATION.md."""
BATTERY_LANDING_RESERVE_PCT = 5.0

# ─────────────────────────────────────────────────────────────────────────────
#  Payload
# ─────────────────────────────────────────────────────────────────────────────

BAY_OFFSET_NED_M = (0.0, 0.0, 0.35)   # 0.35 m below the airframe centre
CARGO_MODEL_PREFIX = "cargo"
CARGO_SDF_URI = "model://cargo_box"
PAYLOAD_HZ = 10

# ─────────────────────────────────────────────────────────────────────────────
#  Safety supervisor
# ─────────────────────────────────────────────────────────────────────────────

GEOFENCE_RADIUS_M = 500.0
"""Also uploaded to PX4 as a real geofence via MAVSDK's geofence plugin.

An autopilot-enforced limit survives a Python crash; a supervisor task does
not. Also keeps the flat-earth geodesy error (about 2 m at 10 km) irrelevant.
"""

HEARTBEAT_TIMEOUT_S = 5.0
SUPERVISOR_HZ = 1.0

# ─────────────────────────────────────────────────────────────────────────────
#  Obstacle memory
# ─────────────────────────────────────────────────────────────────────────────

OBSTACLE_MERGE_RADIUS_M = 5.0
OBSTACLE_DECAY_TAU_DAYS = 14
OBSTACLE_MEMORY_URL = "http://127.0.0.1:5050"
OBSTACLE_REPORT_DEDUPE_M = 10.0
"""Suppress a new report within this distance of one already sent this mission.

A 30 m wall would otherwise be reported on every tick of a single dodge,
turning one obstacle into fifty rows.
"""
OBSTACLE_DEFAULT_RADIUS_M = 3.0
OBSTACLE_MIN_CONFIDENCE = 0.15

# ─────────────────────────────────────────────────────────────────────────────
#  Detour planning
# ─────────────────────────────────────────────────────────────────────────────

DETOUR_MARGIN_M = SAFE_DIST
DETOUR_MAX_ITERATIONS = 3
DETOUR_MAX_ATTEMPTS_PER_LEG = 3
"""After this many escalations on one leg, hover and alert rather than keep
generating longer paths through clutter until the battery gate aborts."""

# ─────────────────────────────────────────────────────────────────────────────
#  Services
# ─────────────────────────────────────────────────────────────────────────────

FLEET_DISPATCH_URL = "http://127.0.0.1:5000"
FLEET_DISPATCH_PORT = 5000
OBSTACLE_MEMORY_PORT = 5050

# ─────────────────────────────────────────────────────────────────────────────
#  Simulation
# ─────────────────────────────────────────────────────────────────────────────

GZ_WORLD = "delivery"
GZ_MODEL_BASE = "x500_delivery"
"""PX4 spawns the model as f"{GZ_MODEL_BASE}_{instance}" -- see
ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim. Model-scoped sensor topics are
derived from that name; see sim_topics.py."""

# ─────────────────────────────────────────────────────────────────────────────
#  PX4 parameters
# ─────────────────────────────────────────────────────────────────────────────

PX4_PARAMS = {
    "MPC_ACC_HOR": MAX_ACCEL_M_S2,   # keep the autopilot's limit and ours equal
    "MPC_ACC_HOR_MAX": 5.0,
    "MPC_JERK_MAX": 8.0,
    "MPC_XY_VEL_MAX": 4.0,
    "MPC_XY_P": 0.95,
    "MPC_Z_VEL_MAX_DN": 1.0,
    "MPC_Z_VEL_MAX_UP": 3.0,
}
"""Applied at mission start with param.set_param_float and read back.

These were previously a comment block asking the operator to type seven values
into a PX4 shell. A manual step does not survive a reboot, cannot be reviewed,
and cannot be verified after the fact -- and nothing in the logs distinguished
"tuned" from "never typed". See drone_agent/px4_params.py.
"""

PX4_PARAM_TOLERANCE = 1e-3
