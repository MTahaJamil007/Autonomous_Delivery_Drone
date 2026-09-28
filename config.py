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


MAVSDK_GRPC_PORT_BASE = 50051
"""gRPC port for each drone's embedded mavsdk_server.

MAVSDK-Python starts its own mavsdk_server per System object when no external
address is given, and each needs its own port. Three drones in one dispatcher
process therefore use 50051, 50052, 50053.
"""

MAVSDK_SYSID_BASE = 245
"""MAVLink system id for each drone's MAVSDK client.

MAVSDK defaults every client to 245. Three clients sharing one id on the same
MAVLink network means PX4 cannot tell them apart, and command acknowledgements
get attributed to the wrong drone.
"""


def drone_index(drone_id: str) -> int:
    """Extract the numeric index from a drone id ('drone-2' -> 2).

    Raises:
        ValueError: if drone_id is not of the form 'drone-N' with N >= 0.
    """
    prefix = "drone-"
    if not drone_id.startswith(prefix):
        raise ValueError(f"Invalid drone_id {drone_id!r} (expected 'drone-N')")
    try:
        index = int(drone_id[len(prefix) :])
    except ValueError as exc:
        raise ValueError(f"Invalid drone_id {drone_id!r} (expected 'drone-N')") from exc
    if index < 0:
        raise ValueError(f"Invalid drone_id {drone_id!r} (index must be >= 0)")
    return index


def mavsdk_port(drone_id: str) -> int:
    """MAVLink UDP port for a drone."""
    return MAVSDK_PORT_BASE + drone_index(drone_id)


MAVSDK_URL_SCHEME = "udp"
"""URL scheme for the MAVLink connection.

MEASURED, NOT ASSUMED. MAVSDK 2.12.10 on this host rejects `udpin://` outright:

    Warn  Unknown protocol (cli_arg.cpp:71)
    Error Connection failed: Invalid connection URL
    Failed to start, exiting...

The previous code tried three spellings in a loop, one of which was
`udpin://0.0.0.0:14540`, and appeared to succeed because its
connection_state() loop broke unconditionally on the first iteration whether or
not a heartbeat had arrived. So it reported a connection it had never made.

`udpin://` is the MAVSDK 3.x name for this scheme. pyproject.toml pins
mavsdk>=2.0,<3, where the correct name is `udp://`. If that pin is ever raised,
this is one of the two things that must change (the other is
Battery.remaining_percent's scale).
"""


def mavsdk_url(drone_id: str) -> str:
    """MAVSDK connection URL for a drone.

    Binds locally and waits for PX4 to connect to us, which is what PX4's
    px4-rc.mavlink does: it sends to 14540+instance.
    """
    return f"{MAVSDK_URL_SCHEME}://0.0.0.0:{mavsdk_port(drone_id)}"


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

SAFE_DIST = 6.5  # m  - closer than this in front triggers a dodge
CLEAR_DIST = 9.0  # m  - front must exceed this to start the clear timer
CLEAR_CONFIRM_S = 2.5  # s  - how long the front must stay clear to unlock
MIN_LOCK_S = 2.0  # s  - minimum dodge duration before testing for clear
ESCALATION_LOCK_S = 12.0  # s  - dodging this long means reactive avoidance lost

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

FRONT_HALF_DEG = 35  # +/- this defines the 70-degree front cone
SIDE_START_DEG = 40  # side sectors span [SIDE_START_DEG, SIDE_END_DEG]
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
    """Apparent width in pixels of a square of side `pad_size_m` at `altitude_m`.

    Plain projective geometry, used for logging and for coarse sanity checks.
    The authoritative question -- "can the decoder actually read this pad from
    here" -- is answered by world.pad_layout, which knows the size and position
    of every marker rather than assuming there is one.
    """
    if altitude_m <= 0.0:
        return float("inf")
    return CAMERA_FX_PX * pad_size_m / altitude_m


def decodable_px_at_altitude(altitude_m: float, pad_size_m: float = PAD_SIZE_M) -> float:
    """Apparent width of the largest marker actually on the pad, at `altitude_m`.

    SUPERSEDED IN SPIRIT by world.pad_layout. It survives because the ratio it
    encodes is still the right correction -- the decoder sees the marker, not
    the plane it is painted on -- but the ratio itself changed completely when
    the pad became a nested set, and code that assumed the old 0.796 would now
    overestimate every marker by a factor of nearly three.

    Prefer pad_layout.coverage_m() and Marker.altitude_band_m(): a nested pad
    has no single decodable width, which is the entire point of it.
    """
    return marker_px_at_altitude(altitude_m, pad_size_m) * MARKER_QUIET_ZONE_FRACTION


def _largest_marker_fraction() -> float:
    """Largest marker's side as a fraction of the pad's, from the live layout."""
    try:
        import world.pad_layout as pad_layout
    except Exception:  # noqa: BLE001 - config must import on a bare checkout
        return 0.29
    return max(m.side_m for m in pad_layout.pad_markers(0)) / pad_layout.PAD_SIZE_M


MARKER_QUIET_ZONE_FRACTION = _largest_marker_fraction()
"""Fraction of the pad's width occupied by its LARGEST decodable marker.

DERIVED from world.pad_layout, not measured off a texture, because the pad is
now generated from that layout and the two can no longer drift apart.

It used to be 0.796: the old pad carried a single marker filling almost the
whole plane, with the remaining fifth as its quiet zone. That is exactly why the
pad became invisible below 2.5 m -- a marker that large outgrows a 320x240 frame
long before touchdown -- and it is what this number changing to roughly 0.29
represents. The nested pad's markers are individually much smaller, and there
are five of them at two scales, so the pad stays readable all the way down.

Consult world.pad_layout for anything that matters; this constant exists for
logging and for the coarse budget in decodable_px_at_altitude().
"""

MARKER_MIN_DECODE_PX = 25.0
"""Below this apparent size, treat a marker as undecodable rather than absent."""

SEARCH_ALT_M = 6.0
"""Altitude at which marker search happens.

Descend here from TARGET_ALT_M before searching. At 6 m a 2 m pad spans 92 px, of which
74 px is decodable marker -- comfortably above MARKER_MIN_DECODE_PX -- while
staying high enough that the ground footprint (2 * 6 * tan(1.047/2)) is about
7 m and the spiral does not need many rings. Belt and braces with PAD_SIZE_M: either lever alone is
fragile, together they give roughly 4x headroom (finding F3).
"""

# ─────────────────────────────────────────────────────────────────────────────
#  Navigation
# ─────────────────────────────────────────────────────────────────────────────

TARGET_ALT_M = 10.0  # cruise / takeoff altitude
CRUISE_SPEED_M_S = 3.5  # maximum forward speed
SLOW_RADIUS_M = 12.0  # start decelerating within this range of the target
MIN_SPEED_M_S = 0.8  # never stall completely on approach
ARRIVAL_M = 2.0  # arrival tolerance
DODGE_SPEED_M_S = 2.5  # sideways dodge speed
BACK_SPEED_M_S = 0.4  # small rearward component while dodging
ALT_GAIN = 0.25  # altitude-hold P gain (1/s)
ALT_MAX_VEL_M_S = 0.5  # clamp on vertical correction
NAV_HZ = 10  # navigation decision rate

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
"""Metric position gain, 1/s: commanded horizontal velocity per metre of offset.

Altitude-invariant by construction. Verified stable rather than assumed: a
closed-loop simulation of the plant (first-order velocity lag tau = 0.35 s, a
two-tick measurement delay, pixel noise and an uncompensated tilt bias) lands
12/12 at this gain with a steady-state error of 2 mm, and only begins to degrade
above K_p = 2.4. tests/test_landing_closed_loop.py is that simulation.
"""

LANDING_K_D = 0.25
"""Damping on the ESTIMATED closing velocity, in seconds.

The alpha-beta estimator produces a velocity state, so the controller can be PD
rather than P. With this plant P alone is already stable, so this term buys
margin rather than fixing a defect -- it is what keeps the loop stable if the
camera rate drops or the vehicle is more sluggish than modelled.
"""

LANDING_MAX_VEL_M_S = 1.5  # clamp on horizontal landing velocity
LANDING_DEADBAND_M = 0.03  # ignore offsets smaller than the measurement noise

LANDING_ALPHA = 0.45
LANDING_BETA = 0.10
"""Alpha-beta filter gains on the measured pad offset, per axis.

Alpha weights the position correction, beta the velocity correction. Replaces a
plain EMA, which has no velocity state and therefore cannot predict through a
dropped frame -- it can only lag. Tuned for a 10 Hz measurement good to about
0.01 m (measured: 3-8 mm below 3 m, 21 mm at 8 m; see
tests/test_pad_estimator.py) and a vehicle that accelerates at a few m/s^2.
"""

CENTER_THRESH_PX = 20  # legacy pixel centring threshold, kept for tests
CENTERED_M = 0.25  # ground offset below which the pad counts as centred

# ── The descent gate ─────────────────────────────────────────────────────────

DESCENT_CONE_SLOPE = 0.35
DESCENT_CONE_INTERCEPT_M = 0.15
DESCENT_ACCEPT_FLOOR_M = 0.18
"""Descend while |offset| < max(FLOOR, SLOPE * altitude + INTERCEPT).

The cone is right in principle -- a tolerable error at 8 m is a miss at 0 m --
but a gate that keeps narrowing eventually demands better centring than the
measurement noise and the wind allow, and then it never opens again. The FLOOR
is what makes the gate always satisfiable: it is set above the 3-8 mm
measurement error and above the achieved steady-state error by a wide margin,
so the gate closes on real drift and never on noise.

The other half of the fix is that failing this gate now PAUSES the descent
(vz = 0) instead of commanding a climb. Climbing widens the cone, which reopens
the gate, which resumes the descent, which re-tightens the gate -- a limit cycle
in altitude that the shipped code entered every time.
"""

DESCENT_EXIT_HYSTERESIS = 1.6
"""Once descending, keep descending until the offset exceeds the entry limit by
this factor. Without hysteresis the gate chatters at the boundary, and a descent
that starts and stops at 10 Hz is a hover with extra steps."""

DESCENT_VZ_HIGH_M_S = 0.8  # above DESCENT_SLOW_ALT_M
DESCENT_VZ_MID_M_S = 0.35  # below DESCENT_SLOW_ALT_M
DESCENT_VZ_FINAL_M_S = 0.20  # below DESCENT_FINAL_ALT_M and during the commit
DESCENT_SLOW_ALT_M = 1.5
DESCENT_FINAL_ALT_M = 0.6

# ── Never climb back into the trap ───────────────────────────────────────────

NO_CLIMB_ALT_M = 3.0
"""Below this altitude the landing loop never commands a climb, for any reason.

This is the structural fix for the reported failure. The shipped code answered a
lost marker by flying back up to SEARCH_ALT_M, and since the marker was lost
*because* the drone had descended, going back up restored the lock and the cycle
repeated until the timeout. Set above the highest altitude at which lock is
expected to be lost (2.5 m centred, 3.4 m tilted) so the rule covers the whole
band where losing the marker is normal rather than exceptional.
"""

ALTITUDE_FLOOR_HYSTERESIS_M = 0.4
"""How far above its lowest reached altitude the drone may ever return.

A ratchet. Even if every other gate misbehaves, the commanded altitude cannot
walk back up the way it did in the observed failure, so a landing attempt is
monotonic by construction and either lands or aborts -- it cannot hover forever.
"""

# ── The open-loop commit ─────────────────────────────────────────────────────

COMMIT_MAX_OFFSET_M = 0.15
"""Offset that must be achieved before the open-loop commit may begin.

On a 2.0 m pad this leaves 0.85 m of margin to the pad edge, so even a full
0.15 m of drift during the blind descent still lands comfortably inside.
"""


def _commit_altitude_m() -> float:
    """Altitude at which to stop using vision and simply descend.

    DERIVED, not chosen. The commit gate reads "below this altitude AND within
    COMMIT_MAX_OFFSET_M", and that pair is only satisfiable if the pad is still
    measurable at that altitude WITH that much offset off the optical axis.

    Picking the number by hand is how the first draft of this file ended up with
    a commit at 0.70 m -- the centred vision floor -- which the drone could only
    satisfy while perfectly centred. Off by even 5 cm it went blind just before
    the gate it was flying toward, so the gate was unsatisfiable in precisely
    the conditions it existed for. That is the same class of mistake as the
    original defect, and deriving the constant is what makes it impossible.

    The 1.15 factor is margin: a gate evaluated at exactly its geometric limit
    is a gate that fails half the time.
    """
    try:
        import world.pad_layout as pad_layout
    except Exception:  # noqa: BLE001 - config must import on a bare checkout
        return 1.35

    tan_h = math.tan(CAMERA_HFOV_RAD / 2.0)
    tan_v = (CAMERA_HEIGHT_PX / 2.0) / CAMERA_FX_PX
    return round(
        pad_layout.visibility_floor_m(CAMERA_FX_PX, tan_h, tan_v, offset_m=COMMIT_MAX_OFFSET_M)
        * 1.15,
        2,
    )


COMMIT_ALT_M = _commit_altitude_m()
"""Altitude below which the descent continues without vision.

This is what PX4's PrecLand does with PLD_FAPPR_ALT, and what ArduPilot's
PrecLand does below its final-approach altitude: near the ground, prop wash,
ground effect and a vanishing field of view all make vision worse than dead
reckoning. Entering the commit requires being centred, so the drone only ever
goes blind from a position it already knows is good.
"""

COMMIT_TIMEOUT_S = 12.0
"""Bound on the commit. 0.70 m at DESCENT_VZ_FINAL_M_S is 3.5 s; this allows
for ground effect slowing the descent without hanging if the drone has snagged
on something."""

TOUCHDOWN_ALT_M = 0.15
"""Fallback touchdown threshold, used only if LandedState never arrives.

Lowered from 0.30 m because the estimator now reports a range good to better
than 1 percent, so the threshold no longer has to absorb altitude error.
"""

TOUCHDOWN_PUSH_S = 2.0
"""Keep commanding descent for this long after ground contact looks likely.

PX4's multicopter land detector needs sustained low thrust, low vertical
velocity and low horizontal velocity before it latches. A controller that stops
commanding descent the instant it thinks it has landed removes the very
condition the detector is waiting for, and the drone sits armed on the ground.
"""

# ── The vision envelope, derived from the pad geometry ───────────────────────


def _pad_vision_envelope() -> tuple[float, float]:
    """(lowest, highest) altitude at which at least one pad marker is usable.

    Computed from world.pad_layout against THIS camera, so the landing loop's
    commit altitude and no-climb altitude stay tied to the pad that is actually
    shipped. Changing a marker size without re-deriving these is how the
    original single-marker pad ended up with a controller that assumed it could
    see the pad at 1 m when it could not.
    """
    try:
        import world.pad_layout as pad_layout
    except Exception:  # noqa: BLE001 - config must import on a bare checkout
        return 0.69, 6.43

    tan_h = math.tan(CAMERA_HFOV_RAD / 2.0)
    tan_v = (CAMERA_HEIGHT_PX / 2.0) / CAMERA_FX_PX
    return pad_layout.coverage_m(CAMERA_FX_PX, tan_h, tan_v)


PAD_VISION_MIN_ALT_M, PAD_VISION_MAX_ALT_M = _pad_vision_envelope()
"""The altitude band over which the pad can be seen at all.

COMMIT_ALT_M is set from the low end: the descent goes open-loop exactly where
the measurement stops existing. SEARCH_ALT_M must sit below the high end or the
search flies at an altitude from which the pad is invisible -- which is what
made the original 10 m search futile.
"""


# ── Lock handling ────────────────────────────────────────────────────────────

LOCK_LOSS_GRACE_S = 1.0
"""How long the pad may go undetected before the lock is considered lost.

A single dropped frame is normal at 10 Hz. The shipped code treated one missed
frame as a lost lock and fell straight into the search branch, so ordinary
detector jitter was enough to trigger the climb. Within the grace period the
estimator coasts on its velocity state instead.
"""

MAX_MEASUREMENT_RESIDUAL_M = 0.25
"""Reject a frame whose independent per-marker estimates disagree by more than
this. Cheap insurance against a misdecoded id steering the drone; see
perception.pad_estimator.PadObservation.residual_m."""

# ── Search ───────────────────────────────────────────────────────────────────

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

STALL_WINDOW_S = 8.0
STALL_MIN_DESCENT_M = 0.25
"""Abort if a descent that should be progressing has not gained this much in
this window. Evaluated on wall-clock rather than only while the marker is
visible, so a stall during a lock-loss hold is caught too."""


def search_ring_step_m(altitude_m: float) -> float:
    """Spiral ring spacing that overlaps the camera footprint at this altitude."""
    footprint_m = 2.0 * altitude_m * math.tan(CAMERA_HFOV_RAD / 2.0)
    return max(0.5, SEARCH_RING_OVERLAP * footprint_m)


# ─────────────────────────────────────────────────────────────────────────────
#  Timeouts
# ─────────────────────────────────────────────────────────────────────────────

MAVSDK_CONNECT_TIMEOUT_S = 30
MAVSDK_PROBE_TIMEOUT_S = 4.0
"""How long a dispatch waits for a MAVLink heartbeat before giving up.

Short on purpose. PX4 emits heartbeats at 1 Hz from the moment its MAVLink
module starts, so if the autopilot is up at all, four seconds is many times
over. Waiting the full MAVSDK_CONNECT_TIMEOUT_S would hold the drone BUSY for
half a minute per doomed dispatch and make a fleet feel wedged even though the
bookkeeping is correct. Confirming that PX4 is actually up before dispatching is
scripts/preflight.py's job, not the mission's.
"""
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

BAY_OFFSET_NED_M = (0.0, 0.0, 0.35)  # 0.35 m below the airframe centre
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
    "MPC_ACC_HOR": MAX_ACCEL_M_S2,  # keep the autopilot's limit and ours equal
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


# ─────────────────────────────────────────────────────────────────────────────
#  Holding station through the open-loop commit
# ─────────────────────────────────────────────────────────────────────────────

COMMIT_HOLD_K_P = 0.7
"""Gain on position drift during the commit, in (m/s) per metre.

WHY THE COMMIT NEEDS A LOOP AT ALL
----------------------------------
The commit descends the last stretch without vision, because below
COMMIT_ALT_M the markers have left the frame. The first version commanded ZERO
HORIZONTAL VELOCITY there and left the rest to PX4, on the reasoning that PX4's
own estimator is better at holding a spot than anything this loop can
extrapolate. That reasoning is sound and the implementation did not follow it:
a zero *velocity* setpoint asks PX4 to stop moving, not to stay put, so any
drift that has already happened is never taken back.

Nine measured landings -- three complete deliveries -- put numbers on it:

    offset at commit        mean 0.042 m   max 0.065 m   (what vision achieves)
    drift during the commit mean 0.170 m   max 0.211 m   (the open-loop stretch)
    touchdown error         mean 0.209 m   max 0.466 m

The controller is roughly four times more accurate than the landing it produces,
and the whole of the gap is unopposed drift over the last 1.2 m.

So the commit now closes the loop on the drone's own position estimate against
the spot it committed from. It is still open loop with respect to VISION -- the
pad is genuinely not visible -- but it is no longer open loop with respect to
POSITION, and the estimator is entirely trustworthy over four to six seconds.
This is what PX4's PrecLand does in its final descent too: it stops using the
camera and holds the estimated target position.

0.7 /s against PX4's ~0.35 s velocity lag gives a loop time constant near 1.4 s,
comfortably faster than a 4-6 s commit and far short of the stability limit.
"""

COMMIT_HOLD_DEADBAND_M = 0.04
"""Drift below this is not corrected. Roughly the horizontal noise of the EKF's
position solution, so correcting inside it would be chasing the estimator rather
than the drone."""

COMMIT_HOLD_MAX_VEL_M_S = 0.4
"""Ceiling on the correction. The drone is within a metre of the ground with the
pad out of sight; a large horizontal command there is never the right answer,
whatever the estimate says."""
