"""Precision landing on a specific nested ArUco pad.

WHY THE PREVIOUS VERSION NEVER LANDED
=====================================
It was not the gains, and it was not the signs. It was the field of view.

The old pad carried one 1.592 m marker. ArUco needs a marker's four corners AND
its white quiet zone inside the frame, so a marker of side s centred under this
camera (320x240, hfov 1.047, tan of the vertical half-FOV = 0.4330) is readable
only above

    s * (1 + 2 * quiet) / (2 * tan(v))  =  2.76 m for s = 1.592

Measured against the real detector and the project's own texture: lock was first
lost at 2.52 m centred, at 3.42 m with 8 degrees of tilt, and was impossible
below 1.8 m in every condition.

What turned that into a hover was the response. Losing the marker set
locked=False and dropped into the GPS-spiral search branch, which commands
`altitude_correction(alt, SEARCH_ALT_M)` -- a climb back to 6 m. Going up
restored the lock, so the drone descended again, went blind at ~2.5 m again, and
climbed again. That is precisely "detects the marker, hovers around it, moves
down in a spiral, never lands", and it repeated until the 240 s timeout.

A closed-loop simulation of the shipped controller against a plant model lands
in 0 of 12 runs and parks at 2.56 m. The same controller with the marker kept
visible lands in 12 of 12 with 2 mm of error. The control law was never the
problem; tests/test_landing_closed_loop.py pins both halves of that result.

WHAT THIS VERSION DOES DIFFERENTLY
==================================
1. THE PAD IS VISIBLE ALL THE WAY DOWN. world.pad_layout puts five markers at
   two scales on each pad; at least one is readable from 0.5 m to beyond 8 m.
   perception.pad_estimator turns any one of them into the pad centre.

2. IT NEVER CLIMBS BACK INTO THE TRAP. Below config.NO_CLIMB_ALT_M the loop
   cannot command a climb for any reason, and an altitude ratchet forbids
   returning more than config.ALTITUDE_FLOOR_HYSTERESIS_M above the lowest
   altitude reached. A landing attempt is monotonic by construction: it lands or
   it aborts, but it cannot hover forever.

3. FAILING THE DESCENT GATE PAUSES THE DESCENT, IT DOES NOT CLIMB. The old cone
   answered a large offset by climbing, which widened the cone, which reopened
   the gate -- a limit cycle in altitude. The gate now has a floor so it stays
   satisfiable, and hysteresis so it does not chatter.

4. IT COMMITS. Below config.COMMIT_ALT_M -- set to the altitude at which the
   centre marker leaves the frame -- the descent continues open-loop, which is
   what PX4's PrecLand (PLD_FAPPR_ALT) and ArduPilot's PrecLand both do. Near
   the ground, prop wash and a vanishing field of view make vision worse than
   dead reckoning.

5. OFFSETS ARE MEASURED IN METRES WITHOUT AN ALTITUDE. The estimator recovers
   scale from the marker geometry, so the loop no longer multiplies its error by
   a drifting EKF altitude. Range comes back out as an independent cross-check.

6. IT ESTIMATES VELOCITY, NOT JUST POSITION. An alpha-beta filter replaces the
   EMA, which gave the controller damping and, more usefully, lets it coast
   through a dropped frame instead of treating one as a lost lock.

7. TILT IS COMPENSATED. The drone tilts to accelerate, which swings a
   rigidly-mounted downward camera off nadir and displaces the pad by
   `range * tan(tilt)` -- 0.52 m at 6 m and 5 degrees, far more than the
   centring tolerance. The loop steers the pad to the NADIR point rather than to
   the image centre.

8. TOUCHDOWN IS PUSHED, NOT ASSUMED. PX4's land detector needs sustained low
   thrust and low velocity; a controller that stops descending the moment it
   thinks it has landed removes the condition the detector is waiting for.

9. TOUCHDOWN DOES NOT WAIT FOR PERMISSION IT WILL NEVER GET. The corollary of
   point 8, learned from a live flight rather than from reasoning. Under an
   offboard descent PX4's land detector is not merely slow to report ON_GROUND
   -- it will not report it at all, because the loop is streaming it a downward
   velocity setpoint the whole way down and the detector refuses to declare a
   landing while it is being commanded to fly.

   The first end-to-end flight of this controller descended beautifully,
   committed at 1.17 m only 0.042 m off centre, settled onto the pad 0.12 m from
   its centre by Gazebo ground truth -- and then reported STALLED and failed the
   mission, because `is_on_ground` let an IN_AIR reading veto a height of
   -0.03 m. So a sustained low height is now sufficient on its own, and a commit
   that overruns hands the vehicle to the autopilot's own land mode before
   calling anything a failure. Both are covered by
   tests/test_landing_integration.py.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

import config
from drone_agent import geo, udp_receiver
from drone_agent.setpoint import SetpointPublisher

logger = logging.getLogger(__name__)


class LandingOutcome:
    TOUCHDOWN = "touchdown"
    SEARCH_EXHAUSTED = "search_exhausted"
    TIMEOUT = "timeout"
    SENSOR_LOST = "sensor_lost"
    STALLED = "stalled"


class LandingPhase:
    """Where the loop is. Reported in drone_state['landing_phase'] so an
    operator watching the UI can see which gate is holding things up."""

    SEARCH = "search"
    APPROACH = "approach"
    DESCEND = "descend"
    HOLD = "hold"
    COMMIT = "commit"
    TOUCHDOWN = "touchdown"


# ═════════════════════════════════════════════════════════════════════════════
#  CAMERA MOUNT
# ═════════════════════════════════════════════════════════════════════════════
#
# The mount is `pose 0 0 -0.05 0 1.5708 0` on base_link -- a +90 degree pitch
# about Y, so the camera looks straight down. Per docs/CALIBRATION.md, and
# confirmed by scripts/calibrate_camera_signs.py:
#
#     image +u (rightwards) -> body RIGHT
#     image +v (downwards)  -> body AFT
#
# Both the offset mapping and the tilt correction below are written in terms of
# these two constants, so correcting the mount is a one-line change in one place
# rather than a hunt for every sign that depends on it.

U_TO_BODY_RIGHT = +1.0
V_TO_BODY_AFT = +1.0


def body_offsets_m(right_m: float, down_m: float) -> tuple[float, float]:
    """(forward_m, right_m) in the body frame, from an image-plane offset.

    `right_m`/`down_m` are the pad centre's position in the image plane, in
    metres, +u and +v. A pad that is AFT of the drone must be closed on by
    flying backwards, hence the negation on forward.
    """
    forward_m = -V_TO_BODY_AFT * down_m
    body_right_m = U_TO_BODY_RIGHT * right_m
    return forward_m, body_right_m


def nadir_offset_px(
    roll_deg: float, pitch_deg: float, fx_px: float = config.CAMERA_FX_PX
) -> tuple[float, float]:
    """Where straight-down lands in the image, relative to the image centre.

    A rigidly-mounted downward camera swings off nadir whenever the airframe
    tilts, so the point on the ground directly beneath the drone is NOT the
    centre pixel. Steering the pad to the centre pixel therefore steers it to
    wherever the current tilt happens to point -- an error of `range * tan(tilt)`,
    which is 0.52 m at 6 m and 5 degrees.

    Derivation, in PX4's FRD body frame (x forward, y right, z down), which is
    what MAVSDK's telemetry.attitude_euler() reports:

        world-down, in body coords, is the third row of Rz(yaw)Ry(pitch)Rx(roll):
            d = [-sin(p), cos(p) sin(r), cos(p) cos(r)]

        projecting that ray through a pinhole whose optical axis is body +Z,
        with image +u along body RIGHT (+Y) and image +v along body AFT (-X):
            u - cx = fx * d_y / d_z      = fx * tan(r)
            v - cy = fx * (-d_x) / d_z   = fx * tan(p) / cos(r)

    Sanity check at roll = +90 degrees (right wing straight down): body +Z, the
    optical axis, swings to point at the world horizon on the left, so nadir is
    a quarter turn away toward body +Y -- far off to the image RIGHT. tan(r)
    diverging positive is exactly that.

    Returns (0, 0) for a level airframe, so a caller with no attitude available
    is simply uncorrected rather than wrong.
    """
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    # Beyond this the projection is meaningless and the drone is not landing
    # anyway; clamping stops tan() from producing an enormous correction.
    limit_rad = math.radians(45.0)
    roll_rad = geo.clamp(roll_rad, -limit_rad, limit_rad)
    pitch_rad = geo.clamp(pitch_rad, -limit_rad, limit_rad)

    u_px = U_TO_BODY_RIGHT * fx_px * math.tan(roll_rad)
    v_px = V_TO_BODY_AFT * fx_px * math.tan(pitch_rad) / max(math.cos(roll_rad), 0.5)
    return u_px, v_px


# ═════════════════════════════════════════════════════════════════════════════
#  ESTIMATION
# ═════════════════════════════════════════════════════════════════════════════


class AlphaBetaTracker:
    """Two-state (position, velocity) tracker per axis, over a 2-vector.

    Replaces the EMA the previous loop used. An EMA has no velocity state, so it
    can only lag a moving target and has nothing to offer when a frame is
    dropped. This has both: `predict` coasts on velocity through a dropout, and
    the velocity state gives the controller a derivative term that a P-on-EMA
    loop cannot have.

    Alpha-beta rather than a full Kalman filter deliberately. The measurement
    noise here is dominated by geometry that is already known and stable
    (3-8 mm below 3 m), so the covariance a Kalman filter would carry adds
    tuning surface without adding information. PX4's own
    landing_target_estimator uses a Kalman filter because it must fuse an
    angular measurement with vehicle acceleration; this measurement is metric
    and direct.
    """

    def __init__(self, alpha: float = config.LANDING_ALPHA, beta: float = config.LANDING_BETA):
        self._alpha = alpha
        self._beta = beta
        self.position: tuple[float, float] | None = None
        self.velocity: tuple[float, float] = (0.0, 0.0)

    def predict(self, dt_s: float) -> tuple[float, float] | None:
        """Coast one tick on the velocity state. Called when no measurement
        arrived, which is what turns a dropped frame into a small extrapolation
        instead of a lost lock."""
        if self.position is None:
            return None
        self.position = (
            self.position[0] + self.velocity[0] * dt_s,
            self.position[1] + self.velocity[1] * dt_s,
        )
        return self.position

    def update(self, measurement: tuple[float, float], dt_s: float) -> tuple[float, float]:
        """Fold in a measurement and return the filtered position."""
        if self.position is None:
            self.position = measurement
            self.velocity = (0.0, 0.0)
            return self.position

        predicted = (
            self.position[0] + self.velocity[0] * dt_s,
            self.position[1] + self.velocity[1] * dt_s,
        )
        residual = (measurement[0] - predicted[0], measurement[1] - predicted[1])

        self.position = (
            predicted[0] + self._alpha * residual[0],
            predicted[1] + self._alpha * residual[1],
        )
        if dt_s > 1e-6:
            self.velocity = (
                self.velocity[0] + self._beta * residual[0] / dt_s,
                self.velocity[1] + self._beta * residual[1] / dt_s,
            )
        return self.position

    def reset(self) -> None:
        self.position = None
        self.velocity = (0.0, 0.0)


class AltitudeRatchet:
    """Remembers the lowest altitude reached and refuses to give it back.

    The single structural guarantee against the observed failure. Whatever any
    other gate decides, the commanded altitude cannot walk back up more than
    `hysteresis` above the best progress made, so a landing attempt is monotonic
    and must terminate. The hysteresis exists because a real descent overshoots
    and settles; without it, one noisy low sample would pin the drone there.
    """

    def __init__(self, hysteresis_m: float = config.ALTITUDE_FLOOR_HYSTERESIS_M):
        self._hysteresis_m = hysteresis_m
        self.lowest_m: float | None = None

    def observe(self, altitude_m: float) -> None:
        if self.lowest_m is None or altitude_m < self.lowest_m:
            self.lowest_m = altitude_m

    @property
    def ceiling_m(self) -> float:
        """The highest altitude still permitted."""
        if self.lowest_m is None:
            return float("inf")
        return self.lowest_m + self._hysteresis_m

    def limit_climb(self, altitude_m: float, down_m_s: float) -> float:
        """Zero out a climb that would breach the ceiling.

        `down_m_s` is NED: negative climbs. Returns it unchanged when it is a
        descent or when there is headroom.
        """
        if down_m_s >= 0.0:
            return down_m_s
        if altitude_m >= self.ceiling_m:
            return 0.0
        return down_m_s


# ═════════════════════════════════════════════════════════════════════════════
#  GATES
# ═════════════════════════════════════════════════════════════════════════════


def descent_gate_limit_m(altitude_m: float) -> float:
    """Offset below which descending is allowed at this altitude.

    A cone that narrows toward the ground, but with a FLOOR. The floor is the
    whole difference between a gate and a deadlock: `slope * alt + intercept`
    keeps shrinking, and once it drops below the achievable steady-state error
    it can never be satisfied again, so the descent stops permanently while the
    drone still holds a perfect lock. config.DESCENT_ACCEPT_FLOOR_M is set an
    order of magnitude above the measured 3-8 mm measurement error.
    """
    cone_m = config.DESCENT_CONE_SLOPE * altitude_m + config.DESCENT_CONE_INTERCEPT_M
    return max(config.DESCENT_ACCEPT_FLOOR_M, cone_m)


def should_descend(offset_m: float, altitude_m: float, currently_descending: bool) -> bool:
    """The descent gate, with hysteresis.

    Entering needs `offset < limit`; leaving needs it to exceed the limit by
    config.DESCENT_EXIT_HYSTERESIS. A gate without hysteresis chatters at its
    boundary, and a descent that starts and stops at 10 Hz is a hover.
    """
    limit_m = descent_gate_limit_m(altitude_m)
    if currently_descending:
        return offset_m < limit_m * config.DESCENT_EXIT_HYSTERESIS
    return offset_m < limit_m


def descent_rate_m_s(altitude_m: float) -> float:
    """Scheduled descent rate: brisk high up, slow for the last metre."""
    if altitude_m > config.DESCENT_SLOW_ALT_M:
        return config.DESCENT_VZ_HIGH_M_S
    if altitude_m > config.DESCENT_FINAL_ALT_M:
        return config.DESCENT_VZ_MID_M_S
    return config.DESCENT_VZ_FINAL_M_S


def may_climb(altitude_m: float) -> bool:
    """False below config.NO_CLIMB_ALT_M.

    Losing the pad low down is NORMAL -- it is what the field of view does -- so
    answering it with a climb turns an expected event into an infinite loop.
    Above this altitude a climb is still the right answer to a genuinely lost
    pad, because there the loss means the drone is somewhere it should not be.
    """
    return altitude_m > config.NO_CLIMB_ALT_M


# ═════════════════════════════════════════════════════════════════════════════
#  MARKER DISAMBIGUATION (legacy per-marker contract)
# ═════════════════════════════════════════════════════════════════════════════


def select_target(detections: list[dict], expected_id: int) -> dict | None:
    """The raw detection matching expected_id, or None.

    Retained because the per-marker contract is still emitted for debugging and
    for tests. The landing loop itself consumes the richer per-PAD observations,
    which fold every visible marker of a pad into one estimate of its centre.
    """
    return next((d for d in detections if d.get("id") == expected_id), None)


# ═════════════════════════════════════════════════════════════════════════════
#  SEARCH PATTERN
# ═════════════════════════════════════════════════════════════════════════════


def camera_footprint_m(altitude_m: float) -> float:
    """Ground width the camera sees at this altitude."""
    return 2.0 * altitude_m * math.tan(config.CAMERA_HFOV_RAD / 2.0)


def generate_spiral_waypoints(
    center_lat: float,
    center_lon: float,
    altitude_m: float = config.SEARCH_ALT_M,
    max_radius_m: float = config.SEARCH_SPIRAL_MAX_RADIUS_M,
    step_m: float | None = None,
) -> list[tuple[float, float]]:
    """Expanding spiral over a GPS point, spaced to overlap the camera swath.

    Ring spacing defaults to 0.6 of the camera footprint at the search altitude,
    so successive passes overlap rather than stripe and leave gaps the pad can
    sit in.
    """
    if step_m is None:
        step_m = config.search_ring_step_m(altitude_m)

    waypoints = [(center_lat, center_lon)]
    ring_count = max(1, int(max_radius_m / step_m))

    for ring in range(1, ring_count + 1):
        radius_m = ring * step_m
        # Points per ring proportional to circumference, so along-track spacing
        # stays roughly constant instead of thinning out on the big rings.
        point_count = max(8, int(2.0 * math.pi * radius_m / step_m))

        for i in range(point_count):
            angle_rad = 2.0 * math.pi * i / point_count
            lat, lon = geo.local_enu_to_lat_lon(
                radius_m * math.cos(angle_rad),
                radius_m * math.sin(angle_rad),
                center_lat,
                center_lon,
            )
            waypoints.append((lat, lon))

    return waypoints


def search_timeout_s(
    waypoints: list[tuple[float, float]],
    speed_m_s: float = config.SEARCH_SPEED_M_S,
) -> float:
    """Time budget for the search, hard-capped by config.LANDING_TIMEOUT_MAX_S."""
    path_m = sum(
        geo.get_distance_m(*waypoints[i], *waypoints[i + 1]) for i in range(len(waypoints) - 1)
    )
    estimated_s = path_m / max(speed_m_s, 0.1) + 15.0
    return min(max(config.LANDING_TIMEOUT_S, estimated_s), config.LANDING_TIMEOUT_MAX_S)


# ═════════════════════════════════════════════════════════════════════════════
#  TOUCHDOWN DETECTION
# ═════════════════════════════════════════════════════════════════════════════


def is_on_ground(drone_state: dict[str, Any], height_m: float | None = None) -> bool:
    """True when we have touched down. Either signal is sufficient.

    PX4's LandedState is authoritative when it says ON_GROUND -- it fuses
    altitude, vertical velocity and thrust, so it does not fire merely because
    the drone descended past a threshold, and it does not miss a touchdown on
    raised ground.

    WHY IT IS NOT AUTHORITATIVE WHEN IT SAYS *IN_AIR*
    -------------------------------------------------
    An earlier version of this function let IN_AIR VETO the height check, on the
    reasoning that the autopilot knows better than a noisy altitude sample. A
    live SITL flight showed that reasoning to be exactly backwards, and it cost
    a landing that had otherwise gone perfectly:

        14:56:27  committing to the landing from 1.17 m, 0.042 m off centre
        14:56:39  commit did not reach the ground within 12 s (still -0.03 m up)

    The drone was on the ground. Gazebo ground truth put it 0.12 m from the pad
    centre with z = -0.013 m, and the loop's own height reading was -0.03 m --
    comfortably inside TOUCHDOWN_ALT_M. It reported STALLED anyway, because
    LandedState was still IN_AIR and the veto made the height check unreachable.

    The reason PX4 says IN_AIR there is structural, not transient: its land
    detector wants sustained low thrust and near-zero velocity, and the commit
    is *streaming a 0.2 m/s downward velocity setpoint in OFFBOARD the entire
    time*. The detector will not declare a landing while it is being actively
    commanded to fly, so under offboard descent LandedState stays IN_AIR right
    through touchdown and beyond. Vetoing on it is therefore a guaranteed
    deadlock at exactly the moment the answer is needed, not a safeguard.

    Noise rejection has not been given up, it has been moved to where it
    belongs. Both callers require the condition to hold continuously for
    config.TOUCHDOWN_PUSH_S before acting, and then confirm through
    `_confirm_landed`, which commands a land and waits for the autopilot to
    actually disarm. A single spurious low sample cannot survive either gate.
    """
    if drone_state.get("landed_state") == "ON_GROUND":
        return True

    if height_m is None:
        height_m = drone_state.get("alt", 99.0)
    return height_m < config.TOUCHDOWN_ALT_M


# ═════════════════════════════════════════════════════════════════════════════
#  MEASUREMENT
# ═════════════════════════════════════════════════════════════════════════════


def observe_pad(
    vision_data: dict[str, Any],
    expected_pad: int,
    drone_state: dict[str, Any],
) -> dict[str, Any] | None:
    """The freshest usable observation of `expected_pad`, or None.

    Reads the per-pad estimates the vision bridge publishes, applies the tilt
    correction, and returns a plain dict so this stays testable without OpenCV,
    numpy, or a camera.
    """
    pads = vision_data.get("pads") or []
    pad = next((p for p in pads if p.get("pad_id") == expected_pad), None)
    if pad is None:
        return None

    px_per_m = float(pad.get("px_per_m", 0.0))
    if px_per_m <= 0.0:
        return None

    residual_m = float(pad.get("residual_m", 0.0))
    if residual_m > config.MAX_MEASUREMENT_RESIDUAL_M:
        # The contributing markers disagree about where the pad centre is, which
        # means at least one of them is wrong. Steering on the average of a good
        # measurement and a bad one is worse than coasting.
        logger.debug("rejecting a pad frame: per-marker estimates disagree by %.3f m", residual_m)
        return None

    err_u = float(pad.get("err_x", 0.0))
    err_v = float(pad.get("err_y", 0.0))

    # ── Tilt correction ──────────────────────────────────────────────────────
    roll_deg = drone_state.get("roll_deg")
    pitch_deg = drone_state.get("pitch_deg")
    tilt_corrected = False
    if roll_deg is not None and pitch_deg is not None:
        fx_px = float(vision_data.get("fx") or config.CAMERA_FX_PX)
        nadir_u, nadir_v = nadir_offset_px(float(roll_deg), float(pitch_deg), fx_px)
        err_u -= nadir_u
        err_v -= nadir_v
        tilt_corrected = True

    right_m = err_u / px_per_m
    down_m = err_v / px_per_m
    forward_m, body_right_m = body_offsets_m(right_m, down_m)

    return {
        "forward_m": forward_m,
        "right_m": body_right_m,
        "offset_m": math.hypot(forward_m, body_right_m),
        "range_m": float(pad.get("range_m", 0.0)),
        "marker_ids": pad.get("markers", []),
        "residual_m": residual_m,
        "tilt_corrected": tilt_corrected,
    }


def working_height_m(observation: dict[str, Any] | None, drone_state: dict[str, Any]) -> float:
    """Height above the pad, preferring the vision range over the EKF altitude.

    `range_m` comes from the marker's known size and is good to better than one
    percent below 6 m -- and, unlike `relative_altitude_m`, it is measured
    against the PAD rather than against wherever the drone happened to take off.
    On a pad that is not at takeoff elevation those differ, and it is the pad
    that matters.

    The EKF altitude remains the fallback and the sanity check: a vision range
    that disagrees with it by more than a factor of two means one of them is
    badly wrong, and the autopilot's is the one with redundant sensors behind it.
    """
    ekf_alt_m = float(drone_state.get("alt", 0.0))
    if observation is None:
        return ekf_alt_m

    range_m = float(observation.get("range_m", 0.0))
    if range_m <= 0.0:
        return ekf_alt_m
    if ekf_alt_m > 0.5 and not (0.5 <= range_m / ekf_alt_m <= 2.0):
        logger.warning(
            "vision range %.2f m disagrees with EKF altitude %.2f m; using the EKF",
            range_m,
            ekf_alt_m,
        )
        return ekf_alt_m
    return range_m


def commit_hold_velocity(
    lat: float,
    lon: float,
    commit_lat: float,
    commit_lon: float,
) -> tuple[float, float]:
    """(north_m_s, east_m_s) that returns the drone to where it committed.

    The commit stops using the camera because the pad has left the frame. It has
    no reason to stop using the POSITION ESTIMATE, which is accurate over the
    four to six seconds a commit lasts and is the only thing left that knows the
    drone has drifted.

    Commanding zero velocity -- the first implementation -- asks PX4 to stop
    moving rather than to stay put, so drift already accumulated is never taken
    back. Measured over twenty-one landings, that cost 0.170 m on average
    against a 0.042 m offset at commit: four fifths of the final error arrived
    after the controller stopped correcting.

    Deadbanded and clamped, because near the ground with no view of the pad a
    large horizontal command is never right, and correcting inside the
    estimator's own noise is chasing the estimator rather than the drone.
    """
    east_m, north_m = geo.lat_lon_to_local_enu(lat, lon, commit_lat, commit_lon)
    drift_m = math.hypot(north_m, east_m)
    if drift_m < config.COMMIT_HOLD_DEADBAND_M:
        return 0.0, 0.0

    north_cmd = geo.clamp(
        -config.COMMIT_HOLD_K_P * north_m,
        -config.COMMIT_HOLD_MAX_VEL_M_S,
        config.COMMIT_HOLD_MAX_VEL_M_S,
    )
    east_cmd = geo.clamp(
        -config.COMMIT_HOLD_K_P * east_m,
        -config.COMMIT_HOLD_MAX_VEL_M_S,
        config.COMMIT_HOLD_MAX_VEL_M_S,
    )
    return north_cmd, east_cmd


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN LANDING ROUTINE
# ═════════════════════════════════════════════════════════════════════════════


async def execute_precision_landing(
    publisher: SetpointPublisher,
    drone,
    expected_marker_id: int,
    expected_lat: float,
    expected_lon: float,
    vision_data: dict[str, Any],
    drone_state: dict[str, Any],
    *,
    drone_id: str,
    fsm: Any = None,
    search_alt_m: float = config.SEARCH_ALT_M,
) -> str:
    """Search for a pad, centre on it, descend, commit and land.

    Returns a LandingOutcome string. Fires FSM events marker_locked, lock_lost,
    centered_stable, touchdown and search_exhausted.

    `expected_marker_id` is the pad's CENTRE marker id, which is also its pad
    index -- so callers that already say "land on marker 1" are unchanged.

    Precondition: the drone is hovering near (expected_lat, expected_lon) with
    the setpoint publisher running. This function never starts or stops offboard.
    """
    from drone_agent.navigation import altitude_correction, descend_to

    expected_pad = expected_marker_id

    logger.info(
        "[%s] landing on pad %d; vision usable from %.2f m to %.2f m",
        drone_id,
        expected_pad,
        config.PAD_VISION_MIN_ALT_M,
        config.PAD_VISION_MAX_ALT_M,
    )

    # ── Descend to the acquisition altitude first ───────────────────────────
    if drone_state.get("alt", 0.0) > search_alt_m + 1.0:
        await descend_to(publisher, drone_state, search_alt_m, drone_id=drone_id)

    waypoints = generate_spiral_waypoints(expected_lat, expected_lon, altitude_m=search_alt_m)
    timeout_s = search_timeout_s(waypoints)
    logger.info(
        "[%s] spiral: %d waypoints, %.1f m ring spacing, %.0f s budget",
        drone_id,
        len(waypoints),
        config.search_ring_step_m(search_alt_m),
        timeout_s,
    )

    tracker = AlphaBetaTracker()
    ratchet = AltitudeRatchet()
    dt_s = 1.0 / config.NAV_HZ
    started_at = time.monotonic()

    locked = False
    descending = False
    committed = False
    commit_started_at = 0.0
    commit_lat = 0.0
    commit_lon = 0.0
    waypoint_index = 0
    last_seen_at = 0.0
    ground_contact_at = 0.0

    # Stall watchdog, on wall clock rather than only while the pad is visible,
    # so a stall during a lock-loss hold is caught too.
    last_progress_t = started_at
    last_progress_alt = drone_state.get("alt", 0.0)

    def phase(name: str, detail: str) -> None:
        drone_state["landing_phase"] = name
        drone_state["status"] = detail

    while True:
        now = time.monotonic()
        elapsed = now - started_at
        heading_deg = float(drone_state.get("heading_deg", publisher.setpoint.yaw_deg))

        observation = None
        vision_dead = udp_receiver.is_stale(vision_data, udp_receiver.VISION_TS_KEY, now=now)
        if not vision_dead:
            observation = observe_pad(vision_data, expected_pad, drone_state)

        height_m = working_height_m(observation, drone_state)
        ratchet.observe(height_m)

        # ── The open-loop commit ─────────────────────────────────────────────
        # Checked before everything else: once committed, no measurement, no
        # timeout and no lock loss may divert the drone. It is close enough to
        # the ground that the only safe direction is down.
        if committed:
            if is_on_ground(drone_state, height_m):
                if ground_contact_at == 0.0:
                    ground_contact_at = now
                    logger.info("[%s] ground contact at %.2f m", drone_id, height_m)
                if now - ground_contact_at >= config.TOUCHDOWN_PUSH_S:
                    phase(LandingPhase.TOUCHDOWN, "touchdown - confirming")
                    publisher.hold(heading_deg)
                    if await _confirm_landed(drone, drone_id):
                        if fsm is not None:
                            fsm.fire("touchdown")
                        return LandingOutcome.TOUCHDOWN
                    # `_confirm_landed` issued a land command, and that took the
                    # vehicle out of OFFBOARD whether or not it disarmed. Without
                    # putting it back, "continuing to push down" would be a
                    # fiction: the setpoints below would stream into a mode that
                    # is not listening, and the loop would sit here until the
                    # commit timeout believing it was descending.
                    await publisher.ensure_offboard()
                    logger.warning(
                        "[%s] the autopilot did not disarm; re-entered offboard "
                        "and continuing to push down",
                        drone_id,
                    )
                    ground_contact_at = now
            else:
                ground_contact_at = 0.0

            if now - commit_started_at > config.COMMIT_TIMEOUT_S:
                # The commit has run long. Before calling this a failure, hand
                # the vehicle to the autopilot's own land mode and see what it
                # says -- which is what PX4's PrecLand does when its final
                # approach does not conclude, and what ArduPilot's PrecLand
                # failsafe does too.
                #
                # It is also the honest thing to do here. This branch fired on a
                # live flight while the drone was ALREADY SITTING ON THE PAD,
                # 0.12 m from its centre, because LandedState had not caught up
                # (see is_on_ground). Declaring STALLED without asking the
                # autopilot threw away a landing that had succeeded. Leaving
                # OFFBOARD is precisely what lets PX4's land detector settle,
                # because it removes the velocity setpoint that was suppressing
                # it.
                logger.warning(
                    "[%s] commit has run %.0f s and is still %.2f m up - "
                    "handing over to the autopilot's land mode",
                    drone_id,
                    config.COMMIT_TIMEOUT_S,
                    height_m,
                )
                publisher.hold(heading_deg)
                if await _confirm_landed(drone, drone_id):
                    logger.info(
                        "[%s] the autopilot confirmed the landing after the handover",
                        drone_id,
                    )
                    if fsm is not None:
                        fsm.fire("touchdown")
                    return LandingOutcome.TOUCHDOWN

                logger.error(
                    "[%s] commit did not reach the ground within %.0f s and the "
                    "autopilot did not confirm a landing either (still %.2f m up)",
                    drone_id,
                    config.COMMIT_TIMEOUT_S,
                    height_m,
                )
                if fsm is not None:
                    fsm.fire("search_exhausted")
                return LandingOutcome.STALLED

            # HOLD THE SPOT WE COMMITTED FROM.
            #
            # Open loop with respect to VISION -- the pad has left the frame and
            # there is nothing to see. Not open loop with respect to POSITION:
            # the estimator still knows perfectly well where the drone is over
            # the four to six seconds this lasts, and it is the only thing left
            # that can notice a drift.
            #
            # An earlier version commanded zero horizontal velocity and left the
            # rest to PX4, reasoning that PX4 holds a spot better than this loop
            # can extrapolate. True, but a zero VELOCITY setpoint asks PX4 to
            # stop moving, not to stay put, so drift already accumulated was
            # never taken back. Twenty-one landings measured 0.170 m of drift
            # against a 0.042 m offset at commit -- four fifths of the final
            # error arriving after the controller stopped correcting.
            #
            # The version before THAT was worse still: it steered on the
            # alpha-beta tracker and coasted it with predict(), which
            # extrapolates along the last measured velocity with nothing to
            # correct it, so the estimate ramped away and actively pushed the
            # drone off the pad. Position hold is the middle course, and the
            # only one of the three with a feedback term.
            north_cmd, east_cmd = commit_hold_velocity(
                drone_state.get("lat", commit_lat),
                drone_state.get("lon", commit_lon),
                commit_lat,
                commit_lon,
            )
            publisher.command_ned(north_cmd, east_cmd, config.DESCENT_VZ_FINAL_M_S, heading_deg)
            phase(LandingPhase.COMMIT, f"committed: {height_m:.2f} m to touchdown")
            await asyncio.sleep(dt_s)
            continue

        # ── Timeout ─────────────────────────────────────────────────────────
        if elapsed > timeout_s:
            logger.error(
                "[%s] pad %d not landed on within %.0f s", drone_id, expected_pad, timeout_s
            )
            publisher.hold(heading_deg)
            publisher.setpoint.down_m_s = ratchet.limit_climb(
                height_m, altitude_correction(height_m, search_alt_m)
            )
            if fsm is not None:
                fsm.fire("search_exhausted")
            return LandingOutcome.TIMEOUT

        # ── Dead vision feed: fail closed ───────────────────────────────────
        if vision_dead:
            phase(LandingPhase.HOLD, "vision feed dead - holding")
            if descending:
                logger.error(
                    "[%s] vision feed went silent at %.1f m - pausing the descent",
                    drone_id,
                    height_m,
                )
                descending = False
            publisher.hold(heading_deg)
            # Hold altitude rather than climbing: a dead camera is a reason to
            # stop, not a reason to undo the descent already achieved.
            publisher.setpoint.down_m_s = 0.0
            await asyncio.sleep(dt_s)
            continue

        # ── Pad in view ─────────────────────────────────────────────────────
        if observation is not None:
            last_seen_at = now
            if not locked:
                locked = True
                logger.info(
                    "[%s] pad %d locked at %.2f m via markers %s",
                    drone_id,
                    expected_pad,
                    height_m,
                    observation["marker_ids"],
                )
                if fsm is not None:
                    fsm.fire("marker_locked")

            filtered = tracker.update((observation["forward_m"], observation["right_m"]), dt_s)
        else:
            # Not seen this tick. Within the grace period this is a dropped
            # frame, not a lost lock: coast on the velocity state.
            filtered = tracker.predict(dt_s) if locked else None
            if locked and now - last_seen_at > config.LOCK_LOSS_GRACE_S:
                locked = False
                descending = False
                tracker.reset()
                filtered = None
                logger.warning(
                    "[%s] lost pad %d at %.2f m after %.1f s without a detection",
                    drone_id,
                    expected_pad,
                    height_m,
                    now - last_seen_at,
                )
                # Only tell the FSM we are searching again if searching is
                # something this loop would actually do.
                #
                # Below NO_CLIMB_ALT_M it is not. Losing the pad down there is
                # the field of view doing exactly what it must -- the markers
                # leave the frame as the drone closes on them -- and the loop
                # responds by holding and then committing, never by climbing
                # away to look again. Firing `lock_lost` regardless drove the
                # FSM APPROACH -> SEARCHING at the very moment the landing was
                # being committed, and SEARCHING has no `touchdown` transition,
                # so the successful landing that followed was rejected:
                #
                #   15:16:55  no lock at 1.45 m but already below the commit
                #             altitude - committing on the last known trim
                #   15:17:13  disarmed - landing confirmed
                #   15:17:13  event 'touchdown' is not valid in SEARCHING
                #   15:17:13  event 'op_confirmed' is not valid in SEARCHING
                #   15:17:16  event 'more_legs' is not valid in SEARCHING
                #
                # The drone had landed and picked up its cargo; the state
                # machine reporting on it had been left behind in SEARCHING and
                # rejected every event for the rest of the mission. The FSM's
                # transition table was right and the event was wrong.
                if fsm is not None and may_climb(height_m):
                    fsm.fire("lock_lost")

        # ── Guided flight, on a live or coasted estimate ─────────────────────
        if filtered is not None:
            forward_m, right_m = filtered
            offset_m = math.hypot(forward_m, right_m)

            # ── Commit? ──────────────────────────────────────────────────────
            if height_m <= config.COMMIT_ALT_M and offset_m <= config.COMMIT_MAX_OFFSET_M:
                committed = True
                commit_started_at = now
                commit_lat = drone_state.get("lat", 0.0)
                commit_lon = drone_state.get("lon", 0.0)
                logger.info(
                    "[%s] committing to the landing from %.2f m, %.3f m off centre",
                    drone_id,
                    height_m,
                    offset_m,
                )
                continue

            if offset_m < config.LANDING_DEADBAND_M:
                forward_cmd = right_cmd = 0.0
            else:
                # PD: position from the tracker, damping from its velocity
                # state. The velocity term opposes the closing rate, which is
                # what keeps the approach from overshooting when the loop delay
                # is longer than modelled.
                forward_cmd = geo.clamp(
                    config.LANDING_K_P * forward_m + config.LANDING_K_D * tracker.velocity[0],
                    -config.LANDING_MAX_VEL_M_S,
                    config.LANDING_MAX_VEL_M_S,
                )
                right_cmd = geo.clamp(
                    config.LANDING_K_P * right_m + config.LANDING_K_D * tracker.velocity[1],
                    -config.LANDING_MAX_VEL_M_S,
                    config.LANDING_MAX_VEL_M_S,
                )

            if offset_m < config.CENTERED_M and not descending and fsm is not None:
                if getattr(fsm.state, "value", None) == "APPROACH":
                    fsm.fire("centered_stable")

            # ── Descent gate: pass -> descend, fail -> PAUSE (never climb) ───
            was_descending = descending
            descending = should_descend(offset_m, height_m, descending)
            if descending:
                down_m_s = descent_rate_m_s(height_m)
                if not was_descending:
                    logger.info(
                        "[%s] descending: %.3f m off at %.2f m (gate %.3f m)",
                        drone_id,
                        offset_m,
                        height_m,
                        descent_gate_limit_m(height_m),
                    )
                phase(
                    LandingPhase.DESCEND,
                    f"landing on {expected_pad}: {offset_m:.2f} m off, {height_m:.2f} m up",
                )
            else:
                down_m_s = 0.0
                if was_descending:
                    logger.info(
                        "[%s] pausing the descent at %.2f m: %.3f m off, gate is %.3f m",
                        drone_id,
                        height_m,
                        offset_m,
                        descent_gate_limit_m(height_m),
                    )
                phase(
                    LandingPhase.APPROACH,
                    f"centring on {expected_pad}: {offset_m:.2f} m off, {height_m:.2f} m up",
                )

            down_m_s = ratchet.limit_climb(height_m, down_m_s)
            publisher.command_body_horizontal(forward_cmd, right_cmd, down_m_s, heading_deg)

            # ── Touchdown without a commit (a low pad, or an early contact) ──
            if is_on_ground(drone_state, height_m):
                if ground_contact_at == 0.0:
                    ground_contact_at = now
                elif now - ground_contact_at >= config.TOUCHDOWN_PUSH_S:
                    phase(LandingPhase.TOUCHDOWN, "touchdown - confirming")
                    publisher.hold(heading_deg)
                    if await _confirm_landed(drone, drone_id):
                        if fsm is not None:
                            fsm.fire("touchdown")
                        return LandingOutcome.TOUCHDOWN
                    # The land command dropped OFFBOARD even though it did not
                    # disarm. Put it back, or every setpoint after this is
                    # published into a mode that ignores it.
                    await publisher.ensure_offboard()
                    ground_contact_at = now
            else:
                ground_contact_at = 0.0

        # ── Pad not in view and the lock is genuinely gone ───────────────────
        else:
            if not may_climb(height_m):
                # Below NO_CLIMB_ALT_M, losing the pad is what the field of view
                # does, not an emergency. Hold and let the grace period or the
                # commit gate resolve it -- climbing here is the exact behaviour
                # that produced the reported hover.
                phase(
                    LandingPhase.HOLD,
                    f"pad {expected_pad} not in view at {height_m:.2f} m - holding",
                )
                publisher.hold(heading_deg)
                publisher.setpoint.down_m_s = 0.0

                # If we are low and centred enough, commit rather than sit here.
                if height_m <= config.COMMIT_ALT_M + 0.3 and tracker.position is None:
                    logger.info(
                        "[%s] no lock at %.2f m, already below the commit "
                        "altitude - committing and holding position for the "
                        "last %.2f m",
                        drone_id,
                        height_m,
                        height_m,
                    )
                    committed = True
                    commit_started_at = now
                    commit_lat = drone_state.get("lat", 0.0)
                    commit_lon = drone_state.get("lon", 0.0)
                await asyncio.sleep(dt_s)
                continue

            if waypoint_index >= len(waypoints):
                logger.error(
                    "[%s] spiral exhausted (%d waypoints) without seeing pad %d",
                    drone_id,
                    len(waypoints),
                    expected_pad,
                )
                publisher.hold(heading_deg)
                publisher.setpoint.down_m_s = ratchet.limit_climb(
                    height_m, altitude_correction(height_m, search_alt_m)
                )
                if fsm is not None:
                    fsm.fire("search_exhausted")
                return LandingOutcome.SEARCH_EXHAUSTED

            wp_lat, wp_lon = waypoints[waypoint_index]
            distance_m = geo.get_distance_m(drone_state["lat"], drone_state["lon"], wp_lat, wp_lon)

            if distance_m < 1.0:
                waypoint_index += 1
                if waypoint_index % 10 == 0:
                    logger.info(
                        "[%s] search waypoint %d/%d",
                        drone_id,
                        waypoint_index,
                        len(waypoints),
                    )
            else:
                bearing = geo.get_bearing(drone_state["lat"], drone_state["lon"], wp_lat, wp_lon)
                north, east = geo.bearing_to_ned(bearing, config.SEARCH_SPEED_M_S)
                publisher.command_ned(
                    north,
                    east,
                    ratchet.limit_climb(height_m, altitude_correction(height_m, search_alt_m)),
                    bearing,
                )
                phase(
                    LandingPhase.SEARCH,
                    f"searching for {expected_pad}: waypoint {waypoint_index}/{len(waypoints)}",
                )

        # ── Stall watchdog ──────────────────────────────────────────────────
        if now - last_progress_t >= config.STALL_WINDOW_S:
            descended_m = last_progress_alt - height_m
            if locked and descended_m < config.STALL_MIN_DESCENT_M:
                logger.error(
                    "[%s] pad held but only %.2f m descended in %.0f s at %.2f m - "
                    "aborting rather than hovering to the timeout",
                    drone_id,
                    descended_m,
                    config.STALL_WINDOW_S,
                    height_m,
                )
                publisher.hold(heading_deg)
                if fsm is not None:
                    fsm.fire("search_exhausted")
                return LandingOutcome.STALLED
            last_progress_t = now
            last_progress_alt = height_m

        await asyncio.sleep(dt_s)


async def _confirm_landed(drone, drone_id: str, timeout_s: float = 12.0) -> bool:
    """Issue land, then wait for the autopilot to confirm disarm.

    Waiting for `armed == False` rather than assuming the land command worked is
    what makes touchdown a fact instead of an intention. Bounded, because an
    unbounded `async for armed in drone.telemetry.armed()` hangs the mission
    forever if the drone never disarms.
    """
    try:
        await asyncio.wait_for(drone.action.land(), timeout=5.0)
    except Exception as exc:  # noqa: BLE001
        logger.warning("[%s] land command failed: %s", drone_id, exc)

    async def _wait_disarmed() -> bool:
        async for armed in drone.telemetry.armed():
            if not armed:
                return True
        return False

    try:
        disarmed = await asyncio.wait_for(_wait_disarmed(), timeout=timeout_s)
    except asyncio.TimeoutError:
        logger.warning("[%s] still armed %.0f s after the land command", drone_id, timeout_s)
        return False

    if disarmed:
        logger.info("[%s] disarmed - landing confirmed", drone_id)
    return disarmed
