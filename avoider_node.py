#!/usr/bin/env python3
"""LiDAR reactive obstacle avoidance with time hysteresis and escalation.

Subscribes to one drone's 2D LiDAR (a ROS LaserScan republished from Gazebo by
ros_gz_bridge), decides between cruising and dodging, and broadcasts the
decision as JSON over UDP to that drone's mission process.

Output contract (see docs/ARCHITECTURE.md):

    {"t": <monotonic s>, "seq": <int>, "action": <str>,
     "eff_front_m": <float>, "med_left_m": <float>, "med_right_m": <float>,
     "dodge_age_s": <float>, "drone_id": <str>}

    action in {CLEAR, DODGE_LEFT, DODGE_RIGHT, ESCALATE}

A datagram is sent for EVERY scan, including clear ones. That is what makes
silence meaningful: the consumer treats a feed that has gone quiet for
config.STALE_SENSOR_TIMEOUT_S as a dead sensor and holds position, rather than
inferring "no news is good news" and flying blind into a wall.

WHAT CHANGED AND WHY
--------------------
1. The decision logic is now the pure function `decide_action`, with exactly
   the signature tests/test_avoider_node.py was written against. That test
   existed for months against an API that had never been built, so the state
   machine had zero coverage.

2. ESCALATE is finally emitted. config.ESCALATION_LOCK_S and
   navigation.py both anticipated an action that nothing had ever sent, so
   "obstacle escalation after 12 s triggers detour planning" -- documented
   behaviour -- could not occur.

3. Thresholds come from config instead of being re-declared as class constants.
   Five values were shadowed here, so tuning config.py did nothing.

4. Returns below config.MIN_VALID_RANGE_M are masked as self-hits. The props
   sit about 0.29 m out and the sensor floor is 0.15 m, so a prop tip reads as
   a permanent 0.29 m wall: the avoider locks a dodge it can never clear.

5. Timing is time.monotonic(). Every duration here is an elapsed-time
   measurement, and wall clock is subject to NTP steps -- a backwards step
   mid-flight makes a timeout fire instantly or never.

6. Scan geometry is validated against config on the first message, so a model
   change that moves the front cone somewhere other than the front is a loud
   startup failure rather than a subtly wrong flight.

Usage:
    python3 avoider_node.py --drone-id drone-0
    python3 avoider_node.py --drone-id drone-1 --scan-topic /drone_1/scan
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import sys
import time

import config

logger = logging.getLogger("avoider")

# Action vocabulary. The consumer switches on these exact strings; keep them and
# docs/ARCHITECTURE.md in sync.
CLEAR = "CLEAR"
DODGE_LEFT = "DODGE_LEFT"
DODGE_RIGHT = "DODGE_RIGHT"
ESCALATE = "ESCALATE"


# ═════════════════════════════════════════════════════════════════════════════
#  PURE DECISION FUNCTION
# ═════════════════════════════════════════════════════════════════════════════

def decide_action(
    eff_front,
    med_left,
    med_right,
    now,
    dodge_dir,
    dodge_start_t,
    clear_first_seen,
    SAFE_DIST,          # noqa: N803 - signature frozen by tests/test_avoider_node.py
    CLEAR_DIST,         # noqa: N803
    CLEAR_CONFIRM_S,    # noqa: N803
    MIN_LOCK_S,         # noqa: N803
    ESCALATION_LOCK_S,  # noqa: N803
):
    """Decide the avoidance action for one scan. Pure: no I/O, no clock read.

    All state is passed in and handed back, which is what makes the state
    machine testable without ROS, without Gazebo and without wall-clock
    dependence. The thresholds are parameters rather than config lookups for
    the same reason: a test can drive an escalation in three calls instead of
    waiting twelve real seconds.

    Args:
        eff_front: Effective clear distance ahead, metres. Combines the median
            and the minimum of the front cone, so a thin obstacle that only one
            beam sees still counts.
        med_left: Median range in the left side sector, metres.
        med_right: Median range in the right side sector, metres.
        now: Current time from a MONOTONIC source, seconds. Only differences
            are used, so the epoch is irrelevant -- but it must not run
            backwards.
        dodge_dir: Currently latched dodge direction, or None when cruising.
        dodge_start_t: `now` at which the current dodge latched.
        clear_first_seen: `now` at which the front first looked clear during
            this dodge, or 0.0 if it has not yet.
        SAFE_DIST: Closer than this in front triggers a dodge, metres.
        CLEAR_DIST: The front must exceed this to start the clear timer, metres.
            Deliberately greater than SAFE_DIST: the gap between them is the
            hysteresis band that stops the drone oscillating at a wall edge.
        CLEAR_CONFIRM_S: How long the front must stay clear before unlatching.
        MIN_LOCK_S: Minimum dodge duration before clearing is even considered,
            so one lucky clear reading cannot abort a dodge that just started.
        ESCALATION_LOCK_S: Dodging continuously for this long means reactive
            avoidance is not going to resolve this obstacle.

    Returns:
        (action, dodge_dir, dodge_start_t, clear_first_seen) -- the action to
        broadcast plus the state to pass into the next call.
    """
    # ── Obstacle ahead ───────────────────────────────────────────────────────
    if eff_front < SAFE_DIST:
        clear_first_seen = 0.0          # any clear progress is invalidated

        if dodge_dir is None:
            # First detection: commit to the side with more room, and stay
            # committed. Re-deciding every tick is what produces ping-ponging.
            dodge_dir = DODGE_LEFT if med_left >= med_right else DODGE_RIGHT
            dodge_start_t = now
            return dodge_dir, dodge_dir, dodge_start_t, clear_first_seen

        # Already dodging. If we have been at it this long, sideways motion is
        # not working -- ask for a planned route instead.
        #
        # LEVEL-TRIGGERED, NOT EDGE-TRIGGERED: ESCALATE is re-sent on every
        # scan while the condition holds. UDP is lossy, so a once-only edge
        # signal that happened to be dropped would never fire again and the
        # drone would dodge into the wall forever. The consumer latches it and
        # bounds re-planning with config.DETOUR_MAX_ATTEMPTS_PER_LEG.
        if now - dodge_start_t >= ESCALATION_LOCK_S:
            return ESCALATE, dodge_dir, dodge_start_t, clear_first_seen

        return dodge_dir, dodge_dir, dodge_start_t, clear_first_seen

    # ── Front is not dangerous, but we are mid-dodge ─────────────────────────
    if dodge_dir is not None:
        time_dodging = now - dodge_start_t

        if time_dodging < MIN_LOCK_S:
            return dodge_dir, dodge_dir, dodge_start_t, clear_first_seen

        if eff_front >= CLEAR_DIST:
            if clear_first_seen == 0.0:
                clear_first_seen = now

            if now - clear_first_seen >= CLEAR_CONFIRM_S:
                return CLEAR, None, dodge_start_t, 0.0

            # Clear, but not for long enough yet. Keep dodging.
            return dodge_dir, dodge_dir, dodge_start_t, clear_first_seen

        # Hysteresis band: SAFE_DIST <= eff_front < CLEAR_DIST. Not dangerous,
        # not confidently clear either. Hold the dodge and reset the timer.
        return dodge_dir, dodge_dir, dodge_start_t, 0.0

    # ── Truly clear ──────────────────────────────────────────────────────────
    return CLEAR, None, dodge_start_t, clear_first_seen


# ═════════════════════════════════════════════════════════════════════════════
#  SCAN PROCESSING
# ═════════════════════════════════════════════════════════════════════════════

def clean_range(value: float) -> float:
    """Map one raw return to a usable distance.

    Two classes of reading are not obstacles:

    * inf/nan/zero -- nothing within range. Substituting INF_REPLACE_M (which
      must exceed CLEAR_DIST) is what lets an empty world read as clear.
    * anything below MIN_VALID_RANGE_M -- the drone's own propellers. They sit
      about 0.29 m out, above the sensor's 0.15 m floor, so these are genuine
      returns from a real object; they are simply not an obstacle to avoid.
      Without this mask a single prop return is an unclearable wall.
    """
    if not math.isfinite(value) or value <= 0.0:
        return config.INF_REPLACE_M
    if value < config.MIN_VALID_RANGE_M:
        return config.INF_REPLACE_M
    return value


def median(values: list[float]) -> float:
    """Median, or INF_REPLACE_M for an empty sector."""
    if not values:
        return config.INF_REPLACE_M
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def sector_ranges(
    ranges: list[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    start_deg: int,
    end_deg: int,
) -> list[float]:
    """Cleaned returns for the angular sector [start_deg, end_deg].

    Angles are degrees relative to straight ahead, counter-clockwise positive,
    matching the scan's own convention. Indices are derived from the message's
    angle_min and angle_increment rather than assumed, so the sector still
    points where it should if the sensor's resolution changes.
    """
    count = len(ranges)
    if count == 0 or angle_increment_rad == 0.0:
        return []

    out = []
    for deg in range(start_deg, end_deg + 1):
        index = round((math.radians(deg) - angle_min_rad) / angle_increment_rad) % count
        out.append(clean_range(ranges[index]))
    return out


def summarize_scan(
    ranges: list[float],
    angle_min_rad: float,
    angle_increment_rad: float,
) -> tuple[float, float, float]:
    """Reduce a full scan to (eff_front_m, med_left_m, med_right_m).

    eff_front takes the minimum of the front cone's median and its outright
    minimum. The median alone ignores a narrow obstacle that only a few beams
    see -- a mast, a pole, the edge of a wall. The minimum alone jumps at a
    single noisy beam. min(median, min) reacts to a thin real obstacle while
    the MIN_VALID_RANGE_M mask handles the noise the raw minimum would import.
    """
    front = sector_ranges(
        ranges, angle_min_rad, angle_increment_rad,
        -config.FRONT_HALF_DEG, config.FRONT_HALF_DEG,
    )
    left = sector_ranges(
        ranges, angle_min_rad, angle_increment_rad,
        config.SIDE_START_DEG, config.SIDE_END_DEG,
    )
    right = sector_ranges(
        ranges, angle_min_rad, angle_increment_rad,
        -config.SIDE_END_DEG, -config.SIDE_START_DEG,
    )

    if not front:
        # No usable front data is not the same as a clear path.
        return 0.0, median(left), median(right)

    eff_front = min(median(front), min(front))
    return eff_front, median(left), median(right)


def validate_scan_geometry(
    sample_count: int,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_max_m: float,
) -> list[str]:
    """Compare the live scan against config. Returns a list of complaints.

    The sector maths above converts degrees to array indices using angle_min
    and angle_increment. If the sensor's geometry stops matching config, the
    "front cone" quietly starts pointing somewhere else, and the drone dodges
    obstacles that are beside it while flying into ones ahead. Loud on startup
    beats subtly wrong in flight.
    """
    problems = []

    if sample_count != config.LIDAR_SAMPLES:
        problems.append(
            f"sample count {sample_count} != config.LIDAR_SAMPLES {config.LIDAR_SAMPLES}")

    if not math.isclose(angle_min_rad, config.LIDAR_ANGLE_MIN_RAD, abs_tol=1e-3):
        problems.append(
            f"angle_min {angle_min_rad:.5f} != config.LIDAR_ANGLE_MIN_RAD "
            f"{config.LIDAR_ANGLE_MIN_RAD:.5f}")

    expected_increment = (
        (config.LIDAR_ANGLE_MAX_RAD - config.LIDAR_ANGLE_MIN_RAD) / config.LIDAR_SAMPLES
    )
    if not math.isclose(angle_increment_rad, expected_increment, rel_tol=2e-2):
        problems.append(
            f"angle_increment {angle_increment_rad:.5f} != expected "
            f"{expected_increment:.5f}")

    if range_max_m > 0 and not math.isclose(
        range_max_m, config.LIDAR_RANGE_MAX_M, rel_tol=1e-2
    ):
        problems.append(
            f"range_max {range_max_m:.2f} != config.LIDAR_RANGE_MAX_M "
            f"{config.LIDAR_RANGE_MAX_M:.2f}")

    if config.INF_REPLACE_M <= config.CLEAR_DIST:
        problems.append(
            f"config.INF_REPLACE_M ({config.INF_REPLACE_M}) must exceed "
            f"config.CLEAR_DIST ({config.CLEAR_DIST}) or an empty world never "
            f"reads as clear")

    return problems


# ═════════════════════════════════════════════════════════════════════════════
#  ROS NODE
# ═════════════════════════════════════════════════════════════════════════════

def build_node_class():
    """Define the ROS node lazily so this module imports without rclpy.

    tests/test_avoider_node.py imports `decide_action` from here. Importing
    rclpy at module scope would make that test depend on a sourced ROS
    environment, which is exactly the kind of coupling that turns one missing
    dependency into a suite that collects nothing.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    class LidarAvoider(Node):
        """Publishes one avoidance decision per LiDAR scan over UDP."""

        def __init__(self, drone_id: str, scan_topic: str, udp_host: str, udp_port: int):
            super().__init__(f"lidar_avoider_{config.drone_index(drone_id)}")

            self._drone_id = drone_id
            self._udp_addr = (udp_host, udp_port)
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Decision state, threaded through decide_action.
            self._dodge_dir: str | None = None
            self._dodge_start_t = 0.0
            self._clear_first_seen = 0.0

            self._seq = 0
            self._geometry_checked = False
            self._last_action = CLEAR

            self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

            self.get_logger().info(
                f"avoider up: {drone_id} | scan={scan_topic} | "
                f"udp={udp_host}:{udp_port} | "
                f"safe={config.SAFE_DIST}m clear={config.CLEAR_DIST}m "
                f"escalate_after={config.ESCALATION_LOCK_S}s "
                f"self_hit_mask={config.MIN_VALID_RANGE_M}m"
            )

        def _on_scan(self, msg) -> None:
            now = time.monotonic()

            if not self._geometry_checked:
                self._geometry_checked = True
                problems = validate_scan_geometry(
                    len(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max
                )
                if problems:
                    # Fatal: the sector maths below would be silently wrong.
                    for problem in problems:
                        self.get_logger().error(f"scan geometry mismatch: {problem}")
                    self.get_logger().error(
                        "Refusing to publish avoidance decisions from a scan whose "
                        "geometry does not match config.py. Fix "
                        "sim/models/x500_delivery/model.sdf or config.py, then "
                        "re-run scripts/preflight.py."
                    )
                    raise SystemExit(2)
                self.get_logger().info(
                    f"scan geometry verified: {len(msg.ranges)} samples, "
                    f"angle_min={msg.angle_min:.4f} rad, range_max={msg.range_max:.1f} m"
                )

            eff_front, med_left, med_right = summarize_scan(
                list(msg.ranges), msg.angle_min, msg.angle_increment
            )

            action, self._dodge_dir, self._dodge_start_t, self._clear_first_seen = (
                decide_action(
                    eff_front, med_left, med_right, now,
                    self._dodge_dir, self._dodge_start_t, self._clear_first_seen,
                    config.SAFE_DIST, config.CLEAR_DIST, config.CLEAR_CONFIRM_S,
                    config.MIN_LOCK_S, config.ESCALATION_LOCK_S,
                )
            )

            dodge_age_s = (
                now - self._dodge_start_t if self._dodge_dir is not None else 0.0
            )

            self._seq += 1
            payload = {
                "t": now,
                "seq": self._seq,
                "action": action,
                "eff_front_m": round(eff_front, 3),
                "med_left_m": round(med_left, 3),
                "med_right_m": round(med_right, 3),
                "dodge_age_s": round(dodge_age_s, 3),
                "drone_id": self._drone_id,
            }

            try:
                self._sock.sendto(json.dumps(payload).encode(), self._udp_addr)
            except OSError as exc:
                # Never let a transport hiccup kill the callback: the consumer's
                # staleness check already handles a gap correctly.
                self.get_logger().warning(f"UDP send failed: {exc}")

            # Log transitions only. At 10 Hz, logging every scan buries the
            # events that matter -- and with three drones it is unreadable.
            if action != self._last_action:
                level = (
                    self.get_logger().error if action == ESCALATE
                    else self.get_logger().warning if action != CLEAR
                    else self.get_logger().info
                )
                level(
                    f"{self._last_action} -> {action} | front={eff_front:.1f}m "
                    f"left={med_left:.1f}m right={med_right:.1f}m "
                    f"dodge_age={dodge_age_s:.1f}s"
                )
                self._last_action = action

        def destroy_node(self) -> bool:
            self._sock.close()
            return super().destroy_node()

    return rclpy, LidarAvoider


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="LiDAR reactive avoidance for one drone.",
    )
    parser.add_argument(
        "--drone-id", default="drone-0",
        help="Which drone this instance serves (default: drone-0). Determines "
             "the UDP port and, unless overridden, the scan topic.",
    )
    parser.add_argument(
        "--scan-topic", default=None,
        help="ROS LaserScan topic. Default: derived from --drone-id via "
             "sim_topics.ros_scan_topic().",
    )
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument(
        "--udp-port", type=int, default=None,
        help="Default: config.lidar_port(drone_id).",
    )
    parser.add_argument("--log-level", default="INFO")
    args, ros_args = parser.parse_known_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s [avoider] %(message)s",
    )

    try:
        config.drone_index(args.drone_id)
    except ValueError as exc:
        print(f"avoider_node: {exc}", file=sys.stderr)
        return 2

    import sim_topics

    scan_topic = args.scan_topic or sim_topics.ros_scan_topic(args.drone_id)
    udp_port = args.udp_port if args.udp_port is not None else config.lidar_port(args.drone_id)

    rclpy, LidarAvoider = build_node_class()

    rclpy.init(args=ros_args or None)
    node = LidarAvoider(args.drone_id, scan_topic, args.udp_host, udp_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001 - shutdown races are not actionable
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
