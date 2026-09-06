#!/usr/bin/env python3
"""docs/ACCEPTANCE.md section 7 -- twenty scripted landings, flown and scored.

    python3 scripts/sitl_landing_trial.py               # 7 missions = 21 landings
    python3 scripts/sitl_landing_trial.py --missions 3
    python3 scripts/sitl_landing_trial.py --analyse-only   # score logs already written

Requires a live rig: `world/spawn_fleet.sh`, then `scripts/run_system.sh`.

WHY THIS DISPATCHES MISSIONS INSTEAD OF CALLING THE LANDING FUNCTION
====================================================================
ACCEPTANCE.md used to say this script should "reuse
`drone_agent.landing.execute_precision_landing` directly rather than going
through the dispatcher, so a failure is attributable to the landing controller
and not to the mission around it".

The first live session with the rewritten controller argued the other way, and
the evidence won. Four defects turned up in that session and THREE OF THEM WERE
NOT IN THE LANDING CONTROLLER:

  * touchdown was never detected, because PX4 does not report `ON_GROUND` while
    OFFBOARD is streaming it a descent;
  * leg 2 hovered until timeout, because nothing re-entered OFFBOARD after the
    land command that confirms a touchdown;
  * the mission FSM was driven to SEARCHING on the way down and then rejected
    the touchdown it had just achieved.

Each one made the delivery fail. A harness that bypassed the mission would have
reported four flawless landings and a system that cannot deliver a package.

Attribution is cheap to recover afterwards -- the per-attempt table names the
phase each attempt reached -- while coverage given up by bypassing the real path
is not recoverable at all.

Each dispatch flies three landings (pickup, drop, home), so seven dispatches
clear the twenty-landing sample.

SCORING
=======
Touchdown error comes from GAZEBO GROUND TRUTH, never from the drone's own EKF:
the controller steers on that EKF, so grading it with the same numbers would be
marking its own homework. The other three criteria are read back out of the
mission and vision logs, which record what happened rather than what this script
believed was happening.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DISPATCH_URL = "http://127.0.0.1:5000"
MODEL = "x500_delivery_0"
LOG_DIR = os.environ.get("LOG_DIR", "/tmp/droneprogram")
MISSION_LOG = os.path.join(LOG_DIR, "fleet_dispatch.log")
VISION_LOG = os.path.join(LOG_DIR, "vision_0.log")

HOME = (30.0314722223, 72.3140833333)  # sim/worlds/delivery.sdf spherical_coordinates
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = 96380.0  # at latitude 30.03

WALL_NORTH_LIMIT_M = 8.0
"""Routes stay south of this. `great_wall` sits at y = +15 m, spans x = -15..15
and is 20 m tall, so a route through it would exercise obstacle avoidance rather
than landing."""


# ─────────────────────────────────────────────────────────────────────────────
#  Routes
# ─────────────────────────────────────────────────────────────────────────────


def route_for(mission_index: int) -> tuple[tuple, tuple]:
    """A distinct pickup and drop per mission.

    Flying one route twenty-one times samples a single approach bearing and a
    single descent geometry, which says very little about a controller that has
    to work from any direction. These spread the bearing over the full circle
    and the range over 15-35 m.
    """
    rng = random.Random(4200 + mission_index)

    def point():
        while True:
            bearing = rng.uniform(0.0, 360.0)
            distance = rng.uniform(15.0, 35.0)
            north = distance * math.cos(math.radians(bearing))
            east = distance * math.sin(math.radians(bearing))
            if north < WALL_NORTH_LIMIT_M:
                return (
                    HOME[0] + north / M_PER_DEG_LAT,
                    HOME[1] + east / M_PER_DEG_LON,
                    round(distance, 1),
                    round(bearing),
                )

    return point(), point()


# ─────────────────────────────────────────────────────────────────────────────
#  Gazebo ground truth
# ─────────────────────────────────────────────────────────────────────────────


class GroundTruth:
    """Poses straight from Gazebo, independent of everything the drone believes."""

    def __init__(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self._poses: dict[str, tuple[float, float, float]] = {}
        self._lock = threading.Lock()
        self._node = Node()
        for topic in ("/world/delivery/pose/info", "/world/delivery/dynamic_pose/info"):
            self._node.subscribe(Pose_V, topic, self._on_pose)

    def _on_pose(self, msg) -> None:
        with self._lock:
            for pose in msg.pose:
                if pose.name == MODEL or "_pad" in pose.name:
                    self._poses[pose.name] = (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    )

    def touchdown_error_m(self) -> tuple[str | None, float | None]:
        """Horizontal distance from the drone to the nearest pad centre."""
        with self._lock:
            poses = dict(self._poses)
        if MODEL not in poses:
            return None, None
        x, y, _ = poses[MODEL]
        pads = {k: v for k, v in poses.items() if "_pad" in k}
        if not pads:
            return None, None
        name, pad = min(pads.items(), key=lambda kv: math.hypot(x - kv[1][0], y - kv[1][1]))
        return name, math.hypot(x - pad[0], y - pad[1])


# ─────────────────────────────────────────────────────────────────────────────
#  Dispatch API
# ─────────────────────────────────────────────────────────────────────────────


def _post(path: str, payload: dict) -> dict:
    request = urllib.request.Request(
        DISPATCH_URL + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.loads(response.read())


def _get(path: str) -> dict:
    with urllib.request.urlopen(DISPATCH_URL + path, timeout=30) as response:
        return json.loads(response.read())


def _log_lines() -> int:
    try:
        with open(MISSION_LOG, errors="ignore") as fh:
            return sum(1 for _ in fh)
    except OSError:
        return 0


def fly(n_missions: int) -> dict:
    truth = GroundTruth()
    time.sleep(2.0)

    landings, missions = [], []
    for index in range(n_missions):
        start_line = _log_lines()
        pickup, drop = route_for(index)
        print(f"\n{'=' * 74}\nMISSION {index + 1}/{n_missions}\n{'=' * 74}", flush=True)
        print(
            f"  pickup {pickup[2]} m @ {pickup[3]} deg   drop {drop[2]} m @ {drop[3]} deg",
            flush=True,
        )

        try:
            response = _post(
                "/dispatch",
                {
                    "pickup_lat": pickup[0],
                    "pickup_lon": pickup[1],
                    "drop_lat": drop[0],
                    "drop_lon": drop[1],
                },
            )
        except Exception as exc:  # noqa: BLE001
            print(f"  dispatch failed: {exc}")
            break

        job = response.get("job_id")
        print(f"  job {job}: {response.get('status')}", flush=True)
        if not job:
            break

        seen, status, deadline, tick = 0, "UNKNOWN", time.time() + 1200, 0
        lines: list[str] = []
        while time.time() < deadline:
            time.sleep(1.0)
            tick += 1
            try:
                with open(MISSION_LOG, errors="ignore") as fh:
                    lines = fh.readlines()[start_line:]
            except OSError:
                lines = []

            # Sample ground truth the moment the controller says it has landed.
            confirmed = sum(1 for line in lines if "disarmed - landing confirmed" in line)
            while confirmed > seen:
                seen += 1
                pad, error_m = truth.touchdown_error_m()
                landings.append(
                    {
                        "mission": index + 1,
                        "leg": seen,
                        "pad": pad,
                        "error_m": None if error_m is None else round(error_m, 3),
                    }
                )
                shown = "?" if error_m is None else f"{error_m:.3f}"
                print(f"  leg {seen} touchdown on {pad}: {shown} m", flush=True)

            # Poll the job endpoint sparingly: every request writes an access-log
            # line into the same file the touchdowns are read from.
            if tick % 5:
                continue
            try:
                status = _get(f"/jobs/{job}").get("status", "UNKNOWN")
            except Exception:  # noqa: BLE001
                status = "UNKNOWN"
            if status in ("COMPLETED", "FAILED", "ABORTED"):
                break

        missions.append({"mission": index + 1, "job": job, "status": status, "touchdowns": seen})
        print(f"  -> mission {status}, {seen} touchdown(s)", flush=True)
        time.sleep(6.0)

    return {"landings": landings, "missions": missions}


# ─────────────────────────────────────────────────────────────────────────────
#  Reading the four criteria back out of the logs
# ─────────────────────────────────────────────────────────────────────────────

_TS = r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d),(\d{3})"

_LEG = re.compile(r"LEG (\w+) -> \([-\d.]+, [-\d.]+\), marker (\d+)")
_START = re.compile(_TS + r".*landing on pad (\d+); vision usable")
_LOCK = re.compile(_TS + r".*pad (\d+) locked at ([\d.]+) m via markers \[([\d, ]*)\]")
_COMMIT = re.compile(_TS + r".*committing to the landing from ([\d.]+) m, ([\d.]+) m off")
_LOWCOMMIT = re.compile(_TS + r".*no lock at ([\d.]+) m,? (?:but )?already below the commit")
_CONTACT = re.compile(_TS + r".*ground contact at ([-\d.]+) m")
_CONFIRM = re.compile(_TS + r".*disarmed - landing confirmed")
_ENDED = re.compile(_TS + r".*landing ended as '(\w+)'")
_INVALID = re.compile(r"event '(\w+)' is not valid in (\w+)")
_VISION = re.compile(_TS + r".*pad (\d+): ([\d.]+) m, offset \(([-\d.]+), ([-\d.]+)\)")


def _seconds(date_part: str, millis: str) -> float:
    hh, mm, ss = date_part.split(" ")[1].split(":")
    return int(hh) * 3600 + int(mm) * 60 + int(ss) + int(millis) / 1000.0


def _sign_changes(values: list[float], deadband: float = 0.02) -> int:
    signs = [1 if v > deadband else (-1 if v < -deadband else 0) for v in values]
    signs = [s for s in signs if s]
    # strict=False is correct here: pairing a list with its own tail is
    # deliberately one element shorter.
    return sum(1 for a, b in zip(signs, signs[1:], strict=False) if a != b)


def analyse(mission_log: str = MISSION_LOG, vision_log: str = VISION_LOG) -> dict:
    attempts: list[dict] = []
    invalid_events: list[str] = []
    current, expected_marker = None, None

    for line in open(mission_log, errors="ignore"):
        leg = _LEG.search(line)
        if leg:
            expected_marker = int(leg.group(2))
        start = _START.search(line)
        if start:
            if current:
                attempts.append(current)
            current = {
                "expected_marker": expected_marker,
                "t_start": _seconds(start.group(1), start.group(2)),
                "locked_pads": [],
                "t_lock": None,
                "lock_alt": None,
                "commit_alt": None,
                "commit_off": None,
                "contact": None,
                "confirmed": False,
                "t_end": None,
                "ended": None,
            }
            continue
        if current is None:
            continue

        lock = _LOCK.search(line)
        if lock:
            current["locked_pads"].append(int(lock.group(3)))
            if current["t_lock"] is None:
                current["t_lock"] = _seconds(lock.group(1), lock.group(2))
                current["lock_alt"] = float(lock.group(4))
            continue
        commit = _COMMIT.search(line)
        if commit:
            current["commit_alt"] = float(commit.group(3))
            current["commit_off"] = float(commit.group(4))
            continue
        low = _LOWCOMMIT.search(line)
        if low:
            current["commit_alt"] = float(low.group(3))
            continue
        contact = _CONTACT.search(line)
        if contact:
            current["contact"] = float(contact.group(3))
            continue
        confirm = _CONFIRM.search(line)
        if confirm:
            current["confirmed"] = True
            current["t_end"] = _seconds(confirm.group(1), confirm.group(2))
            continue
        ended = _ENDED.search(line)
        if ended:
            current["ended"] = ended.group(3)
            continue
        invalid = _INVALID.search(line)
        if invalid:
            invalid_events.append(invalid.group(0))
    if current:
        attempts.append(current)

    frames = []
    try:
        for line in open(vision_log, errors="ignore"):
            frame = _VISION.search(line)
            if frame:
                frames.append(
                    (
                        _seconds(frame.group(1), frame.group(2)),
                        float(frame.group(4)),  # range
                        float(frame.group(5)),  # offset u
                        float(frame.group(6)),  # offset v
                    )
                )
    except OSError:
        pass

    for attempt in attempts:
        t0 = attempt["t_start"]
        t1 = attempt["t_end"] or (t0 + 300.0)
        low = [f for f in frames if t0 <= f[0] <= t1 and f[1] < 3.0]
        attempt["oscillations"] = max(
            _sign_changes([f[2] for f in low]), _sign_changes([f[3] for f in low])
        )

    return {"attempts": attempts, "invalid_events": invalid_events}


def report(analysis: dict, landings: list[dict] | None = None) -> bool:
    attempts = analysis["attempts"]
    invalid = analysis["invalid_events"]

    print("\n" + "=" * 96)
    print("docs/ACCEPTANCE.md section 7 - twenty scripted landings")
    print("=" * 96)
    print(
        f"{'#':>3} {'marker':>6} {'locked':>9} {'lock_s':>7} {'lock_alt':>8} "
        f"{'commit_alt':>10} {'commit_off':>10} {'contact':>8} {'osc<3m':>7}  result"
    )
    print("-" * 96)
    for i, a in enumerate(attempts, 1):
        lock_s = "-" if a["t_lock"] is None else f"{a['t_lock'] - a['t_start']:.1f}"
        wrong = [p for p in a["locked_pads"] if p != a["expected_marker"]]
        print(
            f"{i:3d} {a['expected_marker']:6} {str(sorted(set(a['locked_pads']))):>9} "
            f"{lock_s:>7} {a['lock_alt'] if a['lock_alt'] else '-':>8} "
            f"{a['commit_alt'] if a['commit_alt'] else '-':>10} "
            f"{a['commit_off'] if a['commit_off'] is not None else '-':>10} "
            f"{a['contact'] if a['contact'] is not None else '-':>8} "
            f"{a['oscillations']:7d}  "
            f"{'TOUCHDOWN' if a['confirmed'] else (a['ended'] or 'incomplete')}"
            + ("   WRONG MARKER" if wrong else "")
        )

    n = len(attempts)
    confirmed = [a for a in attempts if a["confirmed"]]
    wrong_marker = [a for a in attempts if any(p != a["expected_marker"] for p in a["locked_pads"])]
    oscillating = [a for a in attempts if a["oscillations"] > 3]
    quick_lock = [
        a for a in attempts if a["t_lock"] is not None and a["t_lock"] - a["t_start"] <= 20.0
    ]

    scored = [row for row in (landings or []) if row.get("error_m") is not None]
    within = [row for row in scored if row["error_m"] < 0.5]

    print("-" * 96)
    print(f"attempts                            : {n}")
    print(f"touchdowns confirmed                : {len(confirmed)}/{n}")
    if scored:
        errors = sorted(row["error_m"] for row in scored)
        print(
            f"within 0.5 m of pad centre          : {len(within)}/{len(scored)}   (need 19 of 20)"
        )
        print(
            f"error (m)                           : min {errors[0]:.3f}  "
            f"median {errors[len(errors) // 2]:.3f}  "
            f"mean {sum(errors) / len(errors):.3f}  max {errors[-1]:.3f}"
        )
    print(f"landings on the WRONG marker        : {len(wrong_marker)}   (need zero)")
    print(f"oscillation events below 3 m        : {len(oscillating)}   (need zero)")
    print(f"lock within 20 s of the search alt  : {len(quick_lock)}/{n}   (need 18 of 20)")
    print(f"invalid FSM events                  : {len(invalid)}   (need zero)")
    for event in invalid[:5]:
        print(f"    {event}")

    commits = [a["commit_off"] for a in attempts if a["commit_off"] is not None]
    if commits:
        print(
            f"offset at commit (m)                : min {min(commits):.3f}  "
            f"mean {sum(commits) / len(commits):.3f}  max {max(commits):.3f}"
        )

    # WHERE THE ERROR COMES FROM.
    #
    # The vision-guided phase and the open-loop commit are two different error
    # sources, and only their sum is visible at touchdown. Pairing each confirmed
    # attempt's centring at commit against its ground-truth error separates them:
    # a small commit offset next to a large final error means the drone drifted
    # after it stopped looking, which is a question about the commit, not about
    # the controller that got it there.
    paired = [
        (a["commit_off"], row["error_m"])
        for a, row in zip(confirmed, scored, strict=False)
        if a["commit_off"] is not None and row["error_m"] is not None
    ]
    if paired:
        drifts = [final - commit for commit, final in paired]
        print(
            f"drift during the commit (m)         : min {min(drifts):+.3f}  "
            f"mean {sum(drifts) / len(drifts):+.3f}  max {max(drifts):+.3f}   "
            f"(touchdown error minus offset at commit, n={len(paired)})"
        )

    passed = (
        n >= 20
        and len(scored) >= 20
        and len(within) >= 19
        and not wrong_marker
        and not oscillating
        and not invalid
        and len(quick_lock) >= 18
    )
    print("-" * 96)
    print(f"SECTION 7: {'PASS' if passed else 'NOT PASSED (see the counts above)'}")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--missions", type=int, default=7, help="dispatches; each flies 3 landings")
    parser.add_argument(
        "--analyse-only", action="store_true", help="score existing logs, do not fly"
    )
    parser.add_argument("--results", default=os.path.join(LOG_DIR, "landing_trial.json"))
    args = parser.parse_args()

    landings = None
    if args.analyse_only:
        try:
            landings = json.load(open(args.results))["landings"]
        except Exception:  # noqa: BLE001
            landings = None
    else:
        flown = fly(args.missions)
        landings = flown["landings"]
        with open(args.results, "w") as fh:
            json.dump(flown, fh, indent=2)
        print(f"\nresults written to {args.results}")

    return 0 if report(analyse(), landings) else 1


if __name__ == "__main__":
    raise SystemExit(main())
