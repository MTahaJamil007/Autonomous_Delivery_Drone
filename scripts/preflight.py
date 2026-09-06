#!/usr/bin/env python3
"""Preflight verification. Fail fast, fail loud, name the missing thing.

WHY THIS EXISTS
---------------
Two of the seven findings behind this remediation stayed hidden for months for
one reason: nothing ever checked that the sensors existed. The drone armed,
took off, and flew with /camera/image and /lidar/scan simply absent, and the
only symptom was a mission that "didn't quite work". A landing pad that decodes
in zero ArUco dictionaries (F1) and a sensor suite living in an uncommitted
third-party submodule (F2) are both invisible at the Python layer and both
obvious to a five-second topic check.

So: nothing dispatches until every check below passes, and a failure names the
specific topic, dimension or port that is wrong rather than reporting a generic
"not ready".

WHAT IT CHECKS
--------------
  1. Gazebo advertises the model-scoped camera and LiDAR topics for each drone.
  2. The camera's live width/height match config (a model swap silently
     invalidates CAMERA_FX_PX and the whole landing control loop with it).
  3. The LiDAR's live angle_min, angle_increment and range_max match config.
  4. The landing pad models exist and their ArUco textures actually decode.
  5. MAVSDK connects on 14540+i.
  6. The obstacle memory service answers on its port.
  7. The fleet dispatcher answers on its port and its database is queryable.

Usage:
    python3 scripts/preflight.py                    # every drone in the fleet
    python3 scripts/preflight.py --drone-id drone-0 # one drone
    python3 scripts/preflight.py --skip mavsdk      # bench checks only

Exit status is 0 only if every selected check passed.
"""

# ruff: noqa: E402 - the sys.path bootstrap below must run before the
# project imports, so that this script works before `pip install -e .`.
from __future__ import annotations

import argparse
import asyncio
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config
import sim_topics

GZ_TIMEOUT_S = 6.0

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"


@dataclass
class Result:
    """One check's outcome. `detail` must name the offending thing on failure."""

    name: str
    status: str
    detail: str = ""


@dataclass
class Report:
    results: list[Result] = field(default_factory=list)

    def add(self, name: str, status: str, detail: str = "") -> None:
        self.results.append(Result(name, status, detail))
        glyph = {PASS: "  ok  ", FAIL: " FAIL ", SKIP: " skip "}[status]
        line = f"[{glyph}] {name}"
        if detail:
            line += f"\n           {detail}"
        print(line, flush=True)

    @property
    def failed(self) -> list[Result]:
        return [r for r in self.results if r.status == FAIL]

    @property
    def ok(self) -> bool:
        return not self.failed


# ─────────────────────────────────────────────────────────────────────────────
#  Gazebo helpers
# ─────────────────────────────────────────────────────────────────────────────


def _gz_available() -> bool:
    return shutil.which("gz") is not None


def _gz_topic_list() -> list[str]:
    """Every topic Gazebo currently advertises."""
    try:
        out = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=GZ_TIMEOUT_S,
            check=False,
        )
    except (subprocess.TimeoutExpired, OSError):
        return []
    return [line.strip() for line in out.stdout.splitlines() if line.strip()]


def _gz_echo_one(topic: str, timeout_s: float = GZ_TIMEOUT_S) -> str:
    """Text form of a single message on `topic`, or '' if none arrived.

    `gz topic -e -n 1` exits on its own after one message, but hangs forever if
    the publisher is silent, so the timeout is the real terminator.
    """
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        return out.stdout
    except subprocess.TimeoutExpired as exc:
        # Partial output is still useful: the fields we want come early.
        stdout = exc.stdout or b""
        return stdout.decode(errors="replace") if isinstance(stdout, bytes) else str(stdout)
    except OSError:
        return ""


def _scalar(text: str, field_name: str) -> float | None:
    """First `field_name: <number>` in a Gazebo text-format message."""
    match = re.search(rf"^\s*{re.escape(field_name)}:\s*(-?[\d.eE+]+)", text, re.MULTILINE)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


# ─────────────────────────────────────────────────────────────────────────────
#  Checks
# ─────────────────────────────────────────────────────────────────────────────


def check_topics_advertised(report: Report, drone_ids: list[str]) -> dict[str, dict[str, str]]:
    """Verify the model-scoped topics exist. Returns the ones found, per drone."""
    if not _gz_available():
        for drone_id in drone_ids:
            report.add(f"{drone_id}: gz topics", SKIP, "`gz` not on PATH")
        return {}

    advertised = set(_gz_topic_list())
    if not advertised:
        report.add(
            "gazebo reachable",
            FAIL,
            "`gz topic -l` returned nothing. Is the Gazebo server running? "
            "Start it with world/spawn_fleet.sh (see RUN_GUIDE.md).",
        )
        return {}
    report.add("gazebo reachable", PASS, f"{len(advertised)} topics advertised")

    found: dict[str, dict[str, str]] = {}
    for drone_id in drone_ids:
        topics = sim_topics.all_topics(drone_id)
        present = {}
        for kind, topic in topics.items():
            if topic in advertised:
                present[kind] = topic
                report.add(f"{drone_id}: {kind} topic", PASS, topic)
            else:
                hint = (
                    f"{topic}\n           NOT advertised. Either the drone has not "
                    f"spawned, or it spawned from a model without the sensor. "
                    f"Confirm PX4_SIM_MODEL={config.GZ_MODEL_BASE} and that "
                    f"GZ_SIM_RESOURCE_PATH includes {PROJECT_ROOT}/sim/models "
                    f"(source sim/env.sh)."
                )
                report.add(f"{drone_id}: {kind} topic", FAIL, hint)
        found[drone_id] = present
    return found


def check_camera_geometry(report: Report, drone_id: str, topic: str) -> None:
    """The live image must match the intrinsics landing control is derived from."""
    text = _gz_echo_one(topic)
    if not text:
        report.add(
            f"{drone_id}: camera geometry",
            FAIL,
            f"{topic} is advertised but published no message within "
            f"{GZ_TIMEOUT_S:.0f}s. The sensor exists but is not producing frames.",
        )
        return

    width = _scalar(text, "width")
    height = _scalar(text, "height")
    if width is None or height is None:
        report.add(
            f"{drone_id}: camera geometry", FAIL, f"could not parse width/height from {topic}"
        )
        return

    if int(width) != config.CAMERA_WIDTH_PX or int(height) != config.CAMERA_HEIGHT_PX:
        report.add(
            f"{drone_id}: camera geometry",
            FAIL,
            f"live {int(width)}x{int(height)} != config "
            f"{config.CAMERA_WIDTH_PX}x{config.CAMERA_HEIGHT_PX}. "
            f"CAMERA_FX_PX ({config.CAMERA_FX_PX:.1f}) is derived from the "
            f"config width, so landing would convert pixels to metres with the "
            f"wrong scale. Fix sim/models/{config.GZ_MODEL_BASE}/model.sdf or config.py.",
        )
        return

    report.add(
        f"{drone_id}: camera geometry",
        PASS,
        f"{int(width)}x{int(height)}, fx={config.CAMERA_FX_PX:.1f} px, "
        f"{config.PAD_SIZE_M} m pad spans "
        f"{config.marker_px_at_altitude(config.SEARCH_ALT_M):.0f} px at "
        f"SEARCH_ALT_M={config.SEARCH_ALT_M} m",
    )


def check_lidar_geometry(report: Report, drone_id: str, topic: str) -> None:
    """The live scan's geometry must match what the avoider's sectors assume."""
    text = _gz_echo_one(topic)
    if not text:
        report.add(
            f"{drone_id}: lidar geometry",
            FAIL,
            f"{topic} is advertised but published no message within "
            f"{GZ_TIMEOUT_S:.0f}s. The sensor exists but is not producing scans.",
        )
        return

    angle_min = _scalar(text, "angle_min")
    angle_step = _scalar(text, "angle_step")
    range_max = _scalar(text, "range_max")
    count = _scalar(text, "count")

    problems = []
    if angle_min is None:
        problems.append("angle_min missing")
    elif not math.isclose(angle_min, config.LIDAR_ANGLE_MIN_RAD, abs_tol=1e-3):
        problems.append(f"angle_min {angle_min:.5f} != config {config.LIDAR_ANGLE_MIN_RAD:.5f}")

    expected_step = (config.LIDAR_ANGLE_MAX_RAD - config.LIDAR_ANGLE_MIN_RAD) / config.LIDAR_SAMPLES
    if angle_step is not None and not math.isclose(angle_step, expected_step, rel_tol=2e-2):
        problems.append(
            f"angle_step {angle_step:.5f} != expected {expected_step:.5f} "
            f"({config.LIDAR_SAMPLES} samples over the full circle)"
        )

    if range_max is None:
        problems.append("range_max missing")
    elif not math.isclose(range_max, config.LIDAR_RANGE_MAX_M, rel_tol=1e-2):
        problems.append(f"range_max {range_max:.2f} != config {config.LIDAR_RANGE_MAX_M:.2f}")

    if count is not None and int(count) != config.LIDAR_SAMPLES:
        problems.append(f"count {int(count)} != config {config.LIDAR_SAMPLES}")

    if problems:
        report.add(
            f"{drone_id}: lidar geometry",
            FAIL,
            "; ".join(problems)
            + ".\n           The avoider maps degrees to array indices using these "
            "values; a mismatch aims the front cone somewhere other than the front.",
        )
        return

    report.add(
        f"{drone_id}: lidar geometry",
        PASS,
        f"{config.LIDAR_SAMPLES} samples, angle_min={angle_min:.4f} rad, "
        f"range_max={range_max:.1f} m",
    )


def check_pad_models(report: Report) -> None:
    """Every pad must decode EVERY marker its layout declares.

    Two findings live in this check.

    F1 was a texture that decoded in zero dictionaries, so the drone could never
    lock on at all.

    The one that produced the reported hover is subtler and is why this check
    now looks at the whole layout rather than at one id: a pad whose only marker
    is large decodes perfectly on the bench and becomes invisible below 2.5 m in
    flight, because the marker outgrows the frame. A texture that is missing its
    small centre marker would pass the old version of this check and strand the
    drone at the same altitude all over again.
    """
    try:
        import cv2
    except ImportError:
        report.add("landing pads decode", SKIP, "OpenCV not importable")
        return

    import world.marker_models as marker_models
    import world.pad_layout as pad_layout

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, pad_layout.ARUCO_DICTIONARY)),
        cv2.aruco.DetectorParameters(),
    )

    problems = []
    checked = []
    for role, uri in marker_models.PAD_MODELS.items():
        model = uri.removeprefix("model://")
        model_dir = PROJECT_ROOT / "sim" / "models" / model
        pad_index = marker_models.ROLE_TO_ID[role]
        expected = {m.marker_id for m in pad_layout.pad_markers(pad_index)}

        if not (model_dir / "model.sdf").exists():
            problems.append(f"{role}: {model_dir} missing")
            continue

        stale = sorted(model_dir.glob("aruco_*.png"))
        if stale:
            problems.append(
                f"{role}: the old single-marker texture {stale[0].name} is still "
                f"present; run `python3 -m world.build_pads`"
            )

        texture = model_dir / f"{model}.png"
        if not texture.exists():
            problems.append(f"{role}: {texture.name} missing; run `python3 -m world.build_pads`")
            continue

        image = cv2.imread(str(texture), cv2.IMREAD_GRAYSCALE)
        if image is None:
            problems.append(f"{role}: {texture.name} unreadable")
            continue

        _corners, ids, _rejected = detector.detectMarkers(image)
        found = set(ids.flatten().tolist()) if ids is not None else set()

        if not found:
            problems.append(
                f"{role}: {texture.name} does NOT decode as "
                f"{pad_layout.ARUCO_DICTIONARY} (no white quiet zone? finding F1)"
            )
        elif not found >= expected:
            problems.append(
                f"{role}: {texture.name} decodes {sorted(found)} but the layout "
                f"declares {sorted(expected)}; missing {sorted(expected - found)}. "
                f"Run `python3 -m world.build_pads`."
            )
        else:
            checked.append(f"{model}={sorted(expected)}")

    if problems:
        report.add("landing pads decode", FAIL, "; ".join(problems))
    else:
        report.add("landing pads decode", PASS, ", ".join(checked))


def check_landing_altitude_budget(report: Report) -> None:
    """The pad must be visible over every altitude the landing loop flies through.

    THIS IS THE CHECK WHOSE ABSENCE COST A DELIVERY. The controller descends
    from SEARCH_ALT_M to the ground; if the pad stops being measurable partway,
    the descent goes blind exactly where precision matters, and the symptom in
    the air is a drone that hovers over its pad and never lands.

    Every number here is derived from the shipped geometry, so changing a marker
    size or the camera without re-deriving the controller's altitudes fails on
    the bench.
    """
    try:
        import world.pad_layout as pad_layout
    except Exception as exc:  # noqa: BLE001
        report.add("landing altitude budget", SKIP, f"pad_layout unimportable: {exc}")
        return

    tan_h = math.tan(config.CAMERA_HFOV_RAD / 2.0)
    tan_v = (config.CAMERA_HEIGHT_PX / 2.0) / config.CAMERA_FX_PX

    try:
        low_m, high_m = pad_layout.coverage_m(config.CAMERA_FX_PX, tan_h, tan_v)
    except ValueError as exc:
        report.add("landing altitude budget", FAIL, str(exc))
        return

    commit_floor_m = pad_layout.visibility_floor_m(
        config.CAMERA_FX_PX, tan_h, tan_v, offset_m=config.COMMIT_MAX_OFFSET_M
    )

    problems = []
    if config.SEARCH_ALT_M > high_m:
        problems.append(
            f"SEARCH_ALT_M {config.SEARCH_ALT_M} m is above the {high_m:.2f} m "
            f"ceiling at which markers stop being decodable -- the search would "
            f"fly where the pad cannot be seen"
        )
    if config.COMMIT_ALT_M < low_m:
        problems.append(
            f"COMMIT_ALT_M {config.COMMIT_ALT_M} m is below the {low_m:.2f} m "
            f"vision floor -- the drone goes blind before it may commit"
        )
    if config.COMMIT_ALT_M <= commit_floor_m:
        problems.append(
            f"COMMIT_ALT_M {config.COMMIT_ALT_M} m leaves no margin over the "
            f"{commit_floor_m:.2f} m floor for a pad {config.COMMIT_MAX_OFFSET_M} m "
            f"off axis -- the commit gate would be unsatisfiable"
        )
    if config.NO_CLIMB_ALT_M < low_m:
        problems.append(
            f"NO_CLIMB_ALT_M {config.NO_CLIMB_ALT_M} m is below the {low_m:.2f} m "
            f"vision floor -- the drone could still climb when it loses the pad, "
            f"which is the limit cycle this design exists to prevent"
        )

    if problems:
        report.add("landing altitude budget", FAIL, "; ".join(problems))
    else:
        report.add(
            "landing altitude budget",
            PASS,
            f"pad visible {low_m:.2f}-{high_m:.2f} m; search {config.SEARCH_ALT_M} m, "
            f"commit {config.COMMIT_ALT_M} m, no-climb {config.NO_CLIMB_ALT_M} m",
        )


async def check_mavsdk(report: Report, drone_id: str, timeout_s: float) -> None:
    """A drone we cannot reach is a mission that would fail after takeoff."""
    try:
        from mavsdk import System
    except ImportError:
        report.add(f"{drone_id}: mavsdk", SKIP, "mavsdk not importable")
        return

    url = config.mavsdk_url(drone_id)
    drone = System()
    try:
        await drone.connect(system_address=url)

        async def _wait_connected() -> None:
            async for state in drone.core.connection_state():
                if state.is_connected:
                    return

        await asyncio.wait_for(_wait_connected(), timeout=timeout_s)
    except asyncio.TimeoutError:
        report.add(
            f"{drone_id}: mavsdk",
            FAIL,
            f"no MAVLink heartbeat on {url} within {timeout_s:.0f}s. "
            f"PX4 SITL instance {config.drone_index(drone_id)} is not running "
            f"(it publishes to 14540+instance).",
        )
        return
    except Exception as exc:  # noqa: BLE001 - report any transport failure verbatim
        report.add(f"{drone_id}: mavsdk", FAIL, f"{url}: {type(exc).__name__}: {exc}")
        return

    report.add(f"{drone_id}: mavsdk", PASS, url)


async def check_obstacle_service(report: Report, timeout_s: float = 5.0) -> None:
    """The obstacle memory service backs fleet-wide sharing; note if it is down."""
    try:
        import aiohttp
    except ImportError:
        report.add("obstacle memory service", SKIP, "aiohttp not importable")
        return

    url = f"{config.OBSTACLE_MEMORY_URL}/"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url, timeout=aiohttp.ClientTimeout(total=timeout_s)) as response:
                if response.status != 200:
                    report.add(
                        "obstacle memory service", FAIL, f"{url} returned HTTP {response.status}"
                    )
                    return
                body = await response.json()
    except Exception as exc:  # noqa: BLE001
        report.add(
            "obstacle memory service",
            FAIL,
            f"{url} unreachable ({type(exc).__name__}). Start it with "
            f"`python3 -m obstacle_memory_service.app` -- without it, obstacle "
            f"prefetch returns nothing and detour planning has no memory to use.",
        )
        return

    report.add("obstacle memory service", PASS, f"{url} -> {body.get('service', 'ok')}")


async def check_dispatch_service(report: Report, timeout_s: float = 5.0) -> None:
    """The dispatcher serves the UI and every job endpoint; nothing works without it.

    This check exists because it was the one service preflight never looked at.
    A stale fleet.db crashed fleet_dispatch during startup lifespan, and
    preflight still reported "8 checks, 0 failures" while run_system.sh printed
    "System ready. Open http://localhost:5000" at a port with nothing behind it.
    The only symptom was the browser failing to connect.

    /fleet/status rather than / on purpose: it answers out of the database, so a
    pass means the schema is queryable too, not merely that a socket is open.
    """
    try:
        import aiohttp
    except ImportError:
        report.add("fleet dispatch service", SKIP, "aiohttp not importable")
        return

    url = f"{config.FLEET_DISPATCH_URL}/fleet/status"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url, timeout=aiohttp.ClientTimeout(total=timeout_s)) as response:
                if response.status != 200:
                    report.add(
                        "fleet dispatch service", FAIL, f"{url} returned HTTP {response.status}"
                    )
                    return
                body = await response.json()
    except Exception as exc:  # noqa: BLE001
        report.add(
            "fleet dispatch service",
            FAIL,
            f"{url} unreachable ({type(exc).__name__}). It should be running on "
            f"port {config.FLEET_DISPATCH_PORT}; if run_system.sh started it, it "
            f"has since exited -- read $LOG_DIR/fleet_dispatch.log for the "
            f"traceback. Nothing can be dispatched and the web UI will not load.",
        )
        return

    fleet_size = body.get("fleet_size", 0)
    if fleet_size != len(config.DRONE_IDS):
        report.add(
            "fleet dispatch service",
            FAIL,
            f"{url} reports {fleet_size} drone(s), config.DRONE_IDS has "
            f"{len(config.DRONE_IDS)}. The dispatcher is running against a "
            f"database registered for a different fleet size.",
        )
        return

    report.add("fleet dispatch service", PASS, f"{url} -> {fleet_size} drone(s) registered")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────


async def run(drone_ids: list[str], skip: set[str], mavsdk_timeout_s: float) -> Report:
    report = Report()

    print(
        f"Preflight: {len(drone_ids)} drone(s), world '{config.GZ_WORLD}', "
        f"model '{config.GZ_MODEL_BASE}'\n"
    )

    if "pads" not in skip:
        check_pad_models(report)
        check_landing_altitude_budget(report)

    found: dict[str, dict[str, str]] = {}
    if "topics" not in skip:
        found = check_topics_advertised(report, drone_ids)

    if "geometry" not in skip:
        for drone_id, topics in found.items():
            if "camera" in topics:
                check_camera_geometry(report, drone_id, topics["camera"])
            if "lidar" in topics:
                check_lidar_geometry(report, drone_id, topics["lidar"])

    if "mavsdk" not in skip:
        for drone_id in drone_ids:
            await check_mavsdk(report, drone_id, mavsdk_timeout_s)

    if "obstacles" not in skip:
        await check_obstacle_service(report)

    if "dispatch" not in skip:
        await check_dispatch_service(report)

    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Verify the simulator and services before dispatching.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Exit 0 only if every selected check passes.",
    )
    parser.add_argument(
        "--drone-id",
        action="append",
        dest="drone_ids",
        metavar="ID",
        help="Check only this drone (repeatable). Default: config.DRONE_IDS.",
    )
    parser.add_argument(
        "--skip",
        action="append",
        default=[],
        choices=["topics", "geometry", "mavsdk", "obstacles", "pads", "dispatch"],
        help="Skip a check group (repeatable).",
    )
    parser.add_argument(
        "--mavsdk-timeout",
        type=float,
        default=10.0,
        metavar="S",
        help="Seconds to wait for a MAVLink heartbeat (default: 10).",
    )
    args = parser.parse_args(argv)

    drone_ids = args.drone_ids or config.DRONE_IDS
    report = asyncio.run(run(drone_ids, set(args.skip), args.mavsdk_timeout))

    print()
    if report.ok:
        print(f"PREFLIGHT PASS - {len(report.results)} checks, 0 failures")
        return 0

    print(f"PREFLIGHT FAIL - {len(report.failed)} of {len(report.results)} checks failed:")
    for result in report.failed:
        print(f"  - {result.name}")
    print("\nRefusing to declare the system flight-ready. See RUN_GUIDE.md.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
