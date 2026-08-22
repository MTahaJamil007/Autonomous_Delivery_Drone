#!/usr/bin/env python3
"""Repository consistency gates. Each one encodes a regression this repo had.

Run individually or all at once:

    python3 scripts/repo_checks.py                 # everything
    python3 scripts/repo_checks.py docs schema     # a subset

Exit status is 0 only if every selected check passes. A failure names the
specific file and line, because a gate that says "the build broke" is a gate
people learn to ignore.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

SKIP_DIRS = {"vision_env", ".git", "__pycache__", ".pytest_cache", ".ruff_cache", "build", "dist"}

SOURCE_SUFFIXES = {".py", ".sh", ".md", ".html", ".sdf", ".toml", ".ini", ".sql"}


def source_files() -> list[Path]:
    files = []
    for path in PROJECT_ROOT.rglob("*"):
        if not path.is_file() or path.suffix not in SOURCE_SUFFIXES:
            continue
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        files.append(path)
    return sorted(files)


# ─────────────────────────────────────────────────────────────────────────────
#  docs
# ─────────────────────────────────────────────────────────────────────────────

# A documentation reference, not an attribute access. `hashlib.md5(...)` and
# `re.match(...)` must not match, which a naive [A-Za-z_]+\.md does.
DOC_REFERENCE = re.compile(r"\b([A-Za-z0-9_][A-Za-z0-9_-]*\.md)(?![A-Za-z0-9_])")


# Historical documents describe a past state, so they legitimately name files
# that have since been deleted. Rewriting them to satisfy this check would
# destroy the record of what was true when they were written.
DOC_CHECK_EXEMPT = {
    "docs/REMEDIATION_PLAN.md",
    "CHANGELOG.md",
    "sim/patches/README.md",
}


def check_docs() -> list[str]:
    """Every referenced .md file must exist.

    Seven references pointed at a HOW_TO_RUN guide that never existed - and two
    of them were inside runtime error strings, so an operator hit them
    mid-incident and went looking for a file that was not there.
    """
    existing = {
        p.name for p in PROJECT_ROOT.rglob("*.md") if not any(part in SKIP_DIRS for part in p.parts)
    }
    problems = []

    for path in source_files():
        if path.relative_to(PROJECT_ROOT).as_posix() in DOC_CHECK_EXEMPT:
            continue
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        for line_number, line in enumerate(text.splitlines(), 1):
            for match in DOC_REFERENCE.finditer(line):
                name = match.group(1)
                if name in existing:
                    continue
                problems.append(
                    f"{path.relative_to(PROJECT_ROOT)}:{line_number} references "
                    f"{name}, which does not exist"
                )
    return problems


# ─────────────────────────────────────────────────────────────────────────────
#  schema
# ─────────────────────────────────────────────────────────────────────────────


def check_schema() -> list[str]:
    """Simulation assets must agree with config.py.

    The landing control law is derived from the camera intrinsics, and the
    acquisition-altitude budget from the pad size. If the SDF and config drift
    apart, every command is silently rescaled and nothing reports an error.
    """
    import config

    problems = []

    model_path = PROJECT_ROOT / "sim" / "models" / "x500_delivery" / "model.sdf"
    model = model_path.read_text()

    for tag, want in (
        ("width", config.CAMERA_WIDTH_PX),
        ("height", config.CAMERA_HEIGHT_PX),
        ("horizontal_fov", config.CAMERA_HFOV_RAD),
        ("samples", config.LIDAR_SAMPLES),
        ("max", config.LIDAR_RANGE_MAX_M),
    ):
        if f"<{tag}>{want}</{tag}>" not in model:
            problems.append(f"x500_delivery/model.sdf: <{tag}> does not match config ({want})")

    # Strip comments: the file discusses <topic> at length explaining its absence.
    without_comments = re.sub(r"(?s)<!--.*?-->", "", model)
    if "<topic>" in without_comments:
        problems.append(
            "x500_delivery/model.sdf declares <topic>, which re-introduces "
            "finding F7 (N drones sharing two topics)"
        )

    for index in (0, 1, 2):
        pad_path = PROJECT_ROOT / "sim" / "models" / f"pad_{index}" / "model.sdf"
        if not pad_path.exists():
            problems.append(f"pad_{index}/model.sdf is missing")
            continue
        pad = pad_path.read_text()
        expected = f"<size>{config.PAD_SIZE_M} {config.PAD_SIZE_M}</size>"
        if expected not in pad:
            problems.append(
                f"pad_{index}/model.sdf: size does not match "
                f"config.PAD_SIZE_M ({config.PAD_SIZE_M})"
            )

    # The world origin must match the map centre, or the browser draws the drone
    # in the wrong country while every internal number stays consistent.
    world = (PROJECT_ROOT / "sim" / "worlds" / "delivery.sdf").read_text()
    template = (PROJECT_ROOT / "fleet_dispatch" / "templates" / "index.html").read_text()

    world_lat = re.search(r"<latitude_deg>([\d.]+)</latitude_deg>", world)
    map_origin = re.search(r"ORIGIN\s*=\s*\[([\d.]+),", template)
    if world_lat and map_origin:
        if abs(float(world_lat.group(1)) - float(map_origin.group(1))) > 0.001:
            problems.append(
                f"world origin latitude {world_lat.group(1)} does not match the "
                f"Leaflet map centre {map_origin.group(1)}"
            )

    # And the derived budget must still clear the decode threshold.
    at_search = config.marker_px_at_altitude(config.SEARCH_ALT_M)
    if at_search < config.MARKER_MIN_DECODE_PX:
        problems.append(
            f"a {config.PAD_SIZE_M} m pad spans only {at_search:.0f} px at "
            f"SEARCH_ALT_M={config.SEARCH_ALT_M} m, below the "
            f"{config.MARKER_MIN_DECODE_PX:.0f} px decode threshold (finding F3)"
        )

    return problems


# ─────────────────────────────────────────────────────────────────────────────
#  tunables
# ─────────────────────────────────────────────────────────────────────────────

SHADOWABLE = [
    "TARGET_ALT",
    "CRUISE_SPEED",
    "SLOW_RADIUS_M",
    "MIN_SPEED",
    "ARRIVAL_M",
    "DODGE_SPEED",
    "BACK_SPEED",
    "ALT_GAIN",
    "ALT_MAX_VEL",
    "NAV_HZ",
    "SAFE_DIST",
    "CLEAR_DIST",
    "CLEAR_CONFIRM_S",
    "MIN_LOCK_S",
    "ESCALATION_LOCK_S",
    "LANDING_TIMEOUT_S",
    "CENTER_THRESH_PX",
]


def check_tunables() -> list[str]:
    """config.py must be the only place a tunable is defined.

    drone_logic.py re-declared ten navigation constants at module scope and
    avoider_node.py five as class constants, so editing config.py had no effect
    on what actually flew. That made the file which claims to centralise tuning
    the most misleading file in the repository.
    """
    problems = []
    pattern = re.compile(r"^\s*(" + "|".join(SHADOWABLE) + r")[A-Z_]*\s*=(?!=)")

    for path in source_files():
        if path.suffix != ".py":
            continue
        relative = path.relative_to(PROJECT_ROOT)
        # config.py is the definition site; tests legitimately bind locals from it.
        if relative.as_posix() == "config.py" or relative.parts[0] == "tests":
            continue
        for line_number, line in enumerate(path.read_text().splitlines(), 1):
            if pattern.match(line):
                problems.append(
                    f"{relative}:{line_number} re-declares a config tunable: {line.strip()}"
                )
    return problems


# ─────────────────────────────────────────────────────────────────────────────
#  blocking calls
# ─────────────────────────────────────────────────────────────────────────────

BLOCKING_PATTERNS = [
    (
        re.compile(r"\bsubprocess\.(run|call|check_output|check_call)\s*\("),
        "synchronous subprocess: use asyncio.create_subprocess_exec "
        "(drone_agent/gz_client.py). Finding F5: a blocking call in an async tick "
        "freezes the setpoint publisher and PX4 drops OFFBOARD.",
    ),
    (
        re.compile(r"\bcv2\.imshow\s*\("),
        "cv2.imshow blocks its calling thread; keep it behind an opt-in flag",
    ),
    (
        re.compile(r"^\s*time\.sleep\s*\(", re.MULTILINE),
        "time.sleep in flight code blocks the event loop; use asyncio.sleep",
    ),
]

# Files where a blocking call is correct: bench tools and operator utilities
# that never run an event loop.
BLOCKING_ALLOWED = {
    "scripts/preflight.py",
    "scripts/repo_checks.py",
    "reset_fleet_database.py",
    "test_connection_now.py",
}

# A per-line opt-out, so an intentional blocking call is annotated at the call
# site with its justification rather than exempting a whole file. Whole-file
# exemptions are how the next genuine blocking call gets in unnoticed.
BLOCKING_OK = re.compile(r"#\s*blocking-ok:")


def check_blocking() -> list[str]:
    """No blocking calls in the flight path (finding F5)."""
    problems = []
    flight_dirs = ("drone_agent", "perception", "drone_web", "fleet_dispatch")

    for path in source_files():
        if path.suffix != ".py":
            continue
        relative = path.relative_to(PROJECT_ROOT).as_posix()
        if relative in BLOCKING_ALLOWED:
            continue
        if not (relative.startswith(flight_dirs) or relative == "avoider_node.py"):
            continue

        raw_lines = path.read_text().splitlines()

        # Blank out docstrings before scanning: these files DISCUSS the old
        # blocking calls at length, and flagging the explanation would be
        # absurd. Line numbers are preserved so a real hit still points
        # somewhere useful.
        text = path.read_text()
        blanked = list(raw_lines)
        for match in re.finditer(r'(?s)"""(.*?)"""', text):
            start = text[: match.start()].count("\n")
            end = text[: match.end()].count("\n")
            for index in range(start, min(end + 1, len(blanked))):
                blanked[index] = ""

        for line_number, line in enumerate(blanked, 1):
            code = line.split("#", 1)[0]
            if not code.strip():
                continue
            for pattern, message in BLOCKING_PATTERNS:
                if pattern.search(code):
                    # The marker may sit on the call line itself or in the
                    # comment block immediately above it, which is where a
                    # multi-line justification naturally goes.
                    window = raw_lines[max(0, line_number - 4) : line_number]
                    if any(BLOCKING_OK.search(candidate) for candidate in window):
                        continue  # annotated with a justification
                    problems.append(f"{relative}:{line_number} {message}")
    return problems


# ─────────────────────────────────────────────────────────────────────────────
#  entry point
# ─────────────────────────────────────────────────────────────────────────────

CHECKS = {
    "docs": ("every referenced .md exists", check_docs),
    "schema": ("sim assets agree with config.py", check_schema),
    "tunables": ("config.py is the only source of tunables", check_tunables),
    "blocking": ("no blocking calls in the flight path", check_blocking),
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "checks",
        nargs="*",
        metavar="CHECK",
        help=f"Checks to run: {', '.join(sorted(CHECKS))}. Default: all.",
    )
    args = parser.parse_args(argv)

    unknown = [name for name in args.checks if name not in CHECKS]
    if unknown:
        parser.error(
            f"unknown check(s): {', '.join(unknown)}. Available: {', '.join(sorted(CHECKS))}"
        )

    selected = args.checks or sorted(CHECKS)
    failed = 0

    for name in selected:
        description, function = CHECKS[name]
        problems = function()
        if problems:
            failed += 1
            print(f"FAIL  {name}: {description}")
            for problem in problems:
                print(f"        {problem}")
        else:
            print(f"ok    {name}: {description}")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
