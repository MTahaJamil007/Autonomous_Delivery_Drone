"""The avoider must survive every action transition it can log.

THE FAILURE THIS PINS DOWN (2026-10-13 flight)
---------------------------------------------
The avoider logged transitions through one call site whose severity varied:
WARN for CLEAR -> DODGE_LEFT, INFO for DODGE_LEFT -> CLEAR. rclpy keys each
logging call site on (function, file, line, bytecode offset) and raises
`ValueError: Logger severity cannot be changed between calls` when a site is
reused at a different severity. So the avoider crashed the instant the drone
cleared its first obstacle, the obstacle feed went silent, and navigation held
position until the 600 s leg timeout. In the simulator: a perfect dodge, then
ten minutes of motionless hover.

tests/test_avoider_node.py only covers decide_action(), which was never wrong.
These tests drive log_transition() -- the code that actually crashed -- through
every transition, against a logger that enforces rclpy's rule, and against
rclpy's real logger when ROS is importable.
"""

from __future__ import annotations

import inspect
import itertools
import json
import math

import pytest

from avoider_node import (
    CLEAR,
    DODGE_LEFT,
    DODGE_RIGHT,
    ESCALATE,
    log_transition,
    render_scan_view,
)

ACTIONS = [CLEAR, DODGE_LEFT, DODGE_RIGHT, ESCALATE]


class CallSiteStrictLogger:
    """Mimics rclpy.impl.rcutils_logger: one severity per call site, forever."""

    def __init__(self) -> None:
        self.sites: dict[tuple, str] = {}
        self.records: list[tuple[str, str]] = []

    def _log(self, severity: str, message: str) -> None:
        caller = inspect.currentframe().f_back.f_back
        site = (caller.f_code.co_name, caller.f_code.co_filename, caller.f_lineno, caller.f_lasti)
        if self.sites.setdefault(site, severity) != severity:
            raise ValueError("Logger severity cannot be changed between calls.")
        self.records.append((severity, message))

    def info(self, message: str) -> None:
        self._log("info", message)

    def warning(self, message: str) -> None:
        self._log("warning", message)

    def error(self, message: str) -> None:
        self._log("error", message)


def _every_transition():
    return [(a, b) for a, b in itertools.product(ACTIONS, ACTIONS) if a != b]


def test_the_flight_sequence_that_crashed_the_avoider():
    """CLEAR -> DODGE_LEFT -> CLEAR, exactly as flown on 2026-10-13."""
    log = CallSiteStrictLogger()
    log_transition(log, CLEAR, DODGE_LEFT, 6.4, 15.0, 15.0, 0.0)
    log_transition(log, DODGE_LEFT, CLEAR, 9.5, 15.0, 15.0, 7.8)
    assert [severity for severity, _ in log.records] == ["warning", "info"]


def test_every_transition_twice_on_one_logger():
    """Any order of any transitions, repeated, never trips the rule."""
    log = CallSiteStrictLogger()
    for _ in range(2):
        for previous, action in _every_transition():
            log_transition(log, previous, action, 5.0, 10.0, 8.0, 1.0)
    assert len(log.records) == 2 * len(_every_transition())


@pytest.mark.parametrize(
    ("action", "severity"),
    [(CLEAR, "info"), (DODGE_LEFT, "warning"), (DODGE_RIGHT, "warning"), (ESCALATE, "error")],
)
def test_severity_matches_seriousness(action, severity):
    log = CallSiteStrictLogger()
    log_transition(log, "X", action, 5.0, 10.0, 8.0, 1.0)
    assert log.records[0][0] == severity
    assert f"X -> {action}" in log.records[0][1]


def test_against_rclpys_real_logger():
    """The same sequence through rclpy itself, where ROS is installed."""
    rclpy_logging = pytest.importorskip("rclpy.logging")
    log = rclpy_logging.get_logger("test_avoider_logging")
    for previous, action in _every_transition() * 2:
        log_transition(log, previous, action, 5.0, 10.0, 8.0, 1.0)


# ── the dashboard's LiDAR view ──────────────────────────────────────────────


def _scan(ranges):
    return {
        "drone_id": "drone-0",
        "seq": 7,
        "action": DODGE_LEFT,
        "eff_front_m": 4.2,
        "med_left_m": 15.0,
        "med_right_m": 9.1,
        "dodge_age_s": 1.5,
        "ranges": ranges,
        "angle_min": -math.pi,
        "angle_increment": 2 * math.pi / len(ranges),
        "range_max": 12.0,
        "transitions": [{"wall": 1.0, "from": CLEAR, "to": DODGE_LEFT, "front_m": 4.2}],
    }


def test_scan_view_is_strict_json_on_an_empty_sky():
    """inf/nan must become null: `Infinity` is not JSON and kills the browser view."""
    view = render_scan_view(_scan([math.inf, math.nan, 0.0, 3.14159, -1.0] * 72), 0.05)
    text = json.dumps(view, allow_nan=False)  # raises on inf/nan
    assert view["ranges"][:5] == [None, None, None, 3.14, None]
    assert "Infinity" not in text and "NaN" not in text


def test_scan_view_carries_the_thresholds_the_view_draws():
    import config

    view = render_scan_view(_scan([5.0] * 360), 0.0)
    assert view["limits"]["safe_m"] == config.SAFE_DIST
    assert view["limits"]["clear_m"] == config.CLEAR_DIST
    assert view["limits"]["front_half_deg"] == config.FRONT_HALF_DEG
    assert view["action"] == DODGE_LEFT
    assert len(view["ranges"]) == 360
