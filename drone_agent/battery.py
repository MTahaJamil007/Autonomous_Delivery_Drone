"""Battery gating: refuse a leg the drone cannot complete and still land.

This was dead code. `check_leg_battery` existed, was correct enough, and was
called from nowhere -- so "battery monitoring gates each leg" was documented
behaviour with no call site. It is now invoked before every leg by
DroneMission.

UNITS, ONCE, LOUDLY
-------------------
Everything here is PERCENT on a 0-100 scale, matching MAVSDK 2.x's
Battery.remaining_percent. In MAVSDK 1.x the same field was a 0-1 fraction. The
old `mavsdk>=1.4.0` requirement spanned that change, so depending on what pip
resolved, a full pack read as either 100 (fine) or 1.0 (instant abort), and a
20% threshold either worked or was never reachable. pyproject.toml now pins
>=2.0,<3; read the note there before relaxing it.
"""

from __future__ import annotations

import logging
from typing import Any

import config
from drone_agent import geo

logger = logging.getLogger(__name__)


def estimate_leg_energy_pct(
    current_lat: float, current_lon: float, target_lat: float, target_lon: float
) -> float:
    """Percent of pack needed to fly a leg and still have landing reserve.

    Linear in distance via config.BATTERY_ENERGY_PER_M_PCT. That constant is a
    PLACEHOLDER -- a conservative guess from typical multirotor figures, not a
    measurement from this airframe. It is deliberately in config with that
    caveat rather than buried here, and docs/CALIBRATION.md records the
    procedure for replacing it with real flight data.

    The estimate ignores altitude changes and wind. Both make it optimistic,
    which is why config.BATTERY_RESERVE_MARGIN_PCT sits on top of it.
    """
    distance_m = geo.get_distance_m(current_lat, current_lon, target_lat, target_lon)
    travel_pct = distance_m * config.BATTERY_ENERGY_PER_M_PCT
    return travel_pct + config.BATTERY_LANDING_RESERVE_PCT


def check_battery_sufficient(current_pct: float, required_pct: float) -> bool:
    """True if `current_pct` covers `required_pct` plus the reserve margin."""
    required_with_margin = required_pct + config.BATTERY_RESERVE_MARGIN_PCT
    sufficient = current_pct >= required_with_margin

    if not sufficient:
        logger.warning(
            "battery gate FAILED: %.1f%% available, %.1f%% needed "
            "(%.1f%% for the leg + %d%% margin)",
            current_pct, required_with_margin, required_pct,
            config.BATTERY_RESERVE_MARGIN_PCT,
        )
    else:
        logger.info(
            "battery gate passed: %.1f%% available, %.1f%% needed",
            current_pct, required_with_margin,
        )
    return sufficient


def check_leg_battery(
    drone_state: dict[str, Any],
    target_lat: float,
    target_lon: float,
    fsm: Any = None,
) -> tuple[bool, str]:
    """Gate one leg on the battery reading already in drone_state.

    Reads the telemetry the mission's own task maintains rather than opening a
    fresh `async for battery in drone.telemetry.battery()` subscription per leg
    -- which is what the previous version did, and which costs a new MAVLink
    stream each time.

    Returns:
        (allowed, reason). `reason` is empty when allowed.

    A missing or zero reading ALLOWS the leg. That is the deliberate choice:
    telemetry that has not populated yet is the overwhelmingly common cause, and
    refusing to fly on absent data would ground every mission at startup. The
    supervisor's independent critical-battery check is the backstop for a pack
    that really is empty.
    """
    battery_pct = drone_state.get("battery_pct")

    if battery_pct is None or battery_pct <= 0.0:
        logger.warning(
            "battery gate: no telemetry yet (%r) - allowing the leg. The safety "
            "supervisor's critical-battery check remains active.", battery_pct,
        )
        return True, ""

    required_pct = estimate_leg_energy_pct(
        drone_state.get("lat", 0.0), drone_state.get("lon", 0.0),
        target_lat, target_lon,
    )

    if check_battery_sufficient(battery_pct, required_pct):
        return True, ""

    reason = (
        f"Battery {battery_pct:.1f}% is below the "
        f"{required_pct + config.BATTERY_RESERVE_MARGIN_PCT:.1f}% needed for "
        f"this leg plus reserve."
    )
    if fsm is not None:
        fsm.fire("battery_low")
    return False, reason
