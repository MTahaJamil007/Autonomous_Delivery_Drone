"""Independent safety monitor with direct autopilot authority.

Runs as its own asyncio task for the mission's lifetime and bypasses the mission
FSM entirely: when it decides to act, it commands the autopilot directly. A
safety layer that has to ask the thing it is supervising for permission is not a
safety layer.

THREE DEFECTS FIXED (P3.5), EACH OF WHICH MADE IT DANGEROUS RATHER THAN INERT
-----------------------------------------------------------------------------
1. THE RTL FALLBACK WAS UNREACHABLE. `_trigger_emergency_rtl` set
   `_emergency_triggered = True` and then, if the RTL command failed, called
   `_trigger_emergency_land` -- which began with `if self._emergency_triggered:
   return`. So an RTL failure produced NO ACTION AT ALL, silently, at the exact
   moment the drone most needed one. Fixed by latching the flag only after a
   command is accepted, and by giving the fallback an explicit override.

2. THE HEARTBEAT CHECK FIRED AT t=0. It computed `time.time() - last_telemetry_ts`
   with last_telemetry_ts defaulting to 0, i.e. seconds since 1970. That
   exceeded any timeout, so the supervisor triggered an emergency RTL on its
   very first tick -- before telemetry could possibly have arrived. Fixed by
   requiring that telemetry has arrived at least once before arming the check.
   (The missing writer for that timestamp is fixed in udp_receiver.py.)

3. THE BATTERY CHECK WAS COMMENTED OUT and the geofence was Python-only. The
   battery check is now live, and a real PX4 geofence is uploaded at mission
   start: an autopilot-enforced limit survives a Python crash, whereas this task
   does not.

Also: every duration now uses time.monotonic(). An NTP step under wall-clock
timing makes a safety timeout fire instantly or never.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

import config
from drone_agent import geo, udp_receiver

logger = logging.getLogger(__name__)


class SafetySupervisor:
    """Watches heartbeat, geofence, battery and sensor liveness."""

    def __init__(
        self,
        drone,
        drone_state: dict[str, Any],
        drone_id: str,
        home_lat: float,
        home_lon: float,
        geofence_radius_m: float = config.GEOFENCE_RADIUS_M,
        heartbeat_timeout_s: float = config.HEARTBEAT_TIMEOUT_S,
        lidar_data: dict[str, Any] | None = None,
    ):
        self._drone = drone
        self._drone_state = drone_state
        self._drone_id = drone_id
        self._home_lat = home_lat
        self._home_lon = home_lon
        self._geofence_radius_m = geofence_radius_m
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._lidar_data = lidar_data

        self._emergency_triggered = False
        self._emergency_reason = ""
        self._battery_warned = False
        self._lidar_stale_warned = False

        logger.info(
            "[%s] supervisor armed: geofence %.0f m, heartbeat %.1f s, "
            "battery critical below %d%%",
            drone_id, geofence_radius_m, heartbeat_timeout_s,
            config.BATTERY_CRITICAL_PCT,
        )

    # ── properties ───────────────────────────────────────────────────────────

    @property
    def emergency_triggered(self) -> bool:
        return self._emergency_triggered

    @property
    def emergency_reason(self) -> str:
        return self._emergency_reason

    # ── setup ────────────────────────────────────────────────────────────────

    async def upload_geofence(self) -> bool:
        """Push a real inclusion fence to the autopilot.

        This is the part that survives us. If this process is killed, the
        supervise() loop stops checking and a drone in OFFBOARD keeps flying its
        last setpoint; PX4's own fence still acts. The loop below remains the
        fast layer that can react earlier and record a reason.
        """
        from drone_agent.px4_params import upload_geofence

        return await upload_geofence(
            self._drone, self._home_lat, self._home_lon, self._geofence_radius_m
        )

    # ── main loop ────────────────────────────────────────────────────────────

    async def supervise(self) -> None:
        """Check everything once per second until cancelled."""
        logger.info("[%s] supervisor running", self._drone_id)
        interval_s = 1.0 / config.SUPERVISOR_HZ

        while True:
            try:
                if self._emergency_triggered:
                    # Nothing further to decide: the autopilot is handling it.
                    await asyncio.sleep(interval_s)
                    continue

                now = time.monotonic()

                if await self._check_heartbeat(now):
                    continue
                if await self._check_geofence():
                    continue
                if await self._check_battery():
                    continue
                self._check_sensor_liveness(now)

                await asyncio.sleep(interval_s)

            except asyncio.CancelledError:
                logger.info("[%s] supervisor stopped", self._drone_id)
                raise
            except Exception:  # noqa: BLE001
                # A supervisor that dies on an unexpected error is worse than one
                # that logs and keeps checking.
                logger.error(
                    "[%s] supervisor check raised; continuing", self._drone_id,
                    exc_info=True,
                )
                await asyncio.sleep(interval_s)

    # ── checks: each returns True if it triggered an emergency ───────────────

    async def _check_heartbeat(self, now: float) -> bool:
        """Telemetry silence means we have lost sight of the drone.

        THE t=0 GUARD. `has_ever_arrived` must be true before this check can
        fire. Without it, the very first tick compares against a zero timestamp,
        concludes telemetry has been missing for decades, and triggers an
        emergency RTL on a drone that is sitting safely on the ground waiting to
        arm.
        """
        if not udp_receiver.has_ever_arrived(
            self._drone_state, udp_receiver.TELEMETRY_TS_KEY
        ):
            return False

        age_s = udp_receiver.age_s(
            self._drone_state, udp_receiver.TELEMETRY_TS_KEY, now=now
        )
        if age_s > self._heartbeat_timeout_s:
            logger.critical(
                "[%s] TELEMETRY LOST: no update for %.1f s (limit %.1f s)",
                self._drone_id, age_s, self._heartbeat_timeout_s,
            )
            await self._trigger_emergency_rtl(
                f"Telemetry lost for {age_s:.1f}s"
            )
            return True
        return False

    async def _check_geofence(self) -> bool:
        """Distance from home against the soft fence."""
        distance_m = geo.get_distance_m(
            self._home_lat, self._home_lon,
            self._drone_state.get("lat", self._home_lat),
            self._drone_state.get("lon", self._home_lon),
        )
        if distance_m > self._geofence_radius_m:
            logger.critical(
                "[%s] GEOFENCE BREACH: %.0f m from home (limit %.0f m)",
                self._drone_id, distance_m, self._geofence_radius_m,
            )
            await self._trigger_emergency_rtl(
                f"Geofence breach at {distance_m:.0f}m from home"
            )
            return True
        return False

    async def _check_battery(self) -> bool:
        """Critical battery: land here rather than attempt to fly home.

        LAND, NOT RTL. Below the critical threshold there is by definition not
        enough energy budgeted to reach home, so an RTL would run the pack flat
        in transit and drop the drone from altitude somewhere unknown. Landing
        immediately puts it down under control.

        UNITS: MAVSDK 2.x reports remaining_percent on 0-100. In 1.x it was 0-1,
        so the same comparison was 100x wrong. pyproject.toml pins >=2.0,<3 for
        this reason; see the note there before relaxing it.
        """
        battery_pct = self._drone_state.get("battery_pct")
        if battery_pct is None:
            return False

        if battery_pct <= 0.0:
            # A zero reading almost always means telemetry has not populated
            # yet, not that the pack is empty. Landing on it would be a
            # false-positive emergency on every mission start.
            return False

        if battery_pct < config.BATTERY_CRITICAL_PCT:
            logger.critical(
                "[%s] CRITICAL BATTERY: %.1f%% (limit %d%%)",
                self._drone_id, battery_pct, config.BATTERY_CRITICAL_PCT,
            )
            await self._trigger_emergency_land(
                f"Critical battery at {battery_pct:.1f}%"
            )
            return True

        warn_at = config.BATTERY_CRITICAL_PCT + config.BATTERY_RESERVE_MARGIN_PCT
        if battery_pct < warn_at and not self._battery_warned:
            self._battery_warned = True
            logger.warning(
                "[%s] battery %.1f%% - below the %d%% reserve margin; the next "
                "leg's pre-flight gate will likely refuse",
                self._drone_id, battery_pct, warn_at,
            )
        return False

    def _check_sensor_liveness(self, now: float) -> None:
        """Note a stale obstacle feed. Deliberately does NOT trigger anything.

        Navigation already fails closed on this and holds position, which is the
        right response: the drone is stationary and safe. Escalating to an RTL
        would fly it across unmonitored airspace with no obstacle sensing at all
        -- strictly more dangerous than hovering.
        """
        if self._lidar_data is None:
            return

        if not udp_receiver.has_ever_arrived(
            self._lidar_data, udp_receiver.LIDAR_TS_KEY
        ):
            return

        age_s = udp_receiver.age_s(
            self._lidar_data, udp_receiver.LIDAR_TS_KEY, now=now
        )
        stale = age_s > 2.0 * config.STALE_SENSOR_TIMEOUT_S

        if stale and not self._lidar_stale_warned:
            self._lidar_stale_warned = True
            logger.warning(
                "[%s] obstacle feed stale for %.1f s. Navigation is holding "
                "position. Check avoider_node.py and ros_gz_bridge for this "
                "drone (see RUN_GUIDE.md).",
                self._drone_id, age_s,
            )
        elif not stale and self._lidar_stale_warned:
            self._lidar_stale_warned = False
            logger.info("[%s] obstacle feed recovered", self._drone_id)

    # ── emergency actions ────────────────────────────────────────────────────

    async def _trigger_emergency_rtl(self, reason: str) -> None:
        """Command return-to-launch, falling back to landing in place.

        THE LATCH ORDER IS THE FIX. `_emergency_triggered` is set only AFTER the
        autopilot accepts the command. Setting it first -- the old behaviour --
        made the `_trigger_emergency_land` fallback return immediately at its own
        guard, so an RTL that failed produced no action whatsoever.
        """
        if self._emergency_triggered:
            return

        self._drone_state["status"] = f"EMERGENCY RTL: {reason}"
        logger.critical("[%s] EMERGENCY RTL: %s", self._drone_id, reason)

        try:
            await asyncio.wait_for(self._drone.action.return_to_launch(), timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            logger.error(
                "[%s] RTL command REJECTED (%s) - falling back to landing here",
                self._drone_id, exc,
            )
            # force=True is what makes this fallback reachable at all.
            await self._trigger_emergency_land(f"RTL failed: {reason}", force=True)
            return

        self._emergency_triggered = True
        self._emergency_reason = reason
        logger.info("[%s] RTL accepted by the autopilot", self._drone_id)

    async def _trigger_emergency_land(self, reason: str, force: bool = False) -> None:
        """Command an immediate landing at the current position.

        Args:
            force: Proceed even if an emergency is already latched. Used by the
                RTL fallback, which is itself running inside an emergency.
        """
        if self._emergency_triggered and not force:
            return

        self._drone_state["status"] = f"EMERGENCY LAND: {reason}"
        logger.critical("[%s] EMERGENCY LAND: %s", self._drone_id, reason)

        try:
            await asyncio.wait_for(self._drone.action.land(), timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            # Nothing further this layer can do. Latch anyway so the mission
            # sees an emergency and stops commanding setpoints, and shout: this
            # is a drone that is not responding to commands.
            logger.critical(
                "[%s] LAND COMMAND ALSO REJECTED (%s). The autopilot is not "
                "accepting commands. PX4's own failsafes and the uploaded "
                "geofence are now the only remaining protection.",
                self._drone_id, exc,
            )
            self._emergency_triggered = True
            self._emergency_reason = f"{reason} (land also rejected)"
            return

        self._emergency_triggered = True
        self._emergency_reason = reason
        logger.info("[%s] emergency land accepted", self._drone_id)

    def reset(self) -> None:
        """Clear the latch. For operator intervention only."""
        logger.info("[%s] supervisor latch cleared", self._drone_id)
        self._emergency_triggered = False
        self._emergency_reason = ""
        self._battery_warned = False
