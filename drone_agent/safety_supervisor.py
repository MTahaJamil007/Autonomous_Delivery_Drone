"""
Safety Supervisor - Independent safety monitoring and emergency actions.

Runs as an independent asyncio task with direct MAVSDK authority, bypassing
the mission FSM for critical safety interventions.
"""

import asyncio
import logging
import time
from typing import Dict, Any
from mavsdk import System

import sys
from pathlib import Path
import config

logger = logging.getLogger(__name__)


class SafetySupervisor:
    """
    Independent safety monitor with direct control authority.
    
    Monitors:
    - Heartbeat loss (connection timeout)
    - Geofence breaches
    - Critical battery levels
    
    Actions:
    - Triggers RTL (Return to Launch) or emergency land
    - Bypasses mission FSM for immediate response
    """
    
    def __init__(
        self,
        drone: System,
        drone_state: Dict[str, Any],
        drone_id: str,
        home_lat: float,
        home_lon: float,
        geofence_radius_m: float = 500.0,
        heartbeat_timeout_s: float = 5.0,
        lidar_data: Dict[str, Any] = None  # R5 BUG 2 FIX: Add lidar_data parameter
    ):
        """
        Initialize safety supervisor.
        
        Args:
            drone: MAVSDK System instance
            drone_state: Shared drone state dictionary
            drone_id: Drone identifier
            home_lat, home_lon: Home position for geofence center
            geofence_radius_m: Maximum distance from home
            heartbeat_timeout_s: Heartbeat timeout threshold
            lidar_data: Shared lidar data dictionary (for obstacle feed staleness check)
        """
        self._drone = drone
        self._drone_state = drone_state
        self._drone_id = drone_id
        self._home_lat = home_lat
        self._home_lon = home_lon
        self._geofence_radius_m = geofence_radius_m
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._lidar_data = lidar_data  # R5 BUG 2 FIX
        
        self._last_heartbeat = time.time()
        self._emergency_triggered = False
        
        logger.info(
            f"[{drone_id}] Safety supervisor initialized: "
            f"geofence={geofence_radius_m}m, heartbeat_timeout={heartbeat_timeout_s}s"
        )
    
    def _get_distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate flat-earth distance in meters."""
        import math
        d_lat = (lat2 - lat1) * 111_320.0
        d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
        return math.hypot(d_lat, d_lon)
    
    async def supervise(self) -> None:
        """
        Main supervision loop. Runs continuously and independently of mission FSM.
        
        Should be started as: asyncio.create_task(supervisor.supervise())
        """
        logger.info(f"[{self._drone_id}] Safety supervisor started")
        
        check_interval = 1.0  # Check every second
        
        while True:
            try:
                # Skip checks if emergency already triggered
                if self._emergency_triggered:
                    await asyncio.sleep(check_interval)
                    continue
                
                # ── Check 1: Heartbeat ──────────────────────────────────────
                # R2.4: Read last telemetry timestamp from drone_state
                last_telemetry_ts = self._drone_state.get("last_telemetry_ts", 0)
                time_since_heartbeat = time.time() - last_telemetry_ts
                
                if time_since_heartbeat > self._heartbeat_timeout_s:
                    logger.critical(
                        f"[{self._drone_id}] HEARTBEAT LOSS! "
                        f"No update for {time_since_heartbeat:.1f}s"
                    )
                    await self._trigger_emergency_rtl("Heartbeat loss")
                    continue
                
                # ── Check 2: Battery Critical ───────────────────────────────
                # Note: Requires battery telemetry integration
                # battery_pct = drone_state.get("battery_pct", 100.0)
                # if battery_pct < config.BATTERY_CRITICAL_PCT:
                #     logger.critical(
                #         f"[{self._drone_id}] CRITICAL BATTERY! {battery_pct:.1f}%"
                #     )
                #     await self._trigger_emergency_land("Critical battery")
                #     continue
                
                # ── Check 3: Geofence Breach ────────────────────────────────
                current_lat = self._drone_state.get("lat", 0.0)
                current_lon = self._drone_state.get("lon", 0.0)
                
                distance_from_home = self._get_distance_m(
                    self._home_lat, self._home_lon,
                    current_lat, current_lon
                )
                
                if distance_from_home > self._geofence_radius_m:
                    logger.critical(
                        f"[{self._drone_id}] GEOFENCE BREACH! "
                        f"{distance_from_home:.0f}m from home (limit: {self._geofence_radius_m}m)"
                    )
                    await self._trigger_emergency_rtl("Geofence breach")
                    continue
                
                # R5 BUG 2 FIX: Check 4: Obstacle Feed Staleness ─────────────
                if self._lidar_data is not None:
                    LIDAR_STALE_TIMEOUT_S = 2.0  # More lenient than nav (nav checks every tick)
                    last_lidar_ts = self._lidar_data.get("last_lidar_ts", 0)
                    time_since_lidar = time.time() - last_lidar_ts
                    
                    if time_since_lidar > LIDAR_STALE_TIMEOUT_S and last_lidar_ts > 0:
                        # Only warn if we've ever received data (last_lidar_ts > 0)
                        # This avoids false alarms during startup
                        logger.warning(
                            f"[{self._drone_id}] OBSTACLE FEED STALE! "
                            f"No update for {time_since_lidar:.1f}s. "
                            f"Check avoider_node.py and ros_gz_bridge (see HOW_TO_RUN.md)."
                        )
                        self._drone_state["status"] = "⚠️ Obstacle feed stale"
                        # Don't trigger emergency - navigation will hold position
                
                # All checks passed
                await asyncio.sleep(check_interval)
            
            except Exception as e:
                logger.error(
                    f"[{self._drone_id}] Safety supervisor error: {e}",
                    exc_info=True
                )
                await asyncio.sleep(check_interval)
    
    async def _trigger_emergency_rtl(self, reason: str) -> None:
        """
        Trigger emergency Return to Launch.
        
        Args:
            reason: Human-readable reason for emergency
        """
        if self._emergency_triggered:
            return
        
        self._emergency_triggered = True
        self._drone_state["status"] = f"EMERGENCY RTL: {reason}"
        
        logger.critical(f"[{self._drone_id}] 🚨 EMERGENCY RTL: {reason}")
        
        try:
            await self._drone.action.return_to_launch()
            logger.info(f"[{self._drone_id}] RTL command sent successfully")
        except Exception as e:
            logger.error(
                f"[{self._drone_id}] Failed to trigger RTL: {e}",
                exc_info=True
            )
            # Fallback: try emergency land
            await self._trigger_emergency_land(f"RTL failed: {reason}")
    
    async def _trigger_emergency_land(self, reason: str) -> None:
        """
        Trigger emergency landing at current position.
        
        Args:
            reason: Human-readable reason for emergency
        """
        if self._emergency_triggered:
            return
        
        self._emergency_triggered = True
        self._drone_state["status"] = f"EMERGENCY LAND: {reason}"
        
        logger.critical(f"[{self._drone_id}] 🚨 EMERGENCY LAND: {reason}")
        
        try:
            await self._drone.action.land()
            logger.info(f"[{self._drone_id}] Emergency land command sent")
        except Exception as e:
            logger.error(
                f"[{self._drone_id}] Failed to trigger emergency land: {e}",
                exc_info=True
            )
    
    def reset(self) -> None:
        """Reset emergency flag (use after manual intervention)."""
        logger.info(f"[{self._drone_id}] Safety supervisor reset")
        self._emergency_triggered = False
