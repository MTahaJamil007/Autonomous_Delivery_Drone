"""
Payload bay management - kinematic cargo following without physics joints.

Design rationale: A physically hung payload introduces pendulum dynamics that
destabilize attitude control. Instead of modeling a real joint or hinge, the
cargo model is **kinematically pose-followed** to the drone each control tick
while "secured" — there is no physics constraint to swing, because there is
no physics joint at all.
"""

import asyncio
import logging
import math
import subprocess
import time
from typing import Dict, Any, Tuple, Callable

import sys
from pathlib import Path
import config

logger = logging.getLogger(__name__)


async def fixed_rate_loop(hz: float, tick_fn: Callable) -> None:
    """
    Execute tick_fn at a fixed rate with drift compensation.
    
    Args:
        hz: Target frequency in Hertz
        tick_fn: Async function to call each tick (should return False to stop loop)
    """
    period = 1.0 / hz
    next_tick = time.monotonic()
    
    while True:
        should_continue = await tick_fn()
        if should_continue is False:
            break
        
        next_tick += period
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            if sleep_time < -period:
                logger.warning(f"Payload update loop behind schedule by {-sleep_time:.3f}s")


class PayloadBay:
    """
    Simulated cargo compartment with kinematic following.
    
    While secured, the cargo model's pose is set directly (not physically
    constrained) to track the drone's position plus a fixed offset — this
    eliminates hanging/pendulum dynamics entirely.
    """
    
    def __init__(self, cargo_model_name: str, drone_id: str, home_lat: float = 0.0, home_lon: float = 0.0):
        """
        Initialize payload bay.
        
        Args:
            cargo_model_name: Name of the Gazebo cargo model entity
            drone_id: Identifier for this drone (e.g., "drone-0")
            home_lat, home_lon: Home position for coordinate transformation
        """
        self._cargo_model = cargo_model_name
        self._drone_id = drone_id
        self._secured = False
        self._home_lat = home_lat
        self._home_lon = home_lon
        self._following_task = None
        logger.info(
            f"PayloadBay initialized for {drone_id}: cargo_model={cargo_model_name}"
        )
    
    async def attach(self) -> bool:
        """
        Call once after PAYLOAD_OP entry at a pickup leg.
        
        Returns:
            True if attach successful
        """
        self._secured = True
        logger.info(f"[{self._drone_id}] Cargo attached: {self._cargo_model}")
        return True
    
    async def release(self) -> bool:
        """
        Call once after PAYLOAD_OP entry at a drop-off leg.
        
        Cargo remains exactly where last placed — since the drone is on the
        ground at touchdown, this is already the correct resting position.
        
        Returns:
            True if release successful
        """
        self._secured = False
        logger.info(f"[{self._drone_id}] Cargo released: {self._cargo_model}")
        return True
    
    def is_secured(self) -> bool:
        """Check if cargo is currently secured."""
        return self._secured
    
    async def update(self, drone_state: Dict[str, Any]) -> None:
        """
        Call every navigation tick to update cargo position.
        
        No-op unless secured. Computes the cargo's world pose from
        drone_state lat/lon/alt + BAY_OFFSET_NED_M and issues a Gazebo
        pose-set transport call.
        
        Args:
            drone_state: Dictionary with 'lat', 'lon', 'alt' keys
        """
        if not self._secured:
            return
        
        # Extract drone position
        lat = drone_state.get("lat", 0.0)
        lon = drone_state.get("lon", 0.0)
        alt = drone_state.get("alt", 0.0)
        
        # Apply NED offset
        offset_n, offset_e, offset_d = config.BAY_OFFSET_NED_M
        
        # Convert lat/lon to Gazebo X/Y coordinates
        x_east, y_north = self._lat_lon_to_gazebo_xy(lat, lon, self._home_lat, self._home_lon)
        
        # Calculate Z position (altitude minus down offset)
        z = alt - offset_d
        
        # Apply NED offsets in Gazebo frame
        x_cargo = x_east + offset_e
        y_cargo = y_north + offset_n
        
        # Issue Gazebo pose-set command
        cmd = (
            f"gz service -s /world/default/set_pose "
            f"--reqtype gz.msgs.Pose "
            f"--reptype gz.msgs.Boolean "
            f"--timeout 100 "
            f"--req 'name: \"{self._cargo_model}\", position: {{x: {x_cargo}, y: {y_cargo}, z: {z}}}'"
        )
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    async def start_following(self, drone_state: Dict[str, Any]) -> None:
        """
        Start a background task that continuously updates cargo position.
        
        This should be called once when the mission starts. The task runs
        at config.NAV_HZ and shares the same timing as the navigation loop.
        
        Args:
            drone_state: Dictionary with current drone state
        """
        async def update_tick() -> bool:
            """Single update tick. Returns True to continue."""
            await self.update(drone_state)
            return True  # Run indefinitely
        
        logger.info(f"[{self._drone_id}] Starting cargo following loop at {config.NAV_HZ} Hz")
        self._following_task = asyncio.create_task(fixed_rate_loop(config.NAV_HZ, update_tick))
    
    async def stop_following(self) -> None:
        """Stop the cargo following background task."""
        if self._following_task is not None:
            self._following_task.cancel()
            try:
                await self._following_task
            except asyncio.CancelledError:
                pass
            self._following_task = None
            logger.info(f"[{self._drone_id}] Cargo following loop stopped")
    
    def _lat_lon_to_gazebo_xy(
        self,
        lat: float,
        lon: float,
        home_lat: float,
        home_lon: float
    ) -> Tuple[float, float]:
        """
        Convert GPS coordinates to Gazebo world X/Y.
        
        Args:
            lat, lon: Target GPS coordinates
            home_lat, home_lon: Reference (home) GPS coordinates
            
        Returns:
            (x_east, y_north) in Gazebo world frame
        """
        d_lat = lat - home_lat
        d_lon = lon - home_lon
        y_north = d_lat * 111_320.0
        x_east = d_lon * (111_320.0 * math.cos(math.radians(home_lat)))
        return x_east, y_north
