"""Payload bay: kinematic cargo following, off the control loop's critical path.

DESIGN: KINEMATIC, NOT PHYSICAL
-------------------------------
The cargo is not joined to the airframe. A physically hung payload introduces
pendulum dynamics that fight the attitude controller, and modelling a hinge in
simulation buys nothing the mission cares about. Instead the cargo model's pose
is commanded to follow the drone while secured. There is no physics constraint
to swing, because there is no joint at all. sim/models/cargo_box is
`static=true` for the same reason -- a dynamic body would free-fall between
pose updates.

TWO FIXES (P3.8)
----------------
1. THE CARGO MODEL DID NOT EXIST. `update()` issued gz set_pose calls naming an
   entity that was present nowhere on this machine, so every call silently
   failed and "the payload bay kinematically follows the airframe" could never
   have been observed working, no matter how correct this code was.
   sim/models/cargo_box/ now provides it, and spawn() creates the instance.

2. IT BLOCKED THE EVENT LOOP (finding F5). `update()` called
   `subprocess.run("gz service ...", shell=True)` synchronously from a tick
   scheduled at config.NAV_HZ. That froze the whole loop -- including the
   offboard setpoint publisher -- for tens to hundreds of milliseconds, ten
   times a second. PX4 drops OFFBOARD when the setpoint stream gaps, so simply
   enabling this documented feature would have caused mid-flight mode loss.

   All Gazebo I/O now goes through drone_agent.gz_client, which uses
   asyncio.create_subprocess_exec. The following loop is also its own task at
   config.PAYLOAD_HZ rather than being inline in the navigation tick, so a slow
   pose call delays only the cargo's visual position.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

import config
from drone_agent import geo, gz_client

logger = logging.getLogger(__name__)


class PayloadBay:
    """A simulated cargo compartment whose contents track the airframe."""

    def __init__(
        self,
        drone_id: str,
        home_lat: float = 0.0,
        home_lon: float = 0.0,
        cargo_model_name: str | None = None,
    ):
        """
        Args:
            drone_id: Owning drone. Also namespaces the cargo entity, because
                three drones sharing one cargo entity name would each teleport
                the same box and only the last writer would be visible.
            home_lat, home_lon: Origin for the GPS-to-Gazebo conversion. Must be
                the same origin the world was configured with, or the cargo
                appears offset from the drone by the difference.
            cargo_model_name: Override the entity name. Defaults to
                '{CARGO_MODEL_PREFIX}_{drone_id}'.
        """
        self._drone_id = drone_id
        self._home_lat = home_lat
        self._home_lon = home_lon
        self._cargo_model = cargo_model_name or f"{config.CARGO_MODEL_PREFIX}_{drone_id}"

        self._secured = False
        self._spawned = False
        self._task: asyncio.Task | None = None
        self._pose_failures = 0

    # ── properties ───────────────────────────────────────────────────────────

    @property
    def cargo_model(self) -> str:
        return self._cargo_model

    def is_secured(self) -> bool:
        return self._secured

    # ── lifecycle ────────────────────────────────────────────────────────────

    async def spawn(self, lat: float, lon: float) -> bool:
        """Place the parcel in the world at the pickup point.

        Called before the pickup leg so the box is visibly waiting there, which
        also makes the attach step observable rather than notional.
        """
        x_east_m, y_north_m = geo.lat_lon_to_local_enu(lat, lon, self._home_lat, self._home_lon)

        # Remove any box left by a previous mission: Gazebo refuses a duplicate
        # entity name rather than replacing it, so the second dispatch of a
        # session would otherwise have no cargo at all.
        await gz_client.remove_model(self._cargo_model)

        self._spawned = await gz_client.spawn_model(
            config.CARGO_SDF_URI,
            self._cargo_model,
            x_east_m,
            y_north_m,
            z_up_m=0.1,
        )
        if self._spawned:
            logger.info(
                "[%s] cargo '%s' placed at the pickup point",
                self._drone_id,
                self._cargo_model,
            )
        else:
            logger.warning(
                "[%s] could not spawn cargo '%s'. The mission continues -- the "
                "parcel is cosmetic -- but the payload demonstration will show "
                "nothing. Check that GZ_SIM_RESOURCE_PATH includes "
                "sim/models (source sim/env.sh).",
                self._drone_id,
                self._cargo_model,
            )
        return self._spawned

    async def attach(self) -> bool:
        """Secure the cargo. Called on PAYLOAD_OP entry at the pickup leg."""
        self._secured = True
        logger.info("[%s] cargo secured: %s", self._drone_id, self._cargo_model)
        return True

    async def release(self) -> bool:
        """Release the cargo where it now sits.

        No placement needed: the drone is on the ground at touchdown when this
        runs, so the last followed pose is already the correct resting position.
        """
        self._secured = False
        logger.info("[%s] cargo released: %s", self._drone_id, self._cargo_model)
        return True

    # ── following ────────────────────────────────────────────────────────────

    async def update(self, drone_state: dict[str, Any]) -> None:
        """Move the cargo to the bay's current world position. No-op if released."""
        if not self._secured or not self._spawned:
            return

        offset_north_m, offset_east_m, offset_down_m = config.BAY_OFFSET_NED_M

        x_east_m, y_north_m = geo.lat_lon_to_local_enu(
            drone_state.get("lat", 0.0),
            drone_state.get("lon", 0.0),
            self._home_lat,
            self._home_lon,
        )

        # NED offsets into the ENU world frame. Down is positive in NED and z is
        # UP in Gazebo, hence the subtraction: a +0.35 m "down" offset must
        # lower the box.
        await gz_client.set_model_pose(
            self._cargo_model,
            x_east_m + offset_east_m,
            y_north_m + offset_north_m,
            max(0.02, drone_state.get("alt", 0.0) - offset_down_m),
        )

    async def start_following(self, drone_state: dict[str, Any]) -> None:
        """Run the following loop as its own task for the mission's lifetime.

        Its own task, deliberately. Inline in the navigation tick -- the previous
        arrangement -- put a subprocess launch on the control loop's critical
        path ten times a second.
        """
        if self._task is not None:
            return

        async def loop() -> None:
            period_s = 1.0 / config.PAYLOAD_HZ
            while True:
                try:
                    await self.update(drone_state)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:  # noqa: BLE001
                    self._pose_failures += 1
                    if self._pose_failures <= 3 or self._pose_failures % 100 == 0:
                        logger.warning(
                            "[%s] cargo pose update failed (#%d): %s",
                            self._drone_id,
                            self._pose_failures,
                            exc,
                        )
                await asyncio.sleep(period_s)

        self._task = asyncio.create_task(loop(), name=f"cargo-{self._drone_id}")
        logger.info(
            "[%s] cargo following at %d Hz (async, off the control loop)",
            self._drone_id,
            config.PAYLOAD_HZ,
        )

    async def stop_following(self) -> None:
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        self._task = None
        logger.info(
            "[%s] cargo following stopped (%d pose failures)",
            self._drone_id,
            self._pose_failures,
        )

    async def cleanup(self) -> None:
        """Stop following and remove the cargo entity at mission end."""
        await self.stop_following()
        if self._spawned:
            await gz_client.remove_model(self._cargo_model)
            self._spawned = False
