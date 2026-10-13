"""Pads and cargo go where the job says, wherever the drone took off from.

THE FAILURE THIS PINS DOWN (2026-10-13)
---------------------------------------
A mission converted pad positions to Gazebo coordinates relative to the drone's
HOME -- wherever it armed -- and spawned them in Gazebo's WORLD frame. The two
agree only when the drone takes off from the world origin. After an RTL had
landed the drone at a previous pickup point, the next mission's pads all
appeared ~22 m from where they belonged, the drone searched bare ground, and
the leg failed `search_exhausted`. Every fleet drone but drone-0 is spawned
off-origin by world/spawn_fleet.sh, so for them this was the normal case.
"""

from __future__ import annotations

import pytest

import config
from drone_agent import geo, gz_client
from drone_agent.contracts import DeliveryJob
from drone_agent.mission import DroneMission
from drone_agent.payload import PayloadBay

# A home 25 m south of the world origin: where the RTL actually landed.
HOME = geo.local_enu_to_lat_lon(0.0, -25.0, config.GZ_WORLD_ORIGIN_LAT, config.GZ_WORLD_ORIGIN_LON)
PICKUP_ENU = (12.0, 30.0)  # world-frame east/north of the pickup pad


@pytest.fixture
def spawned(monkeypatch):
    calls: dict[str, tuple[float, float]] = {}

    async def fake_spawn_model(uri, name, x_east_m, y_north_m, **_kwargs):
        calls[name] = (x_east_m, y_north_m)
        return True

    async def fake_remove(*_args, **_kwargs):
        return True

    monkeypatch.setattr(gz_client, "gz_available", lambda: True)
    monkeypatch.setattr(gz_client, "spawn_model", fake_spawn_model)
    monkeypatch.setattr(gz_client, "remove_model", fake_remove)
    return calls


def _job() -> DeliveryJob:
    lat, lon = geo.local_enu_to_lat_lon(
        *PICKUP_ENU, config.GZ_WORLD_ORIGIN_LAT, config.GZ_WORLD_ORIGIN_LON
    )
    return DeliveryJob(job_id="t", pickup_lat=lat, pickup_lon=lon, drop_lat=lat, drop_lon=lon)


async def test_pads_land_in_the_world_frame_from_an_off_origin_home(spawned):
    mission = DroneMission("drone-0")
    mission.home_lat, mission.home_lon = HOME
    mission._payload = PayloadBay("drone-0", config.GZ_WORLD_ORIGIN_LAT, config.GZ_WORLD_ORIGIN_LON)

    await mission._spawn_pads(_job())

    pickup = spawned["drone-0_pickup_pad"]
    home = spawned["drone-0_home_pad"]
    assert pickup == pytest.approx(PICKUP_ENU, abs=0.05), "pickup pad was placed relative to home"
    assert home == pytest.approx((0.0, -25.0), abs=0.05), "home pad must sit under the drone"


async def test_cargo_lands_in_the_world_frame(spawned):
    bay = PayloadBay("drone-0", config.GZ_WORLD_ORIGIN_LAT, config.GZ_WORLD_ORIGIN_LON)
    lat, lon = geo.local_enu_to_lat_lon(
        *PICKUP_ENU, config.GZ_WORLD_ORIGIN_LAT, config.GZ_WORLD_ORIGIN_LON
    )
    await bay.spawn(lat, lon)
    assert next(iter(spawned.values())) == pytest.approx(PICKUP_ENU, abs=0.05)
