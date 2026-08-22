#!/usr/bin/env python3
"""Diagnose a MAVLink connection to one drone.

Standalone on purpose: when a dispatch fails with "no MAVLink heartbeat", this
answers "is the autopilot reachable at all?" without the mission stack, the
services or the simulator's sensor topics in the way.

    python3 test_connection_now.py                 # drone-0
    python3 test_connection_now.py --drone-id drone-2
    python3 test_connection_now.py --port 14540    # a specific port

WHY THIS FILE WAS REWRITTEN
---------------------------
The previous version had three defects, and all three are instructive:

1. It used `udpin://0.0.0.0:PORT`, which **MAVSDK 2.12.10 rejects outright** as
   an invalid connection URL. mavsdk_server exits, and System.connect() then
   blocks forever in channel_ready_future - so this diagnostic hung rather than
   reporting a failure. The URL now comes from config.mavsdk_url(), so the
   diagnostic and the flight code cannot disagree about it.

2. Its wait loop was

       async for state in drone.core.connection_state():
           if state.is_connected: ...
           break                       # <-- unconditional

   which exits after ONE sample whether or not a heartbeat arrived. That is the
   same pattern that made the old drone_logic report a connection it had never
   made.

3. A bare `except:` swallowed everything including KeyboardInterrupt.
"""

# ruff: noqa: E402 - the sys.path bootstrap below must run before `import
# config`, so that this diagnostic works before `pip install -e .`.
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config


async def probe(url: str, grpc_port: int, sysid: int, timeout_s: float) -> bool:
    """True if a MAVLink heartbeat arrives on `url` within the timeout."""
    from mavsdk import System

    print(f"probing {url} (mavsdk_server gRPC :{grpc_port}, sysid {sysid})…")

    drone = System(mavsdk_server_address=None, port=grpc_port, sysid=sysid)

    # BOUNDED. connect() ends in aiogrpc.channel_ready_future(), which waits
    # forever if the mavsdk_server it just spawned has exited - which is exactly
    # what a rejected connection URL causes.
    try:
        await asyncio.wait_for(drone.connect(system_address=url), timeout=timeout_s + 6.0)
    except asyncio.TimeoutError:
        print("  FAIL: mavsdk_server never became ready.")
        print("        It usually exits at startup over a rejected connection")
        print(f"        URL. This build was given: {url}")
        return False

    async def wait_for_heartbeat() -> None:
        # No unconditional break: keep reading until genuinely connected.
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(wait_for_heartbeat(), timeout=timeout_s)
    except asyncio.TimeoutError:
        print(f"  FAIL: no heartbeat within {timeout_s:.0f}s.")
        return False

    print("  OK: connected.")

    # A heartbeat alone is not readiness; report what the autopilot thinks.
    try:

        async def first_health():
            async for health in drone.telemetry.health():
                return health

        health = await asyncio.wait_for(first_health(), timeout=5.0)
        print(f"     global position ok : {health.is_global_position_ok}")
        print(f"     home position ok   : {health.is_home_position_ok}")
        print(f"     armable            : {health.is_armable}")
        if not health.is_armable:
            print("     NOTE: connected but not armable - the EKF has probably")
            print("           not converged yet. Wait, then re-run.")
    except asyncio.TimeoutError:
        print("     (health telemetry did not arrive within 5s)")

    return True


async def main_async(args: argparse.Namespace) -> int:
    if args.port is not None:
        urls = [(f"{config.MAVSDK_URL_SCHEME}://0.0.0.0:{args.port}", args.port)]
        index = 0
    else:
        index = config.drone_index(args.drone_id)
        urls = [(config.mavsdk_url(args.drone_id), config.mavsdk_port(args.drone_id))]

    print("=" * 58)
    print(" MAVLink connection diagnostic")
    print("=" * 58)

    for url, port in urls:
        if await probe(
            url,
            config.MAVSDK_GRPC_PORT_BASE + index,
            config.MAVSDK_SYSID_BASE + index,
            args.timeout,
        ):
            print(f"\nPX4 is reachable on port {port}.")
            return 0

    print("\nCould not reach PX4. Check, in order:")
    print("  1. Is a PX4 instance running?   pgrep -af px4_sitl_default/bin/px4")
    print("  2. Did it print 'Ready for takeoff!'?")
    print("  3. Does the instance number match the drone id? PX4 publishes to")
    print("     14540 + instance, so drone-2 needs `px4 -i 2`.")
    print("  4. Start it with:  world/spawn_fleet.sh")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--drone-id", default="drone-0")
    parser.add_argument(
        "--port", type=int, default=None, help="Probe this UDP port instead of deriving it."
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
