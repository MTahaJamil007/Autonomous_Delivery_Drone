#!/usr/bin/env python3
"""Confirm the camera sign convention empirically, and measure fx.

WHY THIS SCRIPT EXISTS (finding F6)
-----------------------------------
The image-to-body sign mapping is the one thing in the landing loop that is
easiest to "fix" into a crash. Invert either sign and the controller becomes a
positive feedback loop that accelerates away from the pad — and the symptom
(drone drifts off and the landing times out) looks exactly like a marker that was
never properly acquired. Under time pressure, someone will flip a sign to see if
it helps.

So the derivation is recorded in docs/CALIBRATION.md, and this script confirms
it on the actual airframe rather than leaving it as an argument about rotation
matrices.

METHOD
------
1. Hold a hover with a marker in view.
2. Command a known body-frame velocity for a fixed duration (default: forward).
3. Observe which way the marker's pixel coordinates moved.
4. Compare against the prediction and report agreement or contradiction.

The prediction: flying FORWARD moves the ground beneath the drone backwards
relative to the airframe. Image v (down) maps to body AFT, so a marker the drone
is flying towards moves DOWN the image — err_y INCREASES.

Requires a running PX4 SITL, a spawned drone, and a marker in the camera's view.
Fly it over a pad. Nothing here arms or takes off: put the drone in a hover
first, then run this.

Usage:
    python3 scripts/calibrate_camera_signs.py --drone-id drone-0
    python3 scripts/calibrate_camera_signs.py --axis right --speed 0.5
"""

# ruff: noqa: E402 - the sys.path bootstrap below must run before the
# project imports, so that this script works before `pip install -e .`.
from __future__ import annotations

import argparse
import asyncio
import statistics
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config
from drone_agent import udp_receiver
from drone_agent.landing import select_target
from drone_agent.setpoint import SetpointPublisher


async def collect_detections(
    vision_data: dict, marker_id: int, duration_s: float
) -> list[tuple[float, float, float, float]]:
    """Sample (t, err_x, err_y, size_px) while the drone is moving."""
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        detection = select_target(vision_data.get("detections") or [], marker_id)
        if detection is not None:
            samples.append(
                (
                    time.monotonic(),
                    float(detection["err_x"]),
                    float(detection["err_y"]),
                    float(detection.get("size_px", 0.0)),
                )
            )
        await asyncio.sleep(0.05)
    return samples


async def run(args: argparse.Namespace) -> int:
    from mavsdk import System

    drone_id = args.drone_id
    vision_data: dict = {"detections": []}

    receiver = udp_receiver.make_vision_receiver(drone_id, vision_data)
    receiver.bind()
    receiver_task = asyncio.create_task(receiver.run())

    drone = System(
        mavsdk_server_address=None,
        port=config.MAVSDK_GRPC_PORT_BASE + config.drone_index(drone_id),
        sysid=config.MAVSDK_SYSID_BASE + config.drone_index(drone_id),
    )
    await asyncio.wait_for(drone.connect(system_address=config.mavsdk_url(drone_id)), timeout=15.0)

    drone_state: dict = {"alt": 0.0}

    async def track_altitude() -> None:
        async for position in drone.telemetry.position():
            drone_state["alt"] = position.relative_altitude_m

    altitude_task = asyncio.create_task(track_altitude())
    await asyncio.sleep(2.0)

    print(f"altitude {drone_state['alt']:.2f} m")
    print("waiting for the marker to come into view…")
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if select_target(vision_data.get("detections") or [], args.marker_id):
            break
        await asyncio.sleep(0.2)
    else:
        print(
            f"ERROR: marker {args.marker_id} never appeared. Is the vision "
            f"bridge running for {drone_id}, and is the drone over a pad?"
        )
        return 1

    publisher = SetpointPublisher(drone, drone_id)
    await publisher.start()
    heading = publisher.setpoint.yaw_deg

    try:
        print("\nbaseline (hovering)…")
        baseline = await collect_detections(vision_data, args.marker_id, 2.0)
        if not baseline:
            print("ERROR: no detections while hovering")
            return 1
        base_x = statistics.median(s[1] for s in baseline)
        base_y = statistics.median(s[2] for s in baseline)
        base_size = statistics.median(s[3] for s in baseline)
        print(f"  err_x={base_x:+.1f} px  err_y={base_y:+.1f} px  size={base_size:.1f} px")

        forward = args.speed if args.axis == "forward" else 0.0
        right = args.speed if args.axis == "right" else 0.0
        print(f"\ncommanding {args.axis} at {args.speed} m/s for {args.duration}s…")
        publisher.command_body_horizontal(forward, right, 0.0, heading)

        moving = await collect_detections(vision_data, args.marker_id, args.duration)
        publisher.hold(heading)

        if len(moving) < 5:
            print(
                f"ERROR: only {len(moving)} detections while moving — the "
                f"marker probably left the frame. Try a lower --speed."
            )
            return 1

        print("\nsettling…")
        await asyncio.sleep(2.0)
        after = await collect_detections(vision_data, args.marker_id, 2.0)
        if not after:
            print("ERROR: marker lost after the manoeuvre")
            return 1
        end_x = statistics.median(s[1] for s in after)
        end_y = statistics.median(s[2] for s in after)
        print(f"  err_x={end_x:+.1f} px  err_y={end_y:+.1f} px")

    finally:
        publisher.hold(heading)
        await publisher.stop()
        altitude_task.cancel()
        receiver_task.cancel()
        for task in (altitude_task, receiver_task):
            try:
                await task
            except asyncio.CancelledError:
                pass

    # ── Interpret ────────────────────────────────────────────────────────────
    delta_x = end_x - base_x
    delta_y = end_y - base_y
    altitude_m = max(drone_state["alt"], 0.1)

    print("\n" + "=" * 62)
    print(f"  observed:  delta err_x = {delta_x:+.1f} px")
    print(f"             delta err_y = {delta_y:+.1f} px")
    print(f"  altitude:  {altitude_m:.2f} m")

    if args.axis == "forward":
        expected_axis, observed, other = "err_y", delta_y, delta_x
        prediction = (
            "flying FORWARD should INCREASE err_y (the pad moves down the "
            "image, because image v maps to body aft)"
        )
        agrees = observed > 0
    else:
        expected_axis, observed, other = "err_x", delta_x, delta_y
        prediction = (
            "flying RIGHT should DECREASE err_x (the pad moves left in the "
            "image, because image u maps to body right)"
        )
        agrees = observed < 0

    print(f"\n  prediction: {prediction}")
    print(f"  result:     {expected_axis} changed by {observed:+.1f} px -> ", end="")
    print("AGREES" if agrees else "*** CONTRADICTS ***")

    if abs(other) > abs(observed) * 0.5:
        print(
            f"  WARNING: the other axis moved {other:+.1f} px, which is a lot "
            f"for a single-axis command. Check the yaw was held."
        )

    # fx from the measured pixel displacement over a known ground displacement.
    ground_travel_m = args.speed * args.duration
    if abs(observed) > 1.0 and ground_travel_m > 0:
        measured_fx = abs(observed) * altitude_m / ground_travel_m
        error_pct = 100.0 * (measured_fx - config.CAMERA_FX_PX) / config.CAMERA_FX_PX
        print(f"\n  fx measured:  {measured_fx:.1f} px")
        print(f"  fx in config: {config.CAMERA_FX_PX:.1f} px  ({error_pct:+.1f}%)")
        if abs(error_pct) > 20:
            print(
                "  WARNING: more than 20% off. The commanded velocity is not "
                "the achieved velocity during acceleration, so some error is "
                "expected — but this much suggests the intrinsics are wrong."
            )

    print("=" * 62)
    print("\nRecord this run in docs/CALIBRATION.md, with the date and the measured values.")
    return 0 if agrees else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Empirically confirm the camera sign convention.",
    )
    parser.add_argument("--drone-id", default="drone-0")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--axis", choices=["forward", "right"], default="forward")
    parser.add_argument("--speed", type=float, default=0.4, metavar="M_S")
    parser.add_argument("--duration", type=float, default=3.0, metavar="S")
    args = parser.parse_args()

    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
