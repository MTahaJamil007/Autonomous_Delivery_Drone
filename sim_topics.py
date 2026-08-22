"""Model-scoped Gazebo topic names, derived in one place.

WHY THIS MODULE EXISTS (finding F7)
-----------------------------------
The hand-added sensors used to hardcode `<topic>/camera/image</topic>` and
`<topic>/lidar/scan</topic>`. Those are absolute Gazebo topic names, so with
FLEET_SIZE=3 all three drones published onto the same two topics: every vision
bridge saw a blend of three cameras and every avoider reacted to a blend of
three LiDARs. Per-drone UDP ports solved the downstream half of that problem
and none of the upstream half.

sim/models/x500_delivery/model.sdf now omits `<topic>` entirely, so Gazebo
derives the name from the entity path:

    /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<suffix>

That is unique per spawned instance, which is exactly what a fleet needs. The
cost is that the name is no longer a constant anyone can type, so it is
computed here and passed as a CLI argument to the vision bridge and to
ros_gz_bridge. Every consumer must agree on the derivation, which is why there
is one function per topic and no string literals anywhere else.

The link and sensor names below must match model.sdf. scripts/preflight.py
verifies the derived names are actually advertised before anything flies.

Run as a script to print the topics for a drone -- shell launchers use this
rather than reimplementing the string:

    python3 -m sim_topics --drone-id drone-0 --what camera
"""

from __future__ import annotations

import config

CAMERA_LINK = "camera_link"
CAMERA_SENSOR = "downward_camera"
LIDAR_LINK = "lidar_link"
LIDAR_SENSOR = "rplidar_a1"


def model_name(drone_id: str, model_base: str = config.GZ_MODEL_BASE) -> str:
    """Gazebo entity name PX4 gives the drone.

    PX4's px4-rc.gzsim spawns `${PX4_SIM_MODEL}_${px4_instance}`, so drone-0 is
    `x500_delivery_0`. Note this is NOT settable via PX4_GZ_MODEL_NAME: setting
    that variable makes PX4 skip spawning and attach to a pre-existing model
    instead, which is a different launch mode entirely.
    """
    return f"{model_base}_{config.drone_index(drone_id)}"


def camera_topic(drone_id: str, world: str = config.GZ_WORLD) -> str:
    """Gazebo transport topic carrying the downward camera's images."""
    return (
        f"/world/{world}/model/{model_name(drone_id)}"
        f"/link/{CAMERA_LINK}/sensor/{CAMERA_SENSOR}/image"
    )


def lidar_topic(drone_id: str, world: str = config.GZ_WORLD) -> str:
    """Gazebo transport topic carrying the 2D LiDAR's scans."""
    return (
        f"/world/{world}/model/{model_name(drone_id)}"
        f"/link/{LIDAR_LINK}/sensor/{LIDAR_SENSOR}/scan"
    )


def ros_scan_topic(drone_id: str) -> str:
    """Short ROS topic the LiDAR is remapped onto by ros_gz_bridge.

    The raw Gazebo name is long enough to be unwieldy in `ros2 topic` output
    and in launch files, so the bridge remaps it to this. avoider_node
    subscribes here.
    """
    return f"/{drone_id.replace('-', '_')}/scan"


def ros_gz_bridge_args(drone_id: str, world: str = config.GZ_WORLD) -> list[str]:
    """Full argv for the ros_gz_bridge process serving one drone's LiDAR."""
    gz_topic = lidar_topic(drone_id, world)
    return [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        f"{gz_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "--ros-args", "-r", f"{gz_topic}:={ros_scan_topic(drone_id)}",
    ]


def all_topics(drone_id: str, world: str = config.GZ_WORLD) -> dict[str, str]:
    """Every simulator topic one drone depends on, for preflight checking."""
    return {
        "camera": camera_topic(drone_id, world),
        "lidar": lidar_topic(drone_id, world),
    }


def _main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--drone-id", default="drone-0")
    parser.add_argument("--world", default=config.GZ_WORLD)
    parser.add_argument(
        "--what",
        choices=["camera", "lidar", "ros-scan", "bridge-args", "all"],
        default="all",
    )
    args = parser.parse_args()

    if args.what == "camera":
        print(camera_topic(args.drone_id, args.world))
    elif args.what == "lidar":
        print(lidar_topic(args.drone_id, args.world))
    elif args.what == "ros-scan":
        print(ros_scan_topic(args.drone_id))
    elif args.what == "bridge-args":
        print(" ".join(ros_gz_bridge_args(args.drone_id, args.world)))
    else:
        for name, topic in all_topics(args.drone_id, args.world).items():
            print(f"{name}\t{topic}")
        print(f"ros-scan\t{ros_scan_topic(args.drone_id)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
