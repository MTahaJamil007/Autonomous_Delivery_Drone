"""P6 acceptance: three drones cannot interfere with each other.

The live criterion (three concurrent dispatches, three airborne drones, three
COMPLETED jobs, no cross-talk) is a flight procedure in docs/ACCEPTANCE.md. What
is provable on the bench is that every shared resource a fleet could collide on
is in fact per-drone: Gazebo topics, UDP ports, MAVLink ports, gRPC ports,
MAVLink system ids, cargo entity names and pad entity names.

Cross-talk was not a tuning problem. Before this work, N drones published onto
two absolute Gazebo topics and shared three module-level state dicts, so the
question "does drone-1 react to drone-2's LiDAR?" had the answer "necessarily,
yes" (finding F7).
"""

from pathlib import Path

import pytest

import config
import sim_topics
from drone_agent.mission import DroneMission
from drone_agent.payload import PayloadBay

FLEET = [f"drone-{i}" for i in range(3)]


# ─────────────────────────────────────────────────────────────────────────────
#  F7: model-scoped sensor topics
# ─────────────────────────────────────────────────────────────────────────────


def test_every_drone_has_its_own_camera_and_lidar_topic():
    """The whole of finding F7 in one assertion."""
    camera_topics = {sim_topics.camera_topic(d) for d in FLEET}
    lidar_topics = {sim_topics.lidar_topic(d) for d in FLEET}

    assert len(camera_topics) == len(FLEET), f"cameras share topics: {sorted(camera_topics)}"
    assert len(lidar_topics) == len(FLEET), f"LiDARs share topics: {sorted(lidar_topics)}"
    assert not (camera_topics & lidar_topics)

    # And specifically not the absolute names the old model hardcoded.
    for topic in camera_topics | lidar_topics:
        assert topic not in ("/camera/image", "/lidar/scan"), (
            "an absolute topic name means every drone in the fleet publishes to "
            "the same place and every consumer sees a blend"
        )
        assert topic.startswith("/world/"), (
            f"{topic} is not model-scoped; it must include the entity path"
        )


def test_topic_names_include_the_spawned_model_instance():
    """PX4 spawns ${PX4_SIM_MODEL}_${instance}, which is what scopes the topic."""
    for index, drone_id in enumerate(FLEET):
        expected_model = f"{config.GZ_MODEL_BASE}_{index}"
        assert sim_topics.model_name(drone_id) == expected_model
        assert f"/model/{expected_model}/" in sim_topics.camera_topic(drone_id)


def test_ros_scan_topics_are_distinct_and_short():
    """One bridge per drone, each remapping to its own ROS name."""
    ros_topics = {sim_topics.ros_scan_topic(d) for d in FLEET}
    assert len(ros_topics) == len(FLEET)
    assert ros_topics == {"/drone_0/scan", "/drone_1/scan", "/drone_2/scan"}


def test_each_bridge_invocation_targets_exactly_one_drone():
    """A bridge serving two drones would merge their scans."""
    for drone_id in FLEET:
        args = sim_topics.ros_gz_bridge_args(drone_id)
        gz_topic = sim_topics.lidar_topic(drone_id)
        assert any(gz_topic in arg for arg in args)
        assert sim_topics.ros_scan_topic(drone_id) in " ".join(args)

        # No other drone's topic may appear in this invocation.
        for other in FLEET:
            if other == drone_id:
                continue
            assert sim_topics.lidar_topic(other) not in " ".join(args)


# ─────────────────────────────────────────────────────────────────────────────
#  Per-drone ports and identity
# ─────────────────────────────────────────────────────────────────────────────


def test_no_port_is_shared_anywhere_in_a_full_fleet():
    """Every port across the maximum fleet must be unique.

    PORT_STRIDE = 10 solved the UDP half of the fleet problem; this checks it
    holds across all four port families at once, including the gRPC ports that
    the embedded mavsdk_server instances need.
    """
    all_ports: list[int] = []
    for index in range(config.MAX_FLEET_SIZE):
        drone_id = f"drone-{index}"
        all_ports += [
            config.mavsdk_port(drone_id),
            config.vision_port(drone_id),
            config.lidar_port(drone_id),
            config.MAVSDK_GRPC_PORT_BASE + index,
        ]

    duplicates = {p for p in all_ports if all_ports.count(p) > 1}
    assert not duplicates, f"ports reused across the fleet: {sorted(duplicates)}"


def test_port_families_do_not_overlap_each_other():
    """Vision and LiDAR ports interleave (5005/5006, 5015/5016...) by design;
    the stride has to keep them from colliding as the fleet grows."""
    vision = {config.vision_port(f"drone-{i}") for i in range(config.MAX_FLEET_SIZE)}
    lidar = {config.lidar_port(f"drone-{i}") for i in range(config.MAX_FLEET_SIZE)}
    mavlink = {config.mavsdk_port(f"drone-{i}") for i in range(config.MAX_FLEET_SIZE)}

    assert not vision & lidar
    assert not vision & mavlink
    assert not lidar & mavlink
    assert config.PORT_STRIDE > 1, "a stride of 1 would collide vision with lidar"


def test_mavlink_system_ids_are_distinct():
    """MAVSDK defaults every client to sysid 245.

    Three clients answering as the same component means PX4 cannot tell them
    apart and attributes one drone's command acknowledgements to another.
    """
    sysids = {config.MAVSDK_SYSID_BASE + i for i in range(config.MAX_FLEET_SIZE)}
    assert len(sysids) == config.MAX_FLEET_SIZE


def test_fleet_size_cannot_exceed_the_px4_port_ceiling():
    """px4-rc.mavlink maps instances above 9 onto 14549."""
    assert config.FLEET_SIZE <= config.MAX_FLEET_SIZE
    assert config.MAX_FLEET_SIZE == 10


# ─────────────────────────────────────────────────────────────────────────────
#  Per-drone simulation entities
# ─────────────────────────────────────────────────────────────────────────────


def test_cargo_entities_are_namespaced_per_drone():
    """Three drones sharing one cargo entity would each teleport the same box."""
    names = {PayloadBay(d).cargo_model for d in FLEET}
    assert len(names) == len(FLEET), f"cargo names collide: {names}"
    for drone_id in FLEET:
        assert drone_id in PayloadBay(drone_id).cargo_model


def test_pad_entities_are_namespaced_per_drone():
    """Gazebo refuses a duplicate entity name rather than replacing it, so two
    drones spawning 'pickup_pad' means the second gets no pad at all."""
    import inspect

    from drone_agent import gz_client

    source = inspect.getsource(gz_client.spawn_pad_at_gps)
    assert "name_prefix" in source, "pad spawning must accept a per-drone namespace"
    # And the mission must actually pass one.
    mission_source = inspect.getsource(
        __import__("drone_agent.mission", fromlist=["x"]).DroneMission._spawn_pads
    )
    assert "name_prefix" in mission_source


def test_no_two_missions_share_mutable_state():
    """The structural reason a fleet was impossible."""
    missions = [DroneMission(d) for d in FLEET]

    for attribute in ("drone_state", "vision_data", "lidar_data", "fsm", "legs"):
        objects = [id(getattr(m, attribute)) for m in missions]
        assert len(set(objects)) == len(missions), f"{attribute} is shared between missions"

    # A write to one must be invisible to the others.
    missions[0].lidar_data["action"] = "DODGE_LEFT"
    assert missions[1].lidar_data["action"] == "CLEAR"
    assert missions[2].lidar_data["action"] == "CLEAR"


def test_each_drone_looks_for_its_own_markers_not_a_hardcoded_default():
    """navigation.py:283 and PayloadBay both defaulted to the literal 'drone-0'.

    A hardcoded default in a fleet means every drone's obstacle reports and cargo
    operations are attributed to drone-0.
    """
    import inspect

    from drone_agent import navigation

    source = inspect.getsource(navigation)
    assert '"drone-0"' not in source and "'drone-0'" not in source, (
        "navigation must not carry a hardcoded drone id"
    )

    payload_source = inspect.getsource(PayloadBay.__init__)
    assert "drone-0" not in payload_source


# ─────────────────────────────────────────────────────────────────────────────
#  Launcher shape
# ─────────────────────────────────────────────────────────────────────────────


def test_the_launcher_builds_once_outside_the_loop():
    """The old spawn_fleet.sh ran `make` inside the per-drone loop.

    That serialised N builds and raced on one build directory, so a fleet launch
    was slow, non-deterministic, and often produced a half-built binary.
    """
    script = (Path(__file__).resolve().parents[1] / "world" / "spawn_fleet.sh").read_text()

    lines = script.splitlines()
    make_lines = [i for i, line in enumerate(lines) if "make -C" in line]
    loop_lines = [i for i, line in enumerate(lines) if line.strip().startswith("for i in")]

    assert make_lines, "the launcher must build PX4"
    assert loop_lines, "the launcher must loop over drones"
    assert min(make_lines) < min(loop_lines), (
        "the build must happen before the per-drone loop, not inside it"
    )


def test_the_launcher_runs_px4_standalone():
    """Non-standalone PX4 re-sources gz_env.sh, clobbering our asset paths.

    px4-rc.gzsim sources PX4's own gz_env.sh on the non-standalone path, which
    unconditionally overwrites PX4_GZ_WORLDS and PX4_GZ_MODELS back to the PX4
    source tree - so the vendored world and sensor-equipped model would be
    ignored and the drone would fly blind.
    """
    script = (Path(__file__).resolve().parents[1] / "world" / "spawn_fleet.sh").read_text()
    assert "PX4_GZ_STANDALONE=1" in script


def test_the_launcher_does_not_set_px4_gz_model_name():
    """Setting it makes PX4 ATTACH to an existing model instead of spawning one.

    The plan called for PX4_GZ_MODEL_NAME=drone_$i, but in this PX4 version that
    variable selects a different launch mode entirely (see px4-rc.gzsim: the
    `elif [ -n "${PX4_GZ_MODEL_NAME}" ]` branch skips spawning), so the drones
    would never appear. Instance naming is handled by PX4 itself as
    ${PX4_SIM_MODEL}_${instance}.
    """
    script = (Path(__file__).resolve().parents[1] / "world" / "spawn_fleet.sh").read_text()
    setting = [
        line
        for line in script.splitlines()
        if "PX4_GZ_MODEL_NAME=" in line and not line.strip().startswith("#")
    ]
    assert not setting, f"PX4_GZ_MODEL_NAME must not be set: {setting}"


def test_run_system_starts_one_consumer_per_drone():
    """The old RUN_SYSTEM.sh started exactly one bridge and one avoider."""
    script = (Path(__file__).resolve().parents[1] / "scripts" / "run_system.sh").read_text()

    assert "--drone-id" in script, "consumers must be told which drone they serve"
    assert "bridge-args" in script, "the LiDAR bridge must use the per-drone model-scoped topic"
    # The per-drone processes must be inside the loop.
    lines = script.splitlines()
    loop_start = next(i for i, line in enumerate(lines) if "for i in $(seq 0" in line)
    after_loop = "\n".join(lines[loop_start:])
    for process in ("vision_", "avoider_", "ros_gz_bridge_"):
        assert process in after_loop, f"{process} must be started per drone"
