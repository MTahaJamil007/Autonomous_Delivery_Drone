"""P1 acceptance: the dispatch path either flies or reports why, and never wedges.

These are the plan's P1 acceptance criteria as executable tests. They run
against the real fleet_dispatch database layer with no PX4 and no simulator --
which is exactly the condition under which the original bug bit hardest, because
a dispatch that could not fly consumed a drone permanently and reported success.

Each test uses its own temporary database file, so they neither depend on nor
disturb the developer's fleet.db.
"""

import asyncio
import importlib
from pathlib import Path

import pytest

import config
from drone_agent.contracts import DroneStatus, JobStatus


@pytest.fixture
def fleet_db(tmp_path, monkeypatch):
    """A fleet database in a temp dir, seeded with a one-drone fleet."""
    from fleet_dispatch import db as fleet_db_module

    monkeypatch.setattr(fleet_db_module, "DB_PATH", tmp_path / "fleet.db")
    monkeypatch.setattr(config, "DRONE_IDS", ["drone-0"])
    asyncio.get_event_loop_policy()
    return fleet_db_module


async def test_atomic_claim_prevents_double_assignment(fleet_db):
    """P1.4: two concurrent dispatches for a one-drone fleet.

    Exactly one must win. The old code did create -> get_available -> assign
    across three separate connections, so both callers could read the same drone
    as AVAILABLE and both assign to it.
    """
    await fleet_db.init_database()

    job_a = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)
    job_b = await fleet_db.create_job(30.05, 72.33, 30.06, 72.34)

    claim_a, claim_b = await asyncio.gather(
        fleet_db.claim_drone_for_job(job_a.job_id),
        fleet_db.claim_drone_for_job(job_b.job_id),
    )

    winners = [c for c in (claim_a, claim_b) if c is not None]
    assert len(winners) == 1, f"exactly one claim must succeed for a 1-drone fleet, got {winners}"

    jobs = {j["id"]: j for j in await fleet_db.get_all_jobs()}
    statuses = sorted(j["status"] for j in jobs.values())
    assert statuses == [JobStatus.ASSIGNED.value, JobStatus.QUEUED.value], (
        f"one ASSIGNED and one QUEUED expected, got {statuses}"
    )


async def test_finalize_records_failure_and_frees_the_drone(fleet_db):
    """P1.3: a failed mission is recorded as FAILED with a reason.

    complete_job() hardcoded COMPLETED, so an exception on the mission path was
    indistinguishable in the database from a delivered parcel.
    """
    await fleet_db.init_database()
    job = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)
    drone_id = await fleet_db.claim_drone_for_job(job.job_id)
    assert drone_id is not None

    drone = await fleet_db.get_drone_status(drone_id)
    assert drone["status"] == DroneStatus.BUSY.value

    reason = "ConnectionError: no MAVLink heartbeat on udpin://0.0.0.0:14540"
    await fleet_db.finalize_job(job.job_id, JobStatus.FAILED, reason)

    finished = await fleet_db.get_job(job.job_id)
    assert finished["status"] == JobStatus.FAILED.value
    assert reason in finished["detail"], "the reason must be persisted, not just logged"

    drone = await fleet_db.get_drone_status(drone_id)
    assert drone["status"] == DroneStatus.AVAILABLE.value, "drone must be released"
    assert drone["current_job_id"] is None


async def test_finalize_rejects_a_non_terminal_status(fleet_db):
    """finalize_job() must not be usable to park a job in a running state."""
    await fleet_db.init_database()
    job = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)

    with pytest.raises(ValueError, match="terminal"):
        await fleet_db.finalize_job(job.job_id, JobStatus.IN_PROGRESS, "nope")


async def test_queued_job_is_retrievable_in_fifo_order(fleet_db):
    """P1.5: the drain step needs the oldest queued job, not an arbitrary one."""
    await fleet_db.init_database()

    first = await fleet_db.create_job(30.01, 72.01, 30.02, 72.02)
    # create_job stamps created_at from the clock; ensure a distinct timestamp.
    await asyncio.sleep(0.01)
    await fleet_db.create_job(30.03, 72.03, 30.04, 72.04)

    # Occupy the only drone.
    held = await fleet_db.claim_drone_for_job(first.job_id)
    assert held is not None

    queued = await fleet_db.next_queued_job()
    assert queued is not None
    assert queued.job_id != first.job_id, "the assigned job must not come back"

    await fleet_db.finalize_job(first.job_id, JobStatus.COMPLETED, "done")
    claimed = await fleet_db.claim_drone_for_job(queued.job_id)
    assert claimed is not None, "the freed drone must be claimable by the queued job"


async def test_no_drone_is_stranded_by_repeated_failures(fleet_db):
    """P1 acceptance: 50 rapid dispatch/fail cycles leave no drone stuck BUSY.

    This is the failure mode the whole phase exists to prevent. Before, each
    failed dispatch consumed a drone permanently, so the fleet silently shrank
    to zero and every subsequent dispatch was silently queued forever.
    """
    await fleet_db.init_database()

    for i in range(50):
        job = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)
        drone_id = await fleet_db.claim_drone_for_job(job.job_id)
        assert drone_id is not None, f"fleet exhausted after {i} failed dispatches"
        await fleet_db.finalize_job(job.job_id, JobStatus.FAILED, f"simulated failure {i}")

    drones = await fleet_db.get_all_drones()
    stuck = [d for d in drones if d["status"] != DroneStatus.AVAILABLE.value]
    assert not stuck, f"drones left non-AVAILABLE: {stuck}"

    failed = await fleet_db.get_all_jobs(JobStatus.FAILED.value)
    assert len(failed) == 50
    assert all(j["detail"] for j in failed), "every failure must carry a reason"


async def test_startup_recovers_jobs_orphaned_by_a_crash(fleet_db):
    """A process kill mid-mission must not permanently consume a drone.

    There is no live task left to reach a finally: block after a crash, so
    without startup recovery every restart would shrink the usable fleet by one.
    """
    await fleet_db.init_database()
    job = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)
    drone_id = await fleet_db.claim_drone_for_job(job.job_id)
    await fleet_db.mark_job_in_progress(job.job_id)

    # Simulate a crash: nothing finalises the job.
    recovered = await fleet_db.recover_orphaned_jobs()
    assert recovered == 1

    orphan = await fleet_db.get_job(job.job_id)
    assert orphan["status"] == JobStatus.ABORTED.value, (
        "an interrupted in-flight job is ABORTED, not FAILED: an operator needs "
        "to know whether there is an airframe to go and find"
    )

    drone = await fleet_db.get_drone_status(drone_id)
    assert drone["status"] == DroneStatus.AVAILABLE.value


async def test_job_ids_are_unique_under_rapid_creation(fleet_db):
    """The old md5(timestamp)[:12] collided inside one microsecond."""
    await fleet_db.init_database()
    jobs = [await fleet_db.create_job(30.0, 72.0, 30.1, 72.1) for _ in range(200)]
    ids = {j.job_id for j in jobs}
    assert len(ids) == 200, f"collision: {200 - len(ids)} duplicate job id(s)"


async def test_telemetry_writer_cannot_change_status(fleet_db):
    """P1.6: the heartbeat path must not race status transitions.

    The old update_drone_status() could write the status column, so a heartbeat
    landing just after a job finalised would write BUSY back over AVAILABLE and
    strand the drone.
    """
    await fleet_db.init_database()
    job = await fleet_db.create_job(30.03, 72.31, 30.04, 72.32)
    drone_id = await fleet_db.claim_drone_for_job(job.job_id)
    await fleet_db.finalize_job(job.job_id, JobStatus.COMPLETED, "done")

    # A late heartbeat, arriving after the job finished.
    await fleet_db.update_drone_telemetry(
        drone_id, lat=30.1, lon=72.1, alt=5.0, detail="stale heartbeat"
    )

    drone = await fleet_db.get_drone_status(drone_id)
    assert drone["status"] == DroneStatus.AVAILABLE.value, (
        "a telemetry write must never resurrect BUSY"
    )
    assert drone["lat"] == pytest.approx(30.1)


def test_timestamps_are_timezone_aware():
    """P1.6: datetime.utcnow() returned naive datetimes and is deprecated."""
    from datetime import datetime

    from drone_agent.contracts import utc_now_iso

    parsed = datetime.fromisoformat(utc_now_iso())
    assert parsed.tzinfo is not None, "timestamps must carry an explicit offset"
    assert parsed.utcoffset().total_seconds() == 0
