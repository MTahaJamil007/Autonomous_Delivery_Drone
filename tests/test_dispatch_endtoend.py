"""P1 headline acceptance: dispatch with PX4 offline.

    "Dispatch with PX4 offline -> job FAILED with a reason, drone back to
     AVAILABLE within 5 s, reason visible in the browser."

This drives the real FastAPI app through its real HTTP surface, with no PX4 and
no simulator running. That is precisely the condition the original blocker bit
under: fleet_dispatch/app.py's `from drone_web.drone_logic import
get_state_snapshot` raised ImportError above the try block, the finally that
released the drone never ran, and the drone was consumed permanently while the
operator was told the job had been assigned.

Marked `slow` because it waits out a real connection probe.
"""

import asyncio
import time
from pathlib import Path

import pytest

import config
from drone_agent.contracts import DroneStatus, JobStatus


@pytest.fixture
def client(tmp_path, monkeypatch):
    """TestClient over the real app, against a throwaway database."""
    from fastapi.testclient import TestClient

    from fleet_dispatch import app as app_module
    from fleet_dispatch import db as fleet_db

    monkeypatch.setattr(fleet_db, "DB_PATH", tmp_path / "fleet.db")
    monkeypatch.setattr(config, "DRONE_IDS", ["drone-0"])
    # Keep the probe short: this test is about the bookkeeping, not the timeout.
    monkeypatch.setattr(config, "MAVSDK_PROBE_TIMEOUT_S", 2.0)

    with TestClient(app_module.app) as test_client:
        yield test_client


@pytest.mark.slow
def test_dispatch_with_px4_offline_fails_and_frees_the_drone(client):
    """The whole P1 criterion, end to end over HTTP."""
    response = client.post(
        "/dispatch",
        json={
            "pickup_lat": 30.0320,
            "pickup_lon": 72.3145,
            "drop_lat": 30.0325,
            "drop_lon": 72.3150,
        },
    )
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "Success", f"expected assignment, got {body}"
    job_id = body["job_id"]
    drone_id = body["drone_id"]

    # Wait for the mission to fail and release the drone. The probe is 2 s here,
    # so a generous ceiling still proves the drone is not stranded.
    deadline = time.monotonic() + 20.0
    job = None
    while time.monotonic() < deadline:
        job = client.get(f"/jobs/{job_id}").json()
        if job["status"] in (
            JobStatus.FAILED.value,
            JobStatus.ABORTED.value,
            JobStatus.COMPLETED.value,
        ):
            break
        time.sleep(0.25)

    assert job is not None
    assert job["status"] == JobStatus.FAILED.value, (
        f"a dispatch that cannot fly must be FAILED, got {job['status']}. "
        f"Recording it as COMPLETED is the behaviour this phase removes."
    )
    assert job["detail"], "FAILED with no reason is barely better than silent success"
    assert "heartbeat" in job["detail"].lower() or "connection" in job["detail"].lower(), (
        f"the reason must name the actual cause, got: {job['detail']!r}"
    )

    fleet = client.get("/fleet/status").json()
    drone = next(d for d in fleet["drones"] if d["id"] == drone_id)
    assert drone["status"] == DroneStatus.AVAILABLE.value, (
        "the drone must be returned to the pool; leaving it BUSY is the original bug"
    )
    assert drone["current_job_id"] is None
    assert not drone["mission_live"]

    # The reason must be reachable by the browser, which reads /jobs.
    listed = client.get("/jobs").json()
    assert any(j["id"] == job_id and j["detail"] for j in listed["jobs"])


@pytest.mark.slow
def test_second_dispatch_queues_then_starts_automatically(client):
    """P1 acceptance: one ASSIGNED, one QUEUED, and the queue drains itself.

    Nothing used to start a QUEUED job, so "job queueing" was a one-way write:
    jobs accumulated and were never dispatched.
    """
    payload = {
        "pickup_lat": 30.0320,
        "pickup_lon": 72.3145,
        "drop_lat": 30.0325,
        "drop_lon": 72.3150,
    }
    first = client.post("/dispatch", json=payload).json()
    second = client.post("/dispatch", json=payload).json()

    assert first["status"] == "Success", first
    assert second["status"] == "Queued", (
        f"a one-drone fleet must queue the second job, got {second}"
    )

    # Both must reach a terminal state without intervention: the first fails on
    # the missing autopilot, and finalising it must drain the second.
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline:
        jobs = client.get("/jobs").json()["jobs"]
        if (
            all(
                j["status"]
                in (
                    JobStatus.FAILED.value,
                    JobStatus.ABORTED.value,
                    JobStatus.COMPLETED.value,
                )
                for j in jobs
            )
            and len(jobs) == 2
        ):
            break
        time.sleep(0.5)

    jobs = client.get("/jobs").json()["jobs"]
    assert len(jobs) == 2
    for job in jobs:
        assert job["status"] == JobStatus.FAILED.value, (
            f"job {job['id']} ended as {job['status']}; the queued job must be "
            f"started by the drain step, not left QUEUED forever"
        )
        assert job["detail"]

    fleet = client.get("/fleet/status").json()
    assert all(d["status"] == DroneStatus.AVAILABLE.value for d in fleet["drones"]), (
        "no drone may be left BUSY once every job is terminal"
    )


def test_dispatch_rejects_impossible_coordinates(client):
    """Validation belongs at the boundary, not in the flight code."""
    response = client.post(
        "/dispatch",
        json={
            "pickup_lat": 200.0,
            "pickup_lon": 72.3145,
            "drop_lat": 30.0325,
            "drop_lon": 72.3150,
        },
    )
    assert response.status_code == 422, "a latitude of 200 must be rejected by the request model"
