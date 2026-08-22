"""Fleet Dispatch Center: job intake, drone assignment, live telemetry.

THE BUG THIS FILE EXISTED TO CAUSE
----------------------------------
`run_drone_task` used to begin with

    from drone_web.drone_logic import get_state_snapshot

placed INSIDE the function but ABOVE its `try:`. `get_state_snapshot` did not
exist. So every dispatch raised ImportError before entering the try block, the
`finally: complete_job(...)` never ran, and the drone stayed BUSY forever. A
fleet of three shrank to zero after three dispatches, each of which the UI had
reported as successfully assigned.

Two changes stop that whole class of failure:

1. Both imports are at module scope. A missing symbol is now an immediate crash
   at startup, where it is unmissable, instead of a per-request failure that
   silently consumes a drone.

2. The `try:` opens on the first line of the task body, so nothing can run
   before it and skip the cleanup.

The import fallback that used to define a stub `execute_delivery` is also gone.
It logged critically and returned successfully, so a build that could not fly
reported every mission as COMPLETED. A dispatcher that cannot fly must fail
loudly, not succeed quietly.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel, Field

import config
from drone_agent.contracts import DeliveryJob, JobStatus, MissionResult

# MODULE SCOPE, NO try/except. If these cannot be imported, this process must
# not start: a dispatcher that cannot fly is worse than no dispatcher, because
# it accepts jobs and reports them as assigned.
from drone_web.drone_logic import execute_delivery, get_state_snapshot
from fleet_dispatch import db as fleet_db

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger("fleet_dispatch")


HEARTBEAT_INTERVAL_S = 2.0
WEBSOCKET_INTERVAL_S = 0.5

# Live mission tasks, so a shutdown can cancel them and a status query can tell
# "flying" from "wedged". Holding the reference is not optional: an unreferenced
# asyncio task can be garbage-collected mid-flight, taking its exception with it.
_missions: dict[str, asyncio.Task] = {}


class DispatchRequest(BaseModel):
    pickup_lat: float = Field(..., ge=-90, le=90)
    pickup_lon: float = Field(..., ge=-180, le=180)
    drop_lat: float = Field(..., ge=-90, le=90)
    drop_lon: float = Field(..., ge=-180, le=180)


class DispatchResponse(BaseModel):
    status: str  # Success | Queued | Error
    message: str
    job_id: str | None = None
    drone_id: str | None = None


# ─────────────────────────────────────────────────────────────────────────────
#  Lifecycle
# ─────────────────────────────────────────────────────────────────────────────


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """Startup and shutdown, as one span.

    A lifespan handler rather than the deprecated on_event pair, and the two
    halves belong together anyway: startup recovers jobs a previous run
    orphaned, and shutdown is what stops this run orphaning any.
    """
    await fleet_db.init_database()
    recovered = await fleet_db.recover_orphaned_jobs()
    if recovered:
        logger.warning(
            "released %d job(s) left in flight by a previous run; their drones are AVAILABLE again",
            recovered,
        )
    logger.info(
        "dispatch ready: fleet=%d, port=%d",
        config.FLEET_SIZE,
        config.FLEET_DISPATCH_PORT,
    )

    yield

    # Abort live missions rather than orphaning them. A cancelled mission still
    # passes through run_mission's finally: block, so the job is recorded
    # ABORTED and the drone is freed - without this the next startup would have
    # to recover them, and an operator would see a fleet that shrank over a
    # restart.
    for job_id, task in list(_missions.items()):
        logger.warning("shutdown: cancelling mission for job %s", job_id)
        task.cancel()
    for task in list(_missions.values()):
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await task


app = FastAPI(
    title="Fleet Dispatch Center",
    description="Multi-drone delivery coordination and monitoring",
    version="3.0.0",
    lifespan=lifespan,
)

# Absolute path: the previous Jinja2Templates(directory="templates") resolved
# relative to the process's cwd, so the server only rendered when launched from
# inside fleet_dispatch/.
templates = Jinja2Templates(directory=str(Path(__file__).parent / "templates"))


# ─────────────────────────────────────────────────────────────────────────────
#  Mission execution
# ─────────────────────────────────────────────────────────────────────────────


async def _heartbeat_writer(drone_id: str) -> None:
    """Mirror a flying drone's telemetry into the database for the UI.

    Writes telemetry only, never status: a heartbeat landing just after a job
    finalises would otherwise write BUSY back over AVAILABLE and strand the
    drone. See db.update_drone_telemetry.
    """
    while True:
        try:
            await asyncio.sleep(HEARTBEAT_INTERVAL_S)
            snapshot = get_state_snapshot(drone_id)
            await fleet_db.update_drone_telemetry(
                drone_id,
                lat=snapshot.get("lat"),
                lon=snapshot.get("lon"),
                alt=snapshot.get("alt"),
                battery_pct=snapshot.get("battery_pct"),
                detail=snapshot.get("status"),
            )
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - a telemetry write must not end a flight
            logger.warning("heartbeat write failed for %s: %s", drone_id, exc)


async def run_mission(job: DeliveryJob, drone_id: str) -> None:
    """Fly one job to a terminal state, then always release the drone.

    The `try:` is the first statement in the body on purpose. Anything executed
    before it -- an import, a lookup, a log call -- can raise and skip the
    `finally:`, which is precisely how a failed dispatch used to wedge a drone
    as BUSY permanently.
    """
    heartbeat: asyncio.Task | None = None
    result: MissionResult

    try:
        heartbeat = asyncio.create_task(_heartbeat_writer(drone_id), name=f"heartbeat-{drone_id}")
        await fleet_db.mark_job_in_progress(job.job_id)

        logger.info("job %s: mission starting on %s", job.job_id, drone_id)
        result = await execute_delivery(job, drone_id)
        logger.info(
            "job %s: mission returned %s (%s)",
            job.job_id,
            result.status.value,
            result.detail,
        )

    except asyncio.CancelledError:
        # Cancellation means an operator or a shutdown stopped a mission that
        # was airborne: ABORTED, not FAILED. The distinction tells whoever reads
        # this whether there is an airframe to go and find.
        result = MissionResult.aborted("Mission cancelled while in flight.")
        raise

    except Exception as exc:  # noqa: BLE001 - every failure must reach the database
        # The reason is captured verbatim. "FAILED" with no explanation is what
        # made the previous behaviour so hard to debug.
        result = MissionResult.failed(f"{type(exc).__name__}: {exc}")
        logger.error("job %s failed on %s", job.job_id, drone_id, exc_info=True)

    finally:
        if heartbeat is not None:
            heartbeat.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await heartbeat

        # `result` is always bound: every path above assigns it before arriving
        # here, including the CancelledError path (which assigns, then re-raises
        # — the assignment happens first).
        await fleet_db.finalize_job(job.job_id, result.status, result.detail)
        _missions.pop(job.job_id, None)

        # P1.5: drain the queue. Nothing else starts a QUEUED job, so without
        # this the documented queueing behaviour is a one-way write.
        await _drain_queue()


async def _drain_queue() -> None:
    """Start queued jobs while drones are free.

    Loops rather than starting one, so freeing a drone after a burst of
    dispatches picks up work immediately. Bounded by the queue emptying or by
    no drone being claimable, both of which terminate.
    """
    while True:
        job = await fleet_db.next_queued_job()
        if job is None:
            return

        drone_id = await fleet_db.claim_drone_for_job(job.job_id)
        if drone_id is None:
            return  # fleet is fully committed; try again on next finalize

        logger.info("queue drain: job %s -> %s", job.job_id, drone_id)
        _missions[job.job_id] = asyncio.create_task(
            run_mission(job, drone_id), name=f"mission-{job.job_id}"
        )


# ─────────────────────────────────────────────────────────────────────────────
#  HTTP
# ─────────────────────────────────────────────────────────────────────────────


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(
        request=request, name="index.html", context={"fleet_size": config.FLEET_SIZE}
    )


@app.post("/dispatch", response_model=DispatchResponse)
async def dispatch(request: DispatchRequest) -> DispatchResponse:
    """Create a job and start it if a drone is free, otherwise queue it.

    Uses asyncio.create_task rather than FastAPI's BackgroundTasks: a background
    task runs only after the response is sent and cannot be cancelled, so a
    shutdown mid-mission would orphan it. Missions here are tracked in
    `_missions` and cancelled cleanly.
    """
    try:
        job = await fleet_db.create_job(
            request.pickup_lat,
            request.pickup_lon,
            request.drop_lat,
            request.drop_lon,
        )

        drone_id = await fleet_db.claim_drone_for_job(job.job_id)

        if drone_id is None:
            return DispatchResponse(
                status="Queued",
                message="All drones busy - job queued, will start automatically.",
                job_id=job.job_id,
            )

        _missions[job.job_id] = asyncio.create_task(
            run_mission(job, drone_id), name=f"mission-{job.job_id}"
        )
        return DispatchResponse(
            status="Success",
            message=f"Job {job.job_id} assigned to {drone_id}",
            job_id=job.job_id,
            drone_id=drone_id,
        )

    except Exception as exc:  # noqa: BLE001
        logger.error("dispatch failed", exc_info=True)
        return DispatchResponse(status="Error", message=f"{type(exc).__name__}: {exc}")


@app.get("/fleet/status")
async def fleet_status() -> dict:
    drones = await fleet_db.get_all_drones()
    for drone in drones:
        # Distinguishes "flying" from "marked BUSY but nothing is running".
        drone["mission_live"] = drone.get("current_job_id") in _missions
    return {"fleet_size": len(drones), "drones": drones}


@app.get("/jobs")
async def get_jobs(status: str | None = None) -> dict:
    jobs = await fleet_db.get_all_jobs(status)
    return {"count": len(jobs), "jobs": jobs}


@app.get("/jobs/{job_id}")
async def get_job(job_id: str) -> dict:
    job = await fleet_db.get_job(job_id)
    if job is None:
        return {"error": "Job not found", "job_id": job_id}
    return job


@app.post("/jobs/{job_id}/abort")
async def abort_job(job_id: str) -> dict:
    """Cancel a live mission. The mission's own finally: releases the drone."""
    task = _missions.get(job_id)
    if task is None:
        return {"status": "Error", "message": f"No live mission for job {job_id}"}
    task.cancel()
    return {"status": "Aborting", "job_id": job_id}


# ─────────────────────────────────────────────────────────────────────────────
#  WebSockets
# ─────────────────────────────────────────────────────────────────────────────


@app.websocket("/ws/{drone_id}")
async def websocket_drone_telemetry(websocket: WebSocket, drone_id: str) -> None:
    """Per-drone telemetry channel: /ws/drone-0, /ws/drone-1, ...

    The fleet view opens one of these per drone. Reads from the database rather
    than the in-process registry so the browser sees the same values the
    dispatcher recorded, even for a drone whose mission lives elsewhere.
    """
    await websocket.accept()
    try:
        while True:
            status = await fleet_db.get_drone_status(drone_id)
            if status is None:
                await websocket.send_json(
                    {
                        "drone_id": drone_id,
                        "status": "OFFLINE",
                        "detail": "not in registry",
                    }
                )
            else:
                status["mission_live"] = status.get("current_job_id") in _missions
                await websocket.send_json(status)
            await asyncio.sleep(WEBSOCKET_INTERVAL_S)
    except WebSocketDisconnect:
        logger.debug("websocket closed: %s", drone_id)


@app.websocket("/ws")
async def websocket_fleet(websocket: WebSocket) -> None:
    """Whole-fleet channel, including each job's terminal reason.

    Replaces the old single-drone endpoint that streamed a module-level
    `drone_state` dict -- the same globals that limited the system to one drone.
    """
    await websocket.accept()
    try:
        while True:
            drones = await fleet_db.get_all_drones()
            for drone in drones:
                drone["mission_live"] = drone.get("current_job_id") in _missions
            recent = await fleet_db.get_all_jobs(limit=10)
            await websocket.send_json(
                {
                    "drones": drones,
                    "recent_jobs": recent,
                    "queued": sum(1 for j in recent if j["status"] == JobStatus.QUEUED.value),
                }
            )
            await asyncio.sleep(WEBSOCKET_INTERVAL_S)
    except WebSocketDisconnect:
        logger.debug("fleet websocket closed")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=config.FLEET_DISPATCH_PORT, log_level="info")
