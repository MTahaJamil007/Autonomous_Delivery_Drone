"""Fleet dispatch persistence: drone registry and job queue.

Three defects in the previous version, each of which produced a fleet that
looked healthy and was not:

1. `dispatch()` did create-job, then get-available-drone, then assign-job over
   three separate connections. Two concurrent requests could both read the same
   drone as AVAILABLE and both assign to it. `claim_drone_for_job()` replaces
   that with a single conditional UPDATE whose rowcount tells the caller whether
   it won the race.

2. `complete_job()` wrote status='COMPLETED' unconditionally. Every outcome,
   including an exception, was recorded as a success. `finalize_job()` takes the
   status and a reason.

3. Nothing ever started a QUEUED job. "Job queueing" was a one-way write: jobs
   accumulated and were never dispatched. `next_queued_job()` plus the
   dispatcher's drain step closes that loop.

Also: `datetime.utcnow()` (4 sites) returned naive datetimes and is deprecated;
all timestamps now come from contracts.utc_now_iso().
"""

from __future__ import annotations

import logging
import secrets
from pathlib import Path
from typing import Any

import aiosqlite

import config
from drone_agent.contracts import DeliveryJob, DroneStatus, JobStatus, utc_now_iso

logger = logging.getLogger(__name__)

DB_PATH = Path(__file__).parent / "fleet.db"
SCHEMA_PATH = Path(__file__).parent / "schema.sql"


def generate_job_id() -> str:
    """Random 12-hex-character job id.

    Was md5(timestamp)[:12]. Two jobs created inside the same microsecond
    produced the same id, and the id then collided on a PRIMARY KEY insert --
    or worse, silently referred to the wrong job. `secrets.token_hex` has no
    such dependence on clock resolution.
    """
    return secrets.token_hex(6)


def _connect() -> aiosqlite.Connection:
    """Open a connection configured for concurrent access.

    WAL lets readers proceed while a writer holds the database, which is what
    the WebSocket telemetry pollers need in order not to block a mission's
    heartbeat writes. `busy_timeout` makes a contended write wait rather than
    raising SQLITE_BUSY straight back to the caller.
    """
    return aiosqlite.connect(DB_PATH, timeout=10.0)


async def _configure(db: aiosqlite.Connection) -> None:
    await db.execute("PRAGMA journal_mode=WAL")
    await db.execute("PRAGMA busy_timeout=10000")
    await db.execute("PRAGMA foreign_keys=ON")


# Columns added to schema.sql after the first release, as
# {table: {column: full column definition}}.
#
# WHY THIS TABLE EXISTS
# ---------------------
# schema.sql is all CREATE TABLE IF NOT EXISTS, which is a no-op against a
# database whose tables already exist -- it can create a schema but it can never
# widen one. So a fleet.db written before `detail` was added kept its old shape,
# init_database() logged "fleet database ready", and the very next startup step
# (recover_orphaned_jobs) died on `no such column: detail`, taking the whole
# dispatcher down with it. The web UI simply refused to connect, several
# processes away from the actual cause.
#
# Any future additive column must be listed here as well as in schema.sql.
ADDED_COLUMNS: dict[str, dict[str, str]] = {
    "drones": {"detail": "TEXT DEFAULT ''"},
    "jobs": {"detail": "TEXT DEFAULT ''"},
}


async def _migrate(db: aiosqlite.Connection) -> list[str]:
    """Add any column in ADDED_COLUMNS that this database is missing.

    Idempotent: on an up-to-date database it reads two PRAGMAs and does nothing.
    Only additive ALTERs live here -- anything needing a table rewrite is a
    different problem and should not be hidden inside startup.
    """
    applied: list[str] = []

    for table, columns in ADDED_COLUMNS.items():
        cursor = await db.execute(f"PRAGMA table_info({table})")
        existing = {row[1] for row in await cursor.fetchall()}
        if not existing:
            continue  # table was just created from schema.sql; already current

        for column, definition in columns.items():
            if column not in existing:
                await db.execute(f"ALTER TABLE {table} ADD COLUMN {column} {definition}")
                applied.append(f"{table}.{column}")

    return applied


async def init_database() -> None:
    """Create the schema, migrate an older one, and register the configured fleet."""
    async with _connect() as db:
        await _configure(db)
        await db.executescript(SCHEMA_PATH.read_text())

        applied = await _migrate(db)
        if applied:
            logger.warning("migrated fleet database: added %s", ", ".join(applied))

        now = utc_now_iso()
        for drone_id in config.DRONE_IDS:
            await db.execute(
                "INSERT OR IGNORE INTO drones (id, status, last_heartbeat) VALUES (?, ?, ?)",
                (drone_id, DroneStatus.AVAILABLE.value, now),
            )
        await db.commit()

    logger.info("fleet database ready at %s (%d drones)", DB_PATH, len(config.DRONE_IDS))


# ─────────────────────────────────────────────────────────────────────────────
#  Drones
# ─────────────────────────────────────────────────────────────────────────────


async def update_drone_telemetry(
    drone_id: str,
    *,
    battery_pct: float | None = None,
    lat: float | None = None,
    lon: float | None = None,
    alt: float | None = None,
    detail: str | None = None,
) -> None:
    """Refresh a drone's live telemetry and heartbeat.

    Deliberately CANNOT write the `status` column. The previous
    update_drone_status() could, and the 2 Hz heartbeat task raced
    assign/complete transitions with it -- a heartbeat that landed just after a
    job finished would write BUSY back over AVAILABLE and strand the drone.
    Status transitions go only through claim_drone_for_job(), finalize_job()
    and set_drone_status().
    """
    fields = ["last_heartbeat = ?"]
    values: list[Any] = [utc_now_iso()]

    for column, value in (
        ("battery_pct", battery_pct),
        ("lat", lat),
        ("lon", lon),
        ("alt", alt),
        ("detail", detail),
    ):
        if value is not None:
            fields.append(f"{column} = ?")
            values.append(value)

    values.append(drone_id)
    async with _connect() as db:
        await _configure(db)
        await db.execute(f"UPDATE drones SET {', '.join(fields)} WHERE id = ?", values)
        await db.commit()


async def set_drone_status(drone_id: str, status: DroneStatus, detail: str = "") -> None:
    """Force a drone's status. For operator intervention and OFFLINE marking."""
    async with _connect() as db:
        await _configure(db)
        await db.execute(
            "UPDATE drones SET status = ?, detail = ?, last_heartbeat = ? WHERE id = ?",
            (status.value, detail, utc_now_iso(), drone_id),
        )
        await db.commit()
    logger.info("drone %s -> %s (%s)", drone_id, status.value, detail or "no detail")


async def get_drone_status(drone_id: str) -> dict[str, Any] | None:
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM drones WHERE id = ?", (drone_id,))
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_all_drones() -> list[dict[str, Any]]:
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM drones ORDER BY id")
        return [dict(row) for row in await cursor.fetchall()]


# ─────────────────────────────────────────────────────────────────────────────
#  Jobs
# ─────────────────────────────────────────────────────────────────────────────


async def create_job(
    pickup_lat: float, pickup_lon: float, drop_lat: float, drop_lon: float
) -> DeliveryJob:
    """Insert a QUEUED job and return it as the shared contract type.

    Returns a DeliveryJob rather than a bare id so the caller cannot reassemble
    the coordinates in the wrong order on the way to execute_delivery().
    """
    job = DeliveryJob(generate_job_id(), pickup_lat, pickup_lon, drop_lat, drop_lon)

    async with _connect() as db:
        await _configure(db)
        await db.execute(
            "INSERT INTO jobs (id, pickup_lat, pickup_lon, drop_lat, drop_lon, "
            "status, created_at) VALUES (?, ?, ?, ?, ?, ?, ?)",
            (
                job.job_id,
                job.pickup_lat,
                job.pickup_lon,
                job.drop_lat,
                job.drop_lon,
                JobStatus.QUEUED.value,
                utc_now_iso(),
            ),
        )
        await db.commit()

    logger.info("job %s queued", job.job_id)
    return job


async def claim_drone_for_job(job_id: str) -> str | None:
    """Atomically claim one AVAILABLE drone for a job. Returns its id, or None.

    THE RACE THIS CLOSES
    --------------------
    The conditional UPDATE plus the rowcount check is the whole point. Two
    concurrent dispatches can both SELECT the same drone as AVAILABLE, but only
    one UPDATE can match `status='AVAILABLE'` -- SQLite serialises writers, so
    the loser sees rowcount 0 and moves to the next candidate. Splitting this
    into a SELECT then an unconditional UPDATE, over two connections, is what
    allowed two jobs to be assigned to one drone.

    Everything happens on ONE connection inside ONE transaction, so a crash
    between claiming the drone and marking the job cannot leave a drone BUSY
    with no job pointing at it.
    """
    async with _connect() as db:
        await _configure(db)

        # Least-recently-active first: spreads work rather than always picking
        # drone-0 and leaving the rest of the fleet idle.
        cursor = await db.execute(
            "SELECT id FROM drones WHERE status = ? ORDER BY last_heartbeat ASC",
            (DroneStatus.AVAILABLE.value,),
        )
        candidates = [row[0] for row in await cursor.fetchall()]

        for drone_id in candidates:
            claim = await db.execute(
                "UPDATE drones SET status = ?, current_job_id = ?, detail = ? "
                "WHERE id = ? AND status = ?",
                (DroneStatus.BUSY.value, job_id, "assigned", drone_id, DroneStatus.AVAILABLE.value),
            )
            if claim.rowcount == 0:
                # Lost the race for this drone; try the next candidate.
                logger.debug("lost claim race for %s, trying next", drone_id)
                continue

            assign = await db.execute(
                "UPDATE jobs SET assigned_drone = ?, status = ?, started_at = ? "
                "WHERE id = ? AND status = ?",
                (drone_id, JobStatus.ASSIGNED.value, utc_now_iso(), job_id, JobStatus.QUEUED.value),
            )
            if assign.rowcount == 0:
                # The job was claimed by someone else between our read and now.
                # Roll the drone back so it does not leak as permanently BUSY.
                await db.rollback()
                logger.warning("job %s was no longer QUEUED; released drone %s", job_id, drone_id)
                return None

            await db.commit()
            logger.info("job %s -> drone %s", job_id, drone_id)
            return drone_id

        return None


async def mark_job_in_progress(job_id: str) -> None:
    """ASSIGNED -> IN_PROGRESS, once the mission task actually starts flying."""
    async with _connect() as db:
        await _configure(db)
        await db.execute(
            "UPDATE jobs SET status = ? WHERE id = ? AND status = ?",
            (JobStatus.IN_PROGRESS.value, job_id, JobStatus.ASSIGNED.value),
        )
        await db.commit()


async def finalize_job(job_id: str, status: JobStatus, detail: str = "") -> str | None:
    """Close a job with an honest status and free its drone. Returns the drone id.

    Replaces complete_job(), which hardcoded COMPLETED, so a mission that raised
    an exception was indistinguishable in the database from one that delivered
    the parcel.

    Freeing the drone is unconditional and happens in the same transaction as
    the job update. That is what stops the failure mode this plan was written
    for: an exception on the mission path leaving a drone BUSY forever with no
    live mission behind it, so the fleet silently shrinks with each failure.
    """
    if not status.is_terminal:
        raise ValueError(f"finalize_job() needs a terminal status, got {status.value}")

    async with _connect() as db:
        await _configure(db)

        cursor = await db.execute("SELECT assigned_drone FROM jobs WHERE id = ?", (job_id,))
        row = await cursor.fetchone()
        if row is None:
            logger.error("finalize_job: no such job %s", job_id)
            return None
        drone_id = row[0]

        await db.execute(
            "UPDATE jobs SET status = ?, detail = ?, completed_at = ? WHERE id = ?",
            (status.value, detail, utc_now_iso(), job_id),
        )

        if drone_id:
            await db.execute(
                "UPDATE drones SET status = ?, current_job_id = NULL, detail = ? WHERE id = ?",
                (DroneStatus.AVAILABLE.value, detail[:200], drone_id),
            )

        await db.commit()

    logger.info(
        "job %s -> %s (%s); drone %s released",
        job_id,
        status.value,
        detail or "no detail",
        drone_id,
    )
    return drone_id


async def next_queued_job() -> DeliveryJob | None:
    """Oldest QUEUED job, or None.

    The queue drain reads this after every finalize_job(). Without it the
    documented "job queueing" behaviour was write-only: jobs went in and nothing
    ever took them out.
    """
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(
            "SELECT * FROM jobs WHERE status = ? ORDER BY created_at ASC LIMIT 1",
            (JobStatus.QUEUED.value,),
        )
        row = await cursor.fetchone()
        if row is None:
            return None
        return DeliveryJob(
            row["id"],
            row["pickup_lat"],
            row["pickup_lon"],
            row["drop_lat"],
            row["drop_lon"],
        )


async def get_job(job_id: str) -> dict[str, Any] | None:
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM jobs WHERE id = ?", (job_id,))
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_all_jobs(status: str | None = None, limit: int = 200) -> list[dict[str, Any]]:
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        if status:
            cursor = await db.execute(
                "SELECT * FROM jobs WHERE status = ? ORDER BY created_at DESC LIMIT ?",
                (status, limit),
            )
        else:
            cursor = await db.execute(
                "SELECT * FROM jobs ORDER BY created_at DESC LIMIT ?", (limit,)
            )
        return [dict(row) for row in await cursor.fetchall()]


async def recover_orphaned_jobs() -> int:
    """On startup, fail any job left mid-flight by a crash and free its drone.

    Without this, a process kill during a mission leaves the job IN_PROGRESS and
    the drone BUSY forever -- there is no live task to reach the finally: block,
    so nothing will ever release it. Every restart would shrink the usable fleet
    by one until someone ran reset_fleet_database.py by hand.
    """
    stale = (JobStatus.ASSIGNED.value, JobStatus.IN_PROGRESS.value)
    detail = "Interrupted: the dispatcher restarted while this job was in flight."

    async with _connect() as db:
        await _configure(db)
        cursor = await db.execute(
            f"SELECT id FROM jobs WHERE status IN ({','.join('?' * len(stale))})", stale
        )
        job_ids = [row[0] for row in await cursor.fetchall()]

        if job_ids:
            placeholders = ",".join("?" * len(job_ids))
            await db.execute(
                f"UPDATE jobs SET status = ?, detail = ?, completed_at = ? "
                f"WHERE id IN ({placeholders})",
                (JobStatus.ABORTED.value, detail, utc_now_iso(), *job_ids),
            )

        # Free every drone regardless: a drone marked BUSY at startup cannot
        # have a live mission behind it, because missions do not survive the
        # process that runs them.
        await db.execute(
            "UPDATE drones SET status = ?, current_job_id = NULL, detail = ? WHERE status = ?",
            (DroneStatus.AVAILABLE.value, "released at startup", DroneStatus.BUSY.value),
        )
        await db.commit()

    if job_ids:
        logger.warning("recovered %d orphaned job(s): %s", len(job_ids), ", ".join(job_ids))
    return len(job_ids)
