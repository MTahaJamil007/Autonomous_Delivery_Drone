-- Fleet dispatch schema: drone registry and job queue.

-- WAL is set by the connection helper, not here: PRAGMA journal_mode is a
-- per-database persistent setting but executing it inside executescript() on a
-- fresh file is unreliable, and it must be in force for the atomic claim in
-- claim_drone_for_job() to behave under concurrency.

CREATE TABLE IF NOT EXISTS drones (
    id              TEXT PRIMARY KEY,
    status          TEXT NOT NULL DEFAULT 'AVAILABLE',  -- AVAILABLE|BUSY|OFFLINE|ERROR
    last_heartbeat  TEXT NOT NULL,
    current_job_id  TEXT,
    battery_pct     REAL DEFAULT 100.0,
    lat             REAL DEFAULT 0.0,
    lon             REAL DEFAULT 0.0,
    alt             REAL DEFAULT 0.0,
    detail          TEXT DEFAULT ''      -- last status line shown in the UI
);

CREATE TABLE IF NOT EXISTS jobs (
    id              TEXT PRIMARY KEY,
    pickup_lat      REAL NOT NULL,
    pickup_lon      REAL NOT NULL,
    drop_lat        REAL NOT NULL,
    drop_lon        REAL NOT NULL,
    assigned_drone  TEXT,
    -- QUEUED|ASSIGNED|IN_PROGRESS|COMPLETED|FAILED|ABORTED
    status          TEXT NOT NULL DEFAULT 'QUEUED',
    -- Why the job ended the way it did. A FAILED row with no reason is barely
    -- better than the silent COMPLETED this replaces.
    detail          TEXT DEFAULT '',
    created_at      TEXT NOT NULL,
    started_at      TEXT,
    completed_at    TEXT,
    FOREIGN KEY (assigned_drone) REFERENCES drones(id)
);

-- The circular FK drones.current_job_id -> jobs(id) is deliberately absent:
-- with it, neither table can be inserted into first. Consistency is maintained
-- in claim_drone_for_job() and finalize_job(), which update both sides inside
-- one transaction.

CREATE INDEX IF NOT EXISTS idx_jobs_status  ON jobs(status);
CREATE INDEX IF NOT EXISTS idx_jobs_created ON jobs(created_at);
CREATE INDEX IF NOT EXISTS idx_jobs_drone   ON jobs(assigned_drone);
CREATE INDEX IF NOT EXISTS idx_drones_status ON drones(status);
