-- Fleet Dispatch Database Schema
-- Manages drone registry and job queue for multi-drone operations

-- Create drones table first (no FK dependency)
CREATE TABLE IF NOT EXISTS drones (
    id TEXT PRIMARY KEY,
    status TEXT NOT NULL DEFAULT 'AVAILABLE',  -- AVAILABLE, BUSY, OFFLINE, ERROR
    last_heartbeat TEXT NOT NULL,
    current_job_id TEXT,
    battery_pct REAL DEFAULT 100.0,
    lat REAL DEFAULT 0.0,
    lon REAL DEFAULT 0.0,
    alt REAL DEFAULT 0.0
);

-- Create jobs table second (references drones)
CREATE TABLE IF NOT EXISTS jobs (
    id TEXT PRIMARY KEY,
    pickup_lat REAL NOT NULL,
    pickup_lon REAL NOT NULL,
    drop_lat REAL NOT NULL,
    drop_lon REAL NOT NULL,
    assigned_drone TEXT,
    status TEXT NOT NULL DEFAULT 'QUEUED',  -- QUEUED, ASSIGNED, IN_PROGRESS, COMPLETED, FAILED
    created_at TEXT NOT NULL,
    started_at TEXT,
    completed_at TEXT,
    FOREIGN KEY (assigned_drone) REFERENCES drones(id)
);

-- Note: Removed circular FK from drones.current_job_id → jobs(id)
-- This is managed at application level for consistency
-- SQLite doesn't enforce FKs by default anyway (PRAGMA foreign_keys = OFF)

-- Index for job status queries
CREATE INDEX IF NOT EXISTS idx_jobs_status ON jobs(status);

-- Index for drone status queries
CREATE INDEX IF NOT EXISTS idx_drones_status ON drones(status);

-- Index for job assignment lookups
CREATE INDEX IF NOT EXISTS idx_jobs_drone ON jobs(assigned_drone);
