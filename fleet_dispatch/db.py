"""
Fleet Dispatch Database Module.

Manages drone registry and job queue for multi-drone coordination.
"""

import aiosqlite
import logging
import hashlib
from datetime import datetime
from typing import List, Dict, Any, Optional
from pathlib import Path

# R4.1: Use relative path instead of hardcoded absolute path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config

logger = logging.getLogger(__name__)

# Database file location
DB_PATH = Path(__file__).parent / "fleet.db"
SCHEMA_PATH = Path(__file__).parent / "schema.sql"


def generate_job_id() -> str:
    """Generate a unique job ID based on timestamp."""
    timestamp = datetime.utcnow().isoformat()
    return hashlib.md5(timestamp.encode()).hexdigest()[:12]


async def init_database() -> None:
    """Initialize database schema and register drones from config."""
    async with aiosqlite.connect(DB_PATH) as db:
        # Create tables
        with open(SCHEMA_PATH, 'r') as f:
            schema = f.read()
        await db.executescript(schema)
        
        # Register fleet drones if they don't exist
        now = datetime.utcnow().isoformat()
        for drone_id in config.DRONE_IDS:
            await db.execute(
                """
                INSERT OR IGNORE INTO drones (id, status, last_heartbeat)
                VALUES (?, 'AVAILABLE', ?)
                """,
                (drone_id, now)
            )
        
        await db.commit()
    logger.info(f"Fleet database initialized at {DB_PATH}")


# ════════════════════════════════════════════════════════════════════════════
#  DRONE OPERATIONS
# ════════════════════════════════════════════════════════════════════════════

async def update_drone_status(
    drone_id: str,
    status: Optional[str] = None,
    battery_pct: Optional[float] = None,
    lat: Optional[float] = None,
    lon: Optional[float] = None,
    alt: Optional[float] = None
) -> None:
    """
    Update drone status and telemetry.
    
    R4.4 FIX: status parameter is now optional (None = leave status column untouched).
    This allows heartbeat refresh without racing with assign_job/complete_job status transitions.
    
    Args:
        drone_id: Drone identifier
        status: Status string (AVAILABLE, BUSY, OFFLINE, ERROR) - optional (None = no change)
        battery_pct: Battery percentage (optional)
        lat, lon, alt: GPS position (optional)
    """
    async with aiosqlite.connect(DB_PATH) as db:
        now = datetime.utcnow().isoformat()
        
        # Build dynamic update query (R4.4: only update columns that were passed)
        updates = ["last_heartbeat = ?"]
        values = [now]
        
        if status is not None:
            updates.append("status = ?")
            values.append(status)
        if battery_pct is not None:
            updates.append("battery_pct = ?")
            values.append(battery_pct)
        if lat is not None:
            updates.append("lat = ?")
            values.append(lat)
        if lon is not None:
            updates.append("lon = ?")
            values.append(lon)
        if alt is not None:
            updates.append("alt = ?")
            values.append(alt)
        
        values.append(drone_id)
        
        query = f"UPDATE drones SET {', '.join(updates)} WHERE id = ?"
        await db.execute(query, values)
        await db.commit()


async def get_drone_status(drone_id: str) -> Optional[Dict[str, Any]]:
    """Get current status of a drone."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(
            "SELECT * FROM drones WHERE id = ?",
            (drone_id,)
        )
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_all_drones() -> List[Dict[str, Any]]:
    """Get status of all drones."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM drones ORDER BY id")
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]


# ════════════════════════════════════════════════════════════════════════════
#  JOB OPERATIONS
# ════════════════════════════════════════════════════════════════════════════

async def create_job(
    pickup_lat: float,
    pickup_lon: float,
    drop_lat: float,
    drop_lon: float
) -> str:
    """
    Create a new delivery job.
    
    Args:
        pickup_lat, pickup_lon: Pickup coordinates
        drop_lat, drop_lon: Drop-off coordinates
        
    Returns:
        Job ID
    """
    job_id = generate_job_id()
    now = datetime.utcnow().isoformat()
    
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            INSERT INTO jobs (id, pickup_lat, pickup_lon, drop_lat, drop_lon, status, created_at)
            VALUES (?, ?, ?, ?, ?, 'QUEUED', ?)
            """,
            (job_id, pickup_lat, pickup_lon, drop_lat, drop_lon, now)
        )
        await db.commit()
    
    logger.info(f"Created job {job_id}")
    return job_id


async def assign_job(job_id: str, drone_id: str) -> bool:
    """
    Assign a job to a drone.
    
    Args:
        job_id: Job identifier
        drone_id: Drone identifier
        
    Returns:
        True if assignment succeeded
    """
    now = datetime.utcnow().isoformat()
    
    async with aiosqlite.connect(DB_PATH) as db:
        # Update job
        await db.execute(
            """
            UPDATE jobs
            SET assigned_drone = ?, status = 'ASSIGNED', started_at = ?
            WHERE id = ? AND status = 'QUEUED'
            """,
            (drone_id, now, job_id)
        )
        
        # Update drone
        await db.execute(
            """
            UPDATE drones
            SET status = 'BUSY', current_job_id = ?
            WHERE id = ?
            """,
            (job_id, drone_id)
        )
        
        await db.commit()
        
        logger.info(f"Assigned job {job_id} to drone {drone_id}")
        return True


async def complete_job(job_id: str) -> None:
    """Mark a job as completed and free the assigned drone."""
    now = datetime.utcnow().isoformat()
    
    async with aiosqlite.connect(DB_PATH) as db:
        # Get assigned drone
        cursor = await db.execute(
            "SELECT assigned_drone FROM jobs WHERE id = ?",
            (job_id,)
        )
        row = await cursor.fetchone()
        if not row:
            return
        
        drone_id = row[0]
        
        # Update job
        await db.execute(
            """
            UPDATE jobs
            SET status = 'COMPLETED', completed_at = ?
            WHERE id = ?
            """,
            (now, job_id)
        )
        
        # Free drone
        await db.execute(
            """
            UPDATE drones
            SET status = 'AVAILABLE', current_job_id = NULL
            WHERE id = ?
            """,
            (drone_id,)
        )
        
        await db.commit()
        logger.info(f"Completed job {job_id}, freed drone {drone_id}")


async def get_available_drone() -> Optional[str]:
    """
    Get the ID of an available drone (round-robin).
    
    Returns:
        Drone ID if available, None otherwise
    """
    async with aiosqlite.connect(DB_PATH) as db:
        cursor = await db.execute(
            """
            SELECT id FROM drones
            WHERE status = 'AVAILABLE'
            ORDER BY last_heartbeat ASC
            LIMIT 1
            """
        )
        row = await cursor.fetchone()
        return row[0] if row else None


async def get_job(job_id: str) -> Optional[Dict[str, Any]]:
    """Get job details."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(
            "SELECT * FROM jobs WHERE id = ?",
            (job_id,)
        )
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_all_jobs(status: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Get all jobs, optionally filtered by status.
    
    Args:
        status: Filter by status (QUEUED, ASSIGNED, IN_PROGRESS, COMPLETED, FAILED)
        
    Returns:
        List of job dicts
    """
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        
        if status:
            cursor = await db.execute(
                "SELECT * FROM jobs WHERE status = ? ORDER BY created_at DESC",
                (status,)
            )
        else:
            cursor = await db.execute(
                "SELECT * FROM jobs ORDER BY created_at DESC"
            )
        
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]
