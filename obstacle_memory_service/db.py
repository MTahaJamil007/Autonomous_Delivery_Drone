"""Obstacle memory persistence: async SQLite with confidence decay.

This is the survivor of two implementations. The root-level
obstacle_memory_service.py kept obstacles in a Python dict with its own private
constants, so "fleet-wide obstacle sharing, persisted between missions" reset
every time the process restarted, and config.OBSTACLE_MERGE_RADIUS_M was
ignored. RUN_GUIDE.md's step 3 launched that one. It is deleted; this is what
runs now.

TWO SEMANTIC FIXES
------------------
1. RE-CONFIRMATION NOW RAISES CONFIDENCE. The old merge computed
   (existing + new) / 2, so re-reporting a 0.5-confidence obstacle at 0.5 left
   it at 0.5 forever. Confirming an obstacle a second time is evidence, and it
   has to move the number, or the database cannot distinguish a wall six drones
   have hit from a one-off spurious reading. Merging now uses a probabilistic
   OR: p = p_old + p_new * (1 - p_old), so 0.5 confirmed at 0.5 becomes 0.75 and
   repeated sightings asymptote toward 1.0 without ever exceeding it.

2. POSITIONS MERGE BY CONFIDENCE WEIGHT, not a plain average. An unweighted
   average lets a single low-confidence report drag a well-established
   obstacle's position halfway toward it.

Timestamps are timezone-aware ISO-8601 (contracts.utc_now_iso). The old
datetime.utcnow() produced naive datetimes that this module then parsed back with
fromisoformat() for the decay maths - a naive/aware mismatch there makes every
obstacle either ageless or infinitely old.
"""

from __future__ import annotations

import hashlib
import logging
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import aiosqlite

import config
from drone_agent import geo
from drone_agent.contracts import utc_now_iso

logger = logging.getLogger(__name__)

DB_PATH = Path(__file__).parent / "obstacles.db"
SCHEMA_PATH = Path(__file__).parent / "schema.sql"


def generate_obstacle_id(lat: float, lon: float) -> str:
    """Stable id derived from a rounded position.

    Deliberately position-derived rather than random: two drones reporting the
    same wall from the same place converge on the same id even if the merge-radius
    search misses, which makes a duplicate row an idempotent overwrite rather
    than a second obstacle.
    """
    return hashlib.md5(f"{lat:.6f},{lon:.6f}".encode()).hexdigest()[:16]


def _parse_timestamp(value: str) -> datetime:
    """Parse a stored timestamp, tolerating rows written before the tz fix."""
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        # Legacy naive row: it held UTC, so label it as such rather than letting
        # the comparison below raise.
        return parsed.replace(tzinfo=timezone.utc)
    return parsed


def compute_confidence_decay(
    last_confirmed: datetime, current_time: datetime, base_confidence: float
) -> float:
    """Exponentially decay confidence with age.

        confidence * exp(-age_days / OBSTACLE_DECAY_TAU_DAYS)

    Decay is what makes the database self-correcting: a wall that has been
    demolished stops being routed around after a few tau, without anyone having
    to remember to delete it. Applied at QUERY time, not on write, so the stored
    value stays the evidence and the decay stays a function of when you ask.
    """
    age_days = (current_time - last_confirmed).total_seconds() / 86400.0
    return base_confidence * math.exp(-age_days / config.OBSTACLE_DECAY_TAU_DAYS)


def merge_confidence(existing: float, incoming: float) -> float:
    """Combine two independent sightings' confidences.

    Probabilistic OR: the chance an obstacle is real given two independent
    observations. Monotonically increasing, bounded above by 1.0, and it never
    lets a fresh confirmation *lower* an established obstacle's confidence -
    which an average does, and which is why the P5 acceptance criterion of
    "confidence > 0.5 after re-confirmation" was previously unreachable.
    """
    return min(1.0, existing + incoming * (1.0 - existing))


def _connect() -> aiosqlite.Connection:
    return aiosqlite.connect(DB_PATH, timeout=10.0)


async def _configure(db: aiosqlite.Connection) -> None:
    # WAL: several drones report while the dispatcher and planners read.
    await db.execute("PRAGMA journal_mode=WAL")
    await db.execute("PRAGMA busy_timeout=10000")


async def init_database() -> None:
    async with _connect() as db:
        await _configure(db)
        await db.executescript(SCHEMA_PATH.read_text())
        await db.commit()
    logger.info("obstacle database ready at %s", DB_PATH)


async def upsert_obstacle(
    lat: float,
    lon: float,
    radius_m: float = config.OBSTACLE_DEFAULT_RADIUS_M,
    obstacle_type: str = "static_wall",
    source_drone: str | None = None,
    confidence: float = 0.5,
) -> str:
    """Record an obstacle, merging with a nearby existing one if there is one.

    Runs the whole read-merge-write in ONE transaction on ONE connection, so two
    drones reporting the same wall simultaneously cannot both decide there is no
    existing row and each insert one.
    """
    async with _connect() as db:
        await _configure(db)
        await db.execute("BEGIN IMMEDIATE")

        # Coarse SQL prefilter, then exact metric distance. Comparing every row
        # in Python was fine at ten obstacles and is not the shape to keep.
        pad_deg = config.OBSTACLE_MERGE_RADIUS_M / geo.METRES_PER_DEG_LAT * 2.0
        cursor = await db.execute(
            "SELECT id, lat, lon, confidence, radius_m FROM obstacles "
            "WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?",
            (lat - pad_deg, lat + pad_deg, lon - pad_deg, lon + pad_deg),
        )
        candidates = await cursor.fetchall()

        now = utc_now_iso()

        for existing_id, existing_lat, existing_lon, existing_conf, existing_radius in candidates:
            if (
                geo.get_distance_m(lat, lon, existing_lat, existing_lon)
                > config.OBSTACLE_MERGE_RADIUS_M
            ):
                continue

            new_confidence = merge_confidence(existing_conf, confidence)

            # Confidence-weighted position: a weak new report nudges a strong
            # obstacle slightly; an unweighted average would move it halfway.
            total_weight = existing_conf + confidence
            if total_weight > 0:
                new_lat = (existing_lat * existing_conf + lat * confidence) / total_weight
                new_lon = (existing_lon * existing_conf + lon * confidence) / total_weight
            else:
                new_lat, new_lon = existing_lat, existing_lon

            # Radius grows to cover both sightings: a 30 m wall seen from two
            # ends is one large obstacle, not two small ones.
            new_radius = max(existing_radius, radius_m)

            await db.execute(
                "UPDATE obstacles SET lat = ?, lon = ?, confidence = ?, "
                "radius_m = ?, last_confirmed = ? WHERE id = ?",
                (new_lat, new_lon, new_confidence, new_radius, now, existing_id),
            )
            await db.commit()

            logger.info(
                "merged obstacle %s at (%.6f, %.6f): confidence %.2f -> %.2f (reported by %s)",
                existing_id,
                new_lat,
                new_lon,
                existing_conf,
                new_confidence,
                source_drone or "unknown",
            )
            return existing_id

        obstacle_id = generate_obstacle_id(lat, lon)
        await db.execute(
            "INSERT OR REPLACE INTO obstacles (id, lat, lon, radius_m, "
            "obstacle_type, confidence, source_drone, first_seen, last_confirmed) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (obstacle_id, lat, lon, radius_m, obstacle_type, confidence, source_drone, now, now),
        )
        await db.commit()

        logger.info(
            "new obstacle %s at (%.6f, %.6f), r=%.1f m, confidence %.2f, from %s",
            obstacle_id,
            lat,
            lon,
            radius_m,
            confidence,
            source_drone or "unknown",
        )
        return obstacle_id


def _decay_rows(rows: list[Any], min_confidence: float) -> list[dict[str, Any]]:
    """Apply age decay and filter. Shared by both query paths."""
    now = datetime.now(timezone.utc)
    results = []
    for row in rows:
        record = dict(row)
        decayed = compute_confidence_decay(
            _parse_timestamp(record["last_confirmed"]), now, record["confidence"]
        )
        if decayed >= min_confidence:
            record["confidence"] = decayed
            record["stored_confidence"] = row["confidence"]
            results.append(record)
    return results


async def query_bbox(
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float,
    min_confidence: float = config.OBSTACLE_MIN_CONFIDENCE,
) -> list[dict[str, Any]]:
    """Obstacles in a bounding box, with decay applied and low confidence dropped."""
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(
            "SELECT * FROM obstacles WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?",
            (min_lat, max_lat, min_lon, max_lon),
        )
        results = _decay_rows(await cursor.fetchall(), min_confidence)

    logger.debug(
        "bbox query (%.4f,%.4f)-(%.4f,%.4f) returned %d obstacle(s)",
        min_lat,
        min_lon,
        max_lat,
        max_lon,
        len(results),
    )
    return results


async def get_all_obstacles(
    min_confidence: float = 0.0,
) -> list[dict[str, Any]]:
    async with _connect() as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM obstacles ORDER BY last_confirmed DESC")
        return _decay_rows(await cursor.fetchall(), min_confidence)


async def get_stats() -> dict[str, Any]:
    """Summary for the operator: how much has the fleet actually learned?"""
    obstacles = await get_all_obstacles()
    by_drone: dict[str, int] = {}
    for obstacle in obstacles:
        key = obstacle.get("source_drone") or "unknown"
        by_drone[key] = by_drone.get(key, 0) + 1

    confident = [o for o in obstacles if o["confidence"] >= 0.5]
    return {
        "total": len(obstacles),
        "confident": len(confident),
        "by_source_drone": by_drone,
        "merge_radius_m": config.OBSTACLE_MERGE_RADIUS_M,
        "decay_tau_days": config.OBSTACLE_DECAY_TAU_DAYS,
        "db_path": str(DB_PATH),
    }
