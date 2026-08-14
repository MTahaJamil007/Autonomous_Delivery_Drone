"""
Obstacle Memory Database Module.

Provides async SQLite operations for obstacle storage, retrieval, and
confidence decay management.
"""

import aiosqlite
import logging
import math
import hashlib
from datetime import datetime, timedelta
from typing import List, Dict, Any, Optional, Tuple
from pathlib import Path

# R4.1: Use relative path instead of hardcoded absolute path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config

logger = logging.getLogger(__name__)

# Database file location
DB_PATH = Path(__file__).parent / "obstacles.db"
SCHEMA_PATH = Path(__file__).parent / "schema.sql"


def generate_obstacle_id(lat: float, lon: float) -> str:
    """
    Generate a unique ID for an obstacle based on its location.
    
    Args:
        lat, lon: Obstacle coordinates
        
    Returns:
        Hexadecimal hash string
    """
    location_str = f"{lat:.6f},{lon:.6f}"
    return hashlib.md5(location_str.encode()).hexdigest()[:16]


def get_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate flat-earth distance in meters."""
    d_lat = (lat2 - lat1) * 111_320.0
    d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
    return math.hypot(d_lat, d_lon)


def compute_confidence_decay(
    last_confirmed: datetime,
    current_time: datetime,
    base_confidence: float
) -> float:
    """
    Apply exponential decay to confidence based on age.
    
    Formula: confidence * exp(-age_days / tau)
    
    Args:
        last_confirmed: When obstacle was last confirmed
        current_time: Current time
        base_confidence: Original confidence value
        
    Returns:
        Decayed confidence value
    """
    age = current_time - last_confirmed
    age_days = age.total_seconds() / 86400.0
    
    decay_factor = math.exp(-age_days / config.OBSTACLE_DECAY_TAU_DAYS)
    return base_confidence * decay_factor


async def init_database() -> None:
    """
    Initialize database schema if it doesn't exist.
    """
    async with aiosqlite.connect(DB_PATH) as db:
        with open(SCHEMA_PATH, 'r') as f:
            schema = f.read()
        await db.executescript(schema)
        await db.commit()
    logger.info(f"Database initialized at {DB_PATH}")


async def upsert_obstacle(
    lat: float,
    lon: float,
    radius_m: float = 3.0,
    obstacle_type: str = 'static_wall',
    source_drone: Optional[str] = None,
    confidence: float = 0.5
) -> str:
    """
    Insert or update an obstacle, merging with nearby existing obstacles.
    
    If an obstacle exists within OBSTACLE_MERGE_RADIUS_M, update it by:
    - Bumping confidence (average of old and new)
    - Updating last_confirmed timestamp
    - Averaging the position
    
    Args:
        lat, lon: Obstacle coordinates
        radius_m: Obstacle radius
        obstacle_type: Type classification
        source_drone: Drone that reported this obstacle
        confidence: Initial confidence value
        
    Returns:
        Obstacle ID (existing or newly created)
    """
    async with aiosqlite.connect(DB_PATH) as db:
        # Find nearby obstacles within merge radius
        cursor = await db.execute(
            """
            SELECT id, lat, lon, confidence, first_seen
            FROM obstacles
            """
        )
        rows = await cursor.fetchall()
        
        now_str = datetime.utcnow().isoformat()
        
        # Check for mergeable obstacle
        for row in rows:
            existing_id, existing_lat, existing_lon, existing_conf, first_seen = row
            dist = get_distance_m(lat, lon, existing_lat, existing_lon)
            
            if dist <= config.OBSTACLE_MERGE_RADIUS_M:
                # Merge with existing obstacle
                new_conf = (existing_conf + confidence) / 2.0
                new_lat = (existing_lat + lat) / 2.0
                new_lon = (existing_lon + lon) / 2.0
                
                await db.execute(
                    """
                    UPDATE obstacles
                    SET lat = ?, lon = ?, confidence = ?, last_confirmed = ?
                    WHERE id = ?
                    """,
                    (new_lat, new_lon, min(1.0, new_conf), now_str, existing_id)
                )
                await db.commit()
                
                logger.info(
                    f"Merged obstacle {existing_id} at ({new_lat:.6f}, {new_lon:.6f}), "
                    f"confidence={new_conf:.2f}"
                )
                return existing_id
        
        # No mergeable obstacle found - create new
        obstacle_id = generate_obstacle_id(lat, lon)
        
        await db.execute(
            """
            INSERT OR REPLACE INTO obstacles
            (id, lat, lon, radius_m, obstacle_type, confidence, source_drone, first_seen, last_confirmed)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (obstacle_id, lat, lon, radius_m, obstacle_type, confidence, 
             source_drone, now_str, now_str)
        )
        await db.commit()
        
        logger.info(
            f"Created obstacle {obstacle_id} at ({lat:.6f}, {lon:.6f}), "
            f"confidence={confidence:.2f}"
        )
        return obstacle_id


async def query_bbox(
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float,
    min_confidence: float = 0.15
) -> List[Dict[str, Any]]:
    """
    Query obstacles within a bounding box with confidence decay applied.
    
    Args:
        min_lat, max_lat: Latitude bounds
        min_lon, max_lon: Longitude bounds
        min_confidence: Minimum decayed confidence to include
        
    Returns:
        List of obstacle dicts with decayed confidence
    """
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute(
            """
            SELECT * FROM obstacles
            WHERE lat BETWEEN ? AND ?
            AND lon BETWEEN ? AND ?
            """,
            (min_lat, max_lat, min_lon, max_lon)
        )
        rows = await cursor.fetchall()
        
        current_time = datetime.utcnow()
        results = []
        
        for row in rows:
            row_dict = dict(row)
            
            # Parse timestamp and apply decay
            last_confirmed = datetime.fromisoformat(row_dict['last_confirmed'])
            base_confidence = row_dict['confidence']
            
            decayed_confidence = compute_confidence_decay(
                last_confirmed, current_time, base_confidence
            )
            
            # Filter by minimum confidence
            if decayed_confidence >= min_confidence:
                row_dict['confidence'] = decayed_confidence
                results.append(row_dict)
        
        logger.debug(
            f"Query returned {len(results)} obstacles in bbox "
            f"({min_lat:.4f},{min_lon:.4f}) to ({max_lat:.4f},{max_lon:.4f})"
        )
        
        return results


async def get_all_obstacles() -> List[Dict[str, Any]]:
    """
    Get all obstacles with decayed confidence.
    
    Returns:
        List of all obstacle dicts
    """
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM obstacles")
        rows = await cursor.fetchall()
        
        current_time = datetime.utcnow()
        results = []
        
        for row in rows:
            row_dict = dict(row)
            last_confirmed = datetime.fromisoformat(row_dict['last_confirmed'])
            base_confidence = row_dict['confidence']
            
            decayed_confidence = compute_confidence_decay(
                last_confirmed, current_time, base_confidence
            )
            
            row_dict['confidence'] = decayed_confidence
            results.append(row_dict)
        
        return results
