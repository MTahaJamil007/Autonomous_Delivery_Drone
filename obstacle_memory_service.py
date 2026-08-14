#!/usr/bin/env python3
"""
Obstacle Memory Service - Fleet-wide obstacle database.

Allows drones to:
1. Report newly discovered obstacles
2. Fetch known obstacles before missions
3. Share obstacle data across the entire fleet

Runs on http://127.0.0.1:5050
"""

import asyncio
import logging
import time
import math
from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid

from fastapi import FastAPI, Query, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import uvicorn

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Obstacle Memory Service",
    description="Fleet-wide obstacle database for multi-drone coordination",
    version="1.0.0"
)

# In-memory obstacle database
# In production, this would be backed by a real database (PostgreSQL, MongoDB, etc.)
obstacles_db: Dict[str, Dict[str, Any]] = {}

OBSTACLE_MERGE_RADIUS_M = 5.0  # Merge obstacles within 5m
OBSTACLE_DECAY_TAU_DAYS = 14   # Confidence decay time constant


# ════════════════════════════════════════════════════════════════════════════
#  DATA MODELS
# ════════════════════════════════════════════════════════════════════════════

class ObstacleReport(BaseModel):
    """Report of a newly discovered obstacle."""
    lat: float
    lon: float
    radius_m: float = 3.0
    obstacle_type: str = "static_wall"
    source_drone: str
    confidence: float = 0.5


class ObstacleResponse(BaseModel):
    """Response containing obstacle ID."""
    obstacle_id: str
    message: str
    merged: bool = False


# ════════════════════════════════════════════════════════════════════════════
#  HELPER FUNCTIONS
# ════════════════════════════════════════════════════════════════════════════

def get_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate flat-earth distance in meters."""
    d_lat = (lat2 - lat1) * 111_320.0
    d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
    return math.hypot(d_lat, d_lon)


def apply_confidence_decay(obstacle: Dict[str, Any]) -> float:
    """
    Apply exponential confidence decay based on age.
    
    Confidence decays with tau = OBSTACLE_DECAY_TAU_DAYS.
    """
    age_days = (time.time() - obstacle["first_seen"]) / 86400.0
    decay_factor = math.exp(-age_days / OBSTACLE_DECAY_TAU_DAYS)
    return obstacle["base_confidence"] * decay_factor


def find_nearby_obstacle(lat: float, lon: float, radius_m: float) -> Optional[str]:
    """Find if there's already an obstacle within merge radius."""
    for obs_id, obs in obstacles_db.items():
        distance = get_distance_m(lat, lon, obs["lat"], obs["lon"])
        if distance < OBSTACLE_MERGE_RADIUS_M:
            return obs_id
    return None


# ════════════════════════════════════════════════════════════════════════════
#  API ENDPOINTS
# ════════════════════════════════════════════════════════════════════════════

@app.get("/")
async def root():
    """Root endpoint with service info."""
    return {
        "service": "Obstacle Memory Service",
        "version": "1.0.0",
        "total_obstacles": len(obstacles_db),
        "endpoints": {
            "report": "POST /obstacles",
            "fetch": "GET /obstacles?min_lat=...&max_lat=...&min_lon=...&max_lon=...",
            "all": "GET /obstacles",
            "stats": "GET /stats"
        }
    }


@app.post("/obstacles", response_model=ObstacleResponse)
async def report_obstacle(report: ObstacleReport):
    """
    Report a newly discovered obstacle.
    
    If an obstacle already exists nearby (within MERGE_RADIUS), the reports
    are merged and confidence is increased.
    """
    # Check for nearby existing obstacle
    existing_id = find_nearby_obstacle(report.lat, report.lon, report.radius_m)
    
    if existing_id:
        # Merge with existing obstacle
        obstacle = obstacles_db[existing_id]
        
        # Update confidence (simple average for now)
        old_conf = obstacle["base_confidence"]
        new_conf = (old_conf + report.confidence) / 2.0
        obstacle["base_confidence"] = min(1.0, new_conf)
        
        # Add reporter
        if report.source_drone not in obstacle["reporters"]:
            obstacle["reporters"].append(report.source_drone)
        
        obstacle["report_count"] += 1
        obstacle["last_seen"] = time.time()
        
        logger.info(
            f"🔄 MERGED obstacle {existing_id} from {report.source_drone} "
            f"(confidence: {old_conf:.2f} → {new_conf:.2f}, "
            f"reports: {obstacle['report_count']})"
        )
        
        return ObstacleResponse(
            obstacle_id=existing_id,
            message="Obstacle report merged with existing entry",
            merged=True
        )
    
    else:
        # Create new obstacle
        obstacle_id = str(uuid.uuid4())[:8]
        
        obstacles_db[obstacle_id] = {
            "id": obstacle_id,
            "lat": report.lat,
            "lon": report.lon,
            "radius_m": report.radius_m,
            "obstacle_type": report.obstacle_type,
            "base_confidence": report.confidence,
            "first_seen": time.time(),
            "last_seen": time.time(),
            "reporters": [report.source_drone],
            "report_count": 1
        }
        
        logger.info(
            f"✅ NEW obstacle {obstacle_id} reported by {report.source_drone} "
            f"at ({report.lat:.6f}, {report.lon:.6f})"
        )
        
        return ObstacleResponse(
            obstacle_id=obstacle_id,
            message="New obstacle registered",
            merged=False
        )


@app.get("/obstacles")
async def get_obstacles(
    min_lat: Optional[float] = None,
    max_lat: Optional[float] = None,
    min_lon: Optional[float] = None,
    max_lon: Optional[float] = None,
    min_confidence: float = Query(0.15, ge=0.0, le=1.0)
):
    """
    Fetch obstacles, optionally filtered by bounding box and confidence.
    
    Args:
        min_lat, max_lat, min_lon, max_lon: Optional bounding box
        min_confidence: Minimum confidence threshold
        
    Returns:
        List of obstacles with current (decayed) confidence
    """
    filtered = []
    
    for obs in obstacles_db.values():
        # Apply confidence decay
        current_confidence = apply_confidence_decay(obs)
        
        # Filter by confidence
        if current_confidence < min_confidence:
            continue
        
        # Filter by bounding box if provided
        if min_lat is not None and obs["lat"] < min_lat:
            continue
        if max_lat is not None and obs["lat"] > max_lat:
            continue
        if min_lon is not None and obs["lon"] < min_lon:
            continue
        if max_lon is not None and obs["lon"] > max_lon:
            continue
        
        # Add to results with current confidence
        obstacle_copy = obs.copy()
        obstacle_copy["current_confidence"] = current_confidence
        obstacle_copy["age_days"] = (time.time() - obs["first_seen"]) / 86400.0
        filtered.append(obstacle_copy)
    
    logger.info(
        f"Fetched {len(filtered)}/{len(obstacles_db)} obstacles "
        f"(min_confidence={min_confidence:.2f})"
    )
    
    return {
        "count": len(filtered),
        "obstacles": filtered,
        "filter": {
            "bbox_active": min_lat is not None,
            "min_confidence": min_confidence
        }
    }


@app.get("/obstacles/{obstacle_id}")
async def get_obstacle(obstacle_id: str):
    """Get details of a specific obstacle."""
    if obstacle_id not in obstacles_db:
        raise HTTPException(status_code=404, detail="Obstacle not found")
    
    obs = obstacles_db[obstacle_id].copy()
    obs["current_confidence"] = apply_confidence_decay(obs)
    obs["age_days"] = (time.time() - obs["first_seen"]) / 86400.0
    
    return obs


@app.get("/stats")
async def get_stats():
    """Get database statistics."""
    if not obstacles_db:
        return {
            "total_obstacles": 0,
            "avg_confidence": 0.0,
            "avg_age_days": 0.0
        }
    
    confidences = [apply_confidence_decay(obs) for obs in obstacles_db.values()]
    ages = [(time.time() - obs["first_seen"]) / 86400.0 for obs in obstacles_db.values()]
    
    return {
        "total_obstacles": len(obstacles_db),
        "avg_confidence": sum(confidences) / len(confidences),
        "avg_age_days": sum(ages) / len(ages),
        "oldest_age_days": max(ages) if ages else 0,
        "by_type": {}  # Could be expanded
    }


@app.delete("/obstacles/{obstacle_id}")
async def delete_obstacle(obstacle_id: str):
    """Delete a specific obstacle (admin endpoint)."""
    if obstacle_id not in obstacles_db:
        raise HTTPException(status_code=404, detail="Obstacle not found")
    
    del obstacles_db[obstacle_id]
    logger.info(f"🗑️  Deleted obstacle {obstacle_id}")
    
    return {"message": f"Obstacle {obstacle_id} deleted"}


@app.delete("/obstacles")
async def clear_all_obstacles():
    """Clear all obstacles (admin endpoint)."""
    count = len(obstacles_db)
    obstacles_db.clear()
    logger.warning(f"🗑️  CLEARED all {count} obstacles")
    
    return {"message": f"Cleared {count} obstacles"}


# ════════════════════════════════════════════════════════════════════════════
#  SERVER STARTUP
# ════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    logger.info("🚀 Starting Obstacle Memory Service on http://127.0.0.1:5050")
    uvicorn.run(app, host="127.0.0.1", port=5050, log_level="info")
