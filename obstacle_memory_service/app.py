"""
Obstacle Memory Service - FastAPI REST API.

Provides endpoints for reporting and querying persistent obstacle information
across the drone fleet.
"""

from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel, Field
from typing import Optional, List
import logging

from db import init_database, upsert_obstacle, query_bbox, get_all_obstacles

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Obstacle Memory Service",
    description="Persistent obstacle storage and retrieval for autonomous drone fleet",
    version="1.0.0"
)


# ════════════════════════════════════════════════════════════════════════════
#  REQUEST/RESPONSE MODELS
# ════════════════════════════════════════════════════════════════════════════

class ObstacleReport(BaseModel):
    """Request model for reporting a new obstacle."""
    lat: float = Field(..., description="Latitude of obstacle")
    lon: float = Field(..., description="Longitude of obstacle")
    radius_m: float = Field(3.0, description="Obstacle radius in meters")
    obstacle_type: str = Field("static_wall", description="Obstacle classification")
    source_drone: Optional[str] = Field(None, description="Reporting drone ID")
    confidence: float = Field(0.5, ge=0.0, le=1.0, description="Initial confidence")


class ObstacleResponse(BaseModel):
    """Response model for obstacle operations."""
    obstacle_id: str
    message: str


# ════════════════════════════════════════════════════════════════════════════
#  LIFECYCLE EVENTS
# ════════════════════════════════════════════════════════════════════════════

@app.on_event("startup")
async def startup_event():
    """Initialize database on startup."""
    logger.info("🚀 Obstacle Memory Service starting...")
    await init_database()
    logger.info("✅ Service ready on port 5050")


# ════════════════════════════════════════════════════════════════════════════
#  API ENDPOINTS
# ════════════════════════════════════════════════════════════════════════════

@app.get("/")
async def root():
    """Health check endpoint."""
    return {
        "service": "Obstacle Memory Service",
        "status": "operational",
        "version": "1.0.0"
    }


@app.post("/obstacles", response_model=ObstacleResponse)
async def report_obstacle(report: ObstacleReport):
    """
    Report a new obstacle or update an existing one.
    
    If an obstacle exists within OBSTACLE_MERGE_RADIUS_M, it will be merged
    (position averaged, confidence bumped, timestamp updated). Otherwise, a
    new obstacle is created.
    """
    try:
        obstacle_id = await upsert_obstacle(
            lat=report.lat,
            lon=report.lon,
            radius_m=report.radius_m,
            obstacle_type=report.obstacle_type,
            source_drone=report.source_drone,
            confidence=report.confidence
        )
        
        return ObstacleResponse(
            obstacle_id=obstacle_id,
            message=f"Obstacle recorded: {obstacle_id}"
        )
    
    except Exception as e:
        logger.error(f"Failed to report obstacle: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/obstacles")
async def get_obstacles(
    min_lat: Optional[float] = Query(None, description="Minimum latitude"),
    max_lat: Optional[float] = Query(None, description="Maximum latitude"),
    min_lon: Optional[float] = Query(None, description="Minimum longitude"),
    max_lon: Optional[float] = Query(None, description="Maximum longitude"),
    min_confidence: float = Query(0.15, ge=0.0, le=1.0, description="Minimum confidence threshold")
):
    """
    Query obstacles within a bounding box (or all obstacles if no bbox provided).
    
    Confidence decay is applied at query time based on age since last_confirmed.
    Only obstacles with decayed confidence >= min_confidence are returned.
    """
    try:
        if all(v is not None for v in [min_lat, max_lat, min_lon, max_lon]):
            # Bounding box query
            obstacles = await query_bbox(
                min_lat, max_lat, min_lon, max_lon, min_confidence
            )
        else:
            # Return all obstacles (filtered by confidence)
            all_obstacles = await get_all_obstacles()
            obstacles = [obs for obs in all_obstacles if obs['confidence'] >= min_confidence]
        
        return {
            "count": len(obstacles),
            "obstacles": obstacles
        }
    
    except Exception as e:
        logger.error(f"Failed to query obstacles: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=5050, log_level="info")
