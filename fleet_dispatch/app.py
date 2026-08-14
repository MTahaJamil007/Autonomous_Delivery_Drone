from fastapi import FastAPI, Request, BackgroundTasks, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import Optional, Dict, List
import asyncio
import json
import logging

# Configure logging BEFORE it's used in import fallback (R4.5)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import fleet dispatch database
# R4.1: Use relative path instead of hardcoded absolute path
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fleet_dispatch import db as fleet_db

# Import original drone logic for mission execution
try:
    from drone_web.drone_logic import execute_delivery, drone_state
except ImportError as e:
    # Fallback if import fails - log critically so it's not silent (R4.5 FIXED)
    logger.critical(
        f"FAILED to import the real execute_delivery — every dispatch will be a NO-OP: {e}",
        exc_info=True,
    )
    drone_state = {"lat": 0.0, "lon": 0.0, "status": "Idle"}
    async def execute_delivery(p_lat, p_lon, d_lat, d_lon, drone_id="drone-0"):
        logger.critical(f"STUB execute_delivery called for {drone_id} — this drone is NOT flying.")

app = FastAPI(
    title="Fleet Dispatch Center",
    description="Multi-drone delivery coordination and monitoring",
    version="2.0.0"
)
templates = Jinja2Templates(directory="templates")

# WebSocket connections per drone
drone_connections: Dict[str, List[WebSocket]] = {}

# Legacy single-drone busy flag
drone_busy = False


# ════════════════════════════════════════════════════════════════════════════
#  REQUEST/RESPONSE MODELS
# ════════════════════════════════════════════════════════════════════════════

class DispatchRequest(BaseModel):
    """Request model for dispatching a delivery job."""
    pickup_lat: float
    pickup_lon: float
    drop_lat: float
    drop_lon: float


class DispatchResponse(BaseModel):
    """Response model for dispatch operation."""
    status: str
    message: str
    job_id: Optional[str] = None
    drone_id: Optional[str] = None


# ════════════════════════════════════════════════════════════════════════════
#  LIFECYCLE EVENTS
# ════════════════════════════════════════════════════════════════════════════

@app.on_event("startup")
async def startup_event():
    """Initialize fleet database on startup."""
    logger.info("🚀 Fleet Dispatch Center starting...")
    await fleet_db.init_database()
    logger.info("✅ Fleet database initialized")


# Keep run_drone_task for future integration (R1.4)
async def run_drone_task(job_id: str, drone_id: str, pickup_lat, pickup_lon, drop_lat, drop_lon):
    """
    Execute delivery mission, then always free the drone — success or failure.
    
    R4.1 FIX: Now passes drone_id to execute_delivery for per-drone port assignment.
    R4.4 FIX: Runs concurrent heartbeat refresh task to keep lat/lon/alt/last_heartbeat live.
    """
    # R4.4: Import get_state_snapshot for heartbeat refresh
    from drone_web.drone_logic import get_state_snapshot
    
    # R4.4: Start heartbeat refresh task
    heartbeat_task = None
    async def heartbeat_refresh():
        """Refresh drone telemetry every 2s without touching status."""
        while True:
            try:
                await asyncio.sleep(2.0)
                snapshot = get_state_snapshot(drone_id)
                await fleet_db.update_drone_status(
                    drone_id,
                    status=None,  # R4.4: Explicitly None - don't touch status column
                    lat=snapshot.get("lat"),
                    lon=snapshot.get("lon"),
                    alt=snapshot.get("alt")
                )
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.warning(f"Heartbeat refresh error for {drone_id}: {e}")
    
    try:
        # R4.4: Start heartbeat task
        heartbeat_task = asyncio.create_task(heartbeat_refresh())
        
        # R4.1: Pass drone_id to execute_delivery
        await execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon, drone_id)
    except Exception as e:
        logger.error(f"Mission failed for {drone_id} (job {job_id}): {e}", exc_info=True)
    finally:
        # R4.4: Cancel heartbeat task
        if heartbeat_task is not None:
            heartbeat_task.cancel()
            try:
                await heartbeat_task
            except asyncio.CancelledError:
                pass
        
        # Always free the drone
        await fleet_db.complete_job(job_id)


# ════════════════════════════════════════════════════════════════════════════
#  WEB INTERFACE
# ════════════════════════════════════════════════════════════════════════════

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """Render main fleet dispatch interface."""
    return templates.TemplateResponse(request=request, name="index.html")


# ════════════════════════════════════════════════════════════════════════════
#  FLEET DISPATCH API
# ════════════════════════════════════════════════════════════════════════════

@app.post("/dispatch", response_model=DispatchResponse)
async def dispatch(request: DispatchRequest, background_tasks: BackgroundTasks):
    """
    Dispatch a new delivery job to an available drone.
    
    If a drone is available, the job is assigned immediately and execution begins.
    If all drones are busy, the job is queued.
    """
    try:
        # Create job
        job_id = await fleet_db.create_job(
            request.pickup_lat,
            request.pickup_lon,
            request.drop_lat,
            request.drop_lon
        )
        
        # Try to assign to available drone
        drone_id = await fleet_db.get_available_drone()
        
        if drone_id:
            # Assign job
            await fleet_db.assign_job(job_id, drone_id)
            
            # Start mission in background (R1.4 - uncommented and wired)
            background_tasks.add_task(run_drone_task, job_id, drone_id,
                                     request.pickup_lat, request.pickup_lon,
                                     request.drop_lat, request.drop_lon)
            
            return DispatchResponse(
                status="Success",
                message=f"Job assigned to {drone_id}",
                job_id=job_id,
                drone_id=drone_id
            )
        else:
            # All drones busy - job queued
            return DispatchResponse(
                status="Queued",
                message="All drones busy - job queued",
                job_id=job_id
            )
    
    except Exception as e:
        logger.error(f"Dispatch failed: {e}", exc_info=True)
        return DispatchResponse(
            status="Error",
            message=str(e)
        )


@app.get("/fleet/status")
async def fleet_status():
    """Get status of all drones in the fleet."""
    drones = await fleet_db.get_all_drones()
    return {
        "fleet_size": len(drones),
        "drones": drones
    }


@app.get("/jobs")
async def get_jobs(status: Optional[str] = None):
    """Get all jobs, optionally filtered by status."""
    jobs = await fleet_db.get_all_jobs(status)
    return {
        "count": len(jobs),
        "jobs": jobs
    }


@app.get("/jobs/{job_id}")
async def get_job(job_id: str):
    """Get details of a specific job."""
    job = await fleet_db.get_job(job_id)
    if job:
        return job
    return {"error": "Job not found"}


# ════════════════════════════════════════════════════════════════════════════
#  WEBSOCKET - PER-DRONE TELEMETRY
# ════════════════════════════════════════════════════════════════════════════

@app.websocket("/ws/{drone_id}")
async def websocket_drone_telemetry(websocket: WebSocket, drone_id: str):
    """
    WebSocket endpoint for per-drone live telemetry.
    
    Each drone has its own channel: /ws/drone-0, /ws/drone-1, etc.
    """
    await websocket.accept()
    
    # Register connection
    if drone_id not in drone_connections:
        drone_connections[drone_id] = []
    drone_connections[drone_id].append(websocket)
    
    logger.info(f"WebSocket connected: {drone_id}")
    
    try:
        while True:
            # Get drone status from database
            status = await fleet_db.get_drone_status(drone_id)
            
            if status:
                await websocket.send_json(status)
            else:
                await websocket.send_json({
                    "drone_id": drone_id,
                    "status": "OFFLINE",
                    "message": "Drone not in registry"
                })
            
            await asyncio.sleep(0.5)  # 2 Hz update rate
    
    except WebSocketDisconnect:
        drone_connections[drone_id].remove(websocket)
        logger.info(f"WebSocket disconnected: {drone_id}")


# Legacy single-drone websocket (backward compatibility)
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Legacy single-drone telemetry endpoint."""
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(drone_state)
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass


# ════════════════════════════════════════════════════════════════════════════
#  SERVER STARTUP
# ════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="info")
