from fastapi import FastAPI, Request, BackgroundTasks, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import asyncio
from drone_logic import execute_delivery, drone_state

app = FastAPI()
templates = Jinja2Templates(directory="templates")

drone_busy = False

async def run_drone_task(pickup_lat, pickup_lon, drop_lat, drop_lon):
    global drone_busy
    drone_busy = True  
    try:
        await execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon)
    except Exception as e:
        drone_state["status"] = f"Error: {e}"
    finally:
        drone_busy = False 

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(request=request, name="index.html")

@app.post("/dispatch")
async def dispatch(request: Request, background_tasks: BackgroundTasks):
    global drone_busy
    if drone_busy:
        return {"status": "Error", "message": "Drone is currently busy!"}

    data = await request.json()
    p_lat = float(data['pickup_lat'])
    p_lon = float(data['pickup_lon'])
    d_lat = float(data['drop_lat'])
    d_lon = float(data['drop_lon'])

    background_tasks.add_task(run_drone_task, p_lat, p_lon, d_lat, d_lon)
    return {"status": "Success", "message": "Mission started! Check map for live tracking."}

# NEW: The Live Data WebSocket Endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Blast the current GPS coordinates to the browser 2 times a second
            await websocket.send_json(drone_state)
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass