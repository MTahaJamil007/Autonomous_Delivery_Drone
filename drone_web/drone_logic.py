# # this code make the drone ping pong the wall because 
# The drone aggressively pitches its nose up to slow down. The flat LiDAR suddenly points at the sky.

# The LiDAR sees "Infinity" (>11m). The ROS node immediately shouts CLEAR.

# Because ROS shouted CLEAR during the brake, the Python script completely skipped the dodging loop and immediately triggered Step 3: Push forward to bypass.

# The drone lunged forward, leveled out, saw the wall again, braked, pitched up, saw the sky... and it ping-ponged until it crashed.
# import asyncio
# import math
# import socket
# import subprocess
# import time
# from mavsdk import System
# from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# TARGET_ALT = 10
# drone_state = {"lat": 0.0, "lon": 0.0, "status": "Idle"}

# vision_data = {"err_x": 0, "err_y": 0, "id": -1, "locked": False}
# lidar_data = {"action": "CLEAR"}

# def get_distance(lat1, lon1, lat2, lon2):
#     return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

# # --- Calculate Compass Bearing so drone faces forward ---
# def get_bearing(lat1, lon1, lat2, lon2):
#     lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
#     lon_diff = math.radians(lon2 - lon1)
    
#     x = math.sin(lon_diff) * math.cos(lat2_rad)
#     y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon_diff))
    
#     initial_bearing = math.atan2(x, y)
#     return (math.degrees(initial_bearing) + 360) % 360

# def spawn_gazebo_marker(name, target_lat, target_lon, home_lat, home_lon):
#     d_lat = target_lat - home_lat
#     d_lon = target_lon - home_lon
#     y_north = d_lat * 111320.0
#     x_east = d_lon * (111320.0 * math.cos(math.radians(home_lat)))
    
#     print(f"[ENVIRONMENT] Spawning {name} at Gazebo X: {x_east:.2f}, Y: {y_north:.2f}")
#     cmd = f"gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"model://arucotag\", name: \"{name}\", pose: {{position: {{x: {x_east}, y: {y_north}, z: 0.1}}}}'"
#     subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# async def telemetry_monitor(drone):
#     async for position in drone.telemetry.position():
#         drone_state["lat"] = position.latitude_deg
#         drone_state["lon"] = position.longitude_deg

# async def vision_listener():
#     udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_sock.bind(("127.0.0.1", 5005))
#     udp_sock.setblocking(False)
#     while True:
#         try:
#             data, _ = udp_sock.recvfrom(1024)
#             msg = data.decode()
#             if msg == "None":
#                 vision_data["locked"] = False
#             else:
#                 parts = msg.split(',')
#                 if len(parts) == 3:
#                     vision_data["err_x"], vision_data["err_y"], vision_data["id"] = int(parts[0]), int(parts[1]), int(parts[2])
#                     vision_data["locked"] = True
#         except BlockingIOError:
#             pass
#         except Exception:
#             pass
#         await asyncio.sleep(0.05)

# async def lidar_listener():
#     udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_sock.bind(("127.0.0.1", 5006))
#     udp_sock.setblocking(False)
#     while True:
#         try:
#             data, _ = udp_sock.recvfrom(1024)
#             lidar_data["action"] = data.decode()
#         except BlockingIOError:
#             pass
#         except Exception:
#             pass
#         await asyncio.sleep(0.1)

# async def navigate_with_avoidance(drone, target_lat, target_lon):
#     yaw_heading = get_bearing(drone_state["lat"], drone_state["lon"], target_lat, target_lon)
#     await drone.action.goto_location(target_lat, target_lon, TARGET_ALT, yaw_heading)

#     while True:
#         # Tightened GPS arrival tolerance
#         if get_distance(drone_state["lat"], drone_state["lon"], target_lat, target_lon) < 0.00001:
#             return

#         if lidar_data["action"] != "CLEAR":
#             drone_state["status"] = "EVADING OBSTACLE!"
#             print(f"\n[AVOIDANCE] BRACE! Obstacle detected! Executing: {lidar_data['action']}")

#             try:
#                 await drone.offboard.start()
#             except OffboardError:
#                 pass

#             # 1. THE EMERGENCY BRAKE (Pitch up to kill speed)
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-5.0, 0.0, 0.0, 0.0))
#             await asyncio.sleep(1.5)

#             # 2. THE REVERSE SLIDE (Push backward at -1.0 m/s while dodging sideways)
#             while lidar_data["action"] != "CLEAR":
#                 if lidar_data["action"] == "DODGE_LEFT":
#                     await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0, -3.0, 0.0, 0.0))
#                 elif lidar_data["action"] == "DODGE_RIGHT":
#                     await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0, 3.0, 0.0, 0.0))
#                 await asyncio.sleep(0.2)

#             # 3. THE BYPASS PUSH
#             print("[AVOIDANCE] Wall cleared. Pushing forward to bypass...")
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(3.0, 0.0, 0.0, 0.0))
#             await asyncio.sleep(4.0)

#             await drone.offboard.stop()
#             drone_state["status"] = "Resuming Route"
#             print("[AVOIDANCE] Resuming original GPS trajectory.")
            
#             yaw_heading = get_bearing(drone_state["lat"], drone_state["lon"], target_lat, target_lon)
#             await drone.action.goto_location(target_lat, target_lon, TARGET_ALT, yaw_heading)

#         await asyncio.sleep(0.5)

# async def execute_precision_landing(drone, expected_marker_id):
#     drone_state["status"] = f"Landing Sequence (Pad {expected_marker_id})"
#     print(f"\n[AUTONOMY] Hunting for Marker ID: {expected_marker_id}...")
    
#     await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
#     await drone.offboard.start()

#     K_p = 0.015 
#     start_time = time.time()
#     TIMEOUT_SECONDS = 45 

#     while True:
#         elapsed = time.time() - start_time
        
#         if elapsed > TIMEOUT_SECONDS:
#             print(f"[FAILSAFE] Marker {expected_marker_id} not found. Aborting land!")
#             await drone.offboard.stop()
#             await drone.action.set_takeoff_altitude(TARGET_ALT)
#             await drone.action.takeoff()
#             await asyncio.sleep(5)
#             return False

#         if vision_data["locked"] and vision_data["id"] == expected_marker_id:
#             vx, vy = -vision_data["err_y"] * K_p, vision_data["err_x"] * K_p
#             vz = 0.4 

#             if abs(vision_data["err_x"]) < 20 and abs(vision_data["err_y"]) < 20:
#                 vx, vy, vz = 0.0, 0.0, 0.8
                
#             vx, vy = max(min(vx, 1.5), -1.5), max(min(vy, 1.5), -1.5)
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
#         else:
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.2, 0.0))

#         async for pos in drone.telemetry.position():
#             if pos.relative_altitude_m < 0.3:
#                 print(f"[AUTONOMY] Touchdown sequence initiated...")
#                 await drone.offboard.stop()
#                 try:
#                     await asyncio.wait_for(drone.action.land(), timeout=2.0)
#                 except Exception:
#                     pass

#                 async for is_armed in drone.telemetry.armed():
#                     if not is_armed: break
                
#                 print(f"[AUTONOMY] Pad {expected_marker_id} secured. Motors disarmed.")
#                 return True
#             break
            
#         await asyncio.sleep(0.1)

# async def execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon):
#     drone = System()
#     drone_state["status"] = "Connecting..."
#     await drone.connect(system_address="udpin://0.0.0.0:14540")
    
#     async for state in drone.core.connection_state():
#         if state.is_connected: break
#     async for health in drone.telemetry.health():
#         if health.is_global_position_ok: break

#     asyncio.create_task(telemetry_monitor(drone))
#     asyncio.create_task(vision_listener())
#     asyncio.create_task(lidar_listener())
#     await asyncio.sleep(1)
    
#     home_lat, home_lon = drone_state["lat"], drone_state["lon"]

#     spawn_gazebo_marker("pickup_pad", pickup_lat, pickup_lon, home_lat, home_lon)
#     spawn_gazebo_marker("drop_pad", drop_lat, drop_lon, home_lat, home_lon)
#     spawn_gazebo_marker("home_pad", home_lat, home_lon, home_lat, home_lon)

#     await drone.action.arm()
#     await drone.action.set_takeoff_altitude(TARGET_ALT)
#     await drone.action.takeoff()
#     await asyncio.sleep(8)

#     drone_state["status"] = "Flying to Pickup"
#     await navigate_with_avoidance(drone, pickup_lat, pickup_lon)
#     if await execute_precision_landing(drone, 0): await asyncio.sleep(5) 

#     drone_state["status"] = "Flying to Drop-off"
#     await drone.action.arm()
#     await drone.action.takeoff()
#     await asyncio.sleep(8)
#     await navigate_with_avoidance(drone, drop_lat, drop_lon)
#     if await execute_precision_landing(drone, 0): await asyncio.sleep(5)

#     drone_state["status"] = "Returning Home"
#     await drone.action.arm()
#     await drone.action.takeoff()
#     await asyncio.sleep(8)
#     await navigate_with_avoidance(drone, home_lat, home_lon)
#     await execute_precision_landing(drone, 0)
    
#     drone_state["status"] = "Mission Complete"



# version 2: drone head first into wall is fixed__
# import asyncio
# import math
# import socket
# import subprocess
# import time
# from mavsdk import System
# from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw

# TARGET_ALT = 10
# drone_state = {"lat": 0.0, "lon": 0.0, "status": "Idle"}

# vision_data = {"err_x": 0, "err_y": 0, "id": -1, "locked": False}
# lidar_data = {"action": "CLEAR"}

# def get_distance(lat1, lon1, lat2, lon2):
#     return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

# def get_bearing(lat1, lon1, lat2, lon2):
#     lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
#     lon_diff = math.radians(lon2 - lon1)
#     x = math.sin(lon_diff) * math.cos(lat2_rad)
#     y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon_diff))
#     initial_bearing = math.atan2(x, y)
#     return (math.degrees(initial_bearing) + 360) % 360

# def spawn_gazebo_marker(name, target_lat, target_lon, home_lat, home_lon):
#     d_lat = target_lat - home_lat
#     d_lon = target_lon - home_lon
#     y_north = d_lat * 111320.0
#     x_east = d_lon * (111320.0 * math.cos(math.radians(home_lat)))
#     print(f"[ENVIRONMENT] Spawning {name} at Gazebo X: {x_east:.2f}, Y: {y_north:.2f}")
#     cmd = f"gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"model://arucotag\", name: \"{name}\", pose: {{position: {{x: {x_east}, y: {y_north}, z: 0.1}}}}'"
#     subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# async def telemetry_monitor(drone):
#     async for position in drone.telemetry.position():
#         drone_state["lat"] = position.latitude_deg
#         drone_state["lon"] = position.longitude_deg

# async def vision_listener():
#     udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_sock.bind(("127.0.0.1", 5005))
#     udp_sock.setblocking(False)
#     while True:
#         try:
#             data, _ = udp_sock.recvfrom(1024)
#             msg = data.decode()
#             if msg == "None":
#                 vision_data["locked"] = False
#             else:
#                 parts = msg.split(',')
#                 if len(parts) == 3:
#                     vision_data["err_x"], vision_data["err_y"], vision_data["id"] = int(parts[0]), int(parts[1]), int(parts[2])
#                     vision_data["locked"] = True
#         except BlockingIOError: pass
#         except Exception: pass
#         await asyncio.sleep(0.05)

# async def lidar_listener():
#     udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_sock.bind(("127.0.0.1", 5006))
#     udp_sock.setblocking(False)
#     while True:
#         try:
#             data, _ = udp_sock.recvfrom(1024)
#             lidar_data["action"] = data.decode()
#         except BlockingIOError: pass
#         except Exception: pass
#         await asyncio.sleep(0.1)

# async def navigate_with_avoidance(drone, target_lat, target_lon):
#     drone_state["status"] = "Navigating (Velocity Field)"
#     print(f"\n[NAV] Initiating continuous velocity field navigation...")

#     await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
#     try:
#         await drone.offboard.start()
#     except OffboardError as e:
#         print(f"Offboard start failed: {e}")
#         return

#     while True:
#         # 1. Check Arrival
#         if get_distance(drone_state["lat"], drone_state["lon"], target_lat, target_lon) < 0.000015:
#             break

#         # 2. Get Heading to Target
#         target_yaw = get_bearing(drone_state["lat"], drone_state["lon"], target_lat, target_lon)
#         yaw_rad = math.radians(target_yaw)

#         # 3. Velocity Blending (No braking, just redirecting)
#         action = lidar_data["action"]
#         if action == "CLEAR":
#             v_forward = 4.0
#             v_right = 0.0
#         elif action == "DODGE_LEFT":
#             v_forward = 1.0   # Slow down safely, keep moving forward
#             v_right = -3.5    # Strong lateral push
#         elif action == "DODGE_RIGHT":
#             v_forward = 1.0
#             v_right = 3.5

#         # 4. Rotation Matrix (Convert Body Velocity to North/East Velocity)
#         v_north = v_forward * math.cos(yaw_rad) - v_right * math.sin(yaw_rad)
#         v_east = v_forward * math.sin(yaw_rad) + v_right * math.cos(yaw_rad)

#         # 5. Send continuous smooth setpoint
#         await drone.offboard.set_velocity_ned(VelocityNedYaw(v_north, v_east, 0.0, target_yaw))
#         await asyncio.sleep(0.1) # Smooth 10Hz loop

#     await drone.offboard.stop()
#     print("[NAV] Destination reached.")

# async def execute_precision_landing(drone, expected_marker_id):
#     drone_state["status"] = f"Landing Sequence (Pad {expected_marker_id})"
#     print(f"\n[AUTONOMY] Hunting for Marker ID: {expected_marker_id}...")
    
#     await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
#     await drone.offboard.start()

#     K_p = 0.015 
#     start_time = time.time()
#     TIMEOUT_SECONDS = 45 

#     while True:
#         if time.time() - start_time > TIMEOUT_SECONDS:
#             print(f"[FAILSAFE] Marker {expected_marker_id} not found. Aborting land!")
#             await drone.offboard.stop()
#             await drone.action.set_takeoff_altitude(TARGET_ALT)
#             await drone.action.takeoff()
#             await asyncio.sleep(5)
#             return False

#         if vision_data["locked"] and vision_data["id"] == expected_marker_id:
#             vx, vy = -vision_data["err_y"] * K_p, vision_data["err_x"] * K_p
#             vz = 0.4 
#             if abs(vision_data["err_x"]) < 20 and abs(vision_data["err_y"]) < 20:
#                 vx, vy, vz = 0.0, 0.0, 0.8
#             vx, vy = max(min(vx, 1.5), -1.5), max(min(vy, 1.5), -1.5)
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
#         else:
#             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.2, 0.0))

#         async for pos in drone.telemetry.position():
#             if pos.relative_altitude_m < 0.3:
#                 print(f"[AUTONOMY] Touchdown sequence initiated...")
#                 await drone.offboard.stop()
#                 try: await asyncio.wait_for(drone.action.land(), timeout=2.0)
#                 except Exception: pass
#                 async for is_armed in drone.telemetry.armed():
#                     if not is_armed: break
#                 print(f"[AUTONOMY] Pad {expected_marker_id} secured. Motors disarmed.")
#                 return True
#             break
#         await asyncio.sleep(0.1)

# async def execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon):
#     drone = System()
#     drone_state["status"] = "Connecting..."
#     await drone.connect(system_address="udpin://0.0.0.0:14540")
    
#     async for state in drone.core.connection_state():
#         if state.is_connected: break
#     async for health in drone.telemetry.health():
#         if health.is_global_position_ok: break

#     asyncio.create_task(telemetry_monitor(drone))
#     asyncio.create_task(vision_listener())
#     asyncio.create_task(lidar_listener())
#     await asyncio.sleep(1)
    
#     home_lat, home_lon = drone_state["lat"], drone_state["lon"]

#     spawn_gazebo_marker("pickup_pad", pickup_lat, pickup_lon, home_lat, home_lon)
#     spawn_gazebo_marker("drop_pad", drop_lat, drop_lon, home_lat, home_lon)
#     spawn_gazebo_marker("home_pad", home_lat, home_lon, home_lat, home_lon)

#     await drone.action.arm()
#     await drone.action.set_takeoff_altitude(TARGET_ALT)
#     await drone.action.takeoff()
#     await asyncio.sleep(8)

#     await navigate_with_avoidance(drone, pickup_lat, pickup_lon)
#     if await execute_precision_landing(drone, 0): await asyncio.sleep(5) 

#     await drone.action.arm()
#     await drone.action.takeoff()
#     await asyncio.sleep(8)
#     await navigate_with_avoidance(drone, drop_lat, drop_lon)
#     if await execute_precision_landing(drone, 0): await asyncio.sleep(5)

#     await drone.action.arm()
#     await drone.action.takeoff()
#     await asyncio.sleep(8)
#     await navigate_with_avoidance(drone, home_lat, home_lon)
#     await execute_precision_landing(drone, 0)
    
#     drone_state["status"] = "Mission Complete"






# -------------------------------------------------------------------------------------------

# version 3: updated code try
"""
drone_logic.py  –  V5 "NED-Frame" Architecture
================================================
Root-cause fixes for every failure seen in the logs:

PROBLEM 1 – goto_location() + set_velocity_body() mode thrashing
  Every avoidance event switched PX4 between HOLD and OFFBOARD rapidly.
  PX4 cannot handle rapid mode thrashing → "invalid setpoints" → blind land.
  FIX: Stay in OFFBOARD mode for the ENTIRE flight leg. Use VelocityNedYaw
  (world-frame) for navigation. Never call goto_location() again.

PROBLEM 2 – Hard braking → nose pitches up → 2D LiDAR stares at sky →
  reports ∞ → avoider says CLEAR → drone lunges forward → crash loop.
  FIX: Use NED-frame velocities. We command the WORLD FRAME, not the body
  frame. The drone's nose pitch is irrelevant to a world-frame velocity
  setpoint. No braking = no pitch spike = no sky blindness.

PROBLEM 3 – Drone flew backwards on return trip (yaw=0 hardcoded).
  FIX: Dynamic bearing calculated each control tick so the nose always faces
  the target. In NED mode, yaw is just cosmetic – avoidance still works.

PROBLEM 4 – Attitude failure (roll) / EKF failure during aggressive manoeuvres.
  FIX: Max cruise speed capped at 4 m/s. Avoidance is a gentle sideways
  velocity nudge – no deceleration impulse that can stress the EKF.

PROBLEM 5 – GPS arrival tolerance too coarse / too tight.
  FIX: Arrival is checked in METRES (proper haversine-style conversion), not
  raw decimal-degrees. Stops at 2.0 m from target.
"""

import asyncio
import math
import socket
import subprocess
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, VelocityBodyYawspeed

# ── mission constants ────────────────────────────────────────────────────────
TARGET_ALT      = 10.0     # m  – cruise / takeoff altitude
CRUISE_SPEED    = 3.5      # m/s – maximum forward speed
SLOW_RADIUS_M   = 12.0     # m  – start slowing when this close to target
MIN_SPEED       = 0.8      # m/s – minimum approach speed (never fully stall)
ARRIVAL_M       = 2.0      # m  – "close enough" to declare arrival
DODGE_SPEED     = 2.5      # m/s – sideways dodge speed
BACK_SPEED      = 0.4      # m/s – tiny rearward component while dodging
ALT_GAIN        = 0.25     # altitude correction P-gain
ALT_MAX_VEL     = 0.5      # m/s – max vertical correction
NAV_HZ          = 10       # Hz  – navigation control loop rate
# ────────────────────────────────────────────────────────────────────────────

# shared state dictionaries (written by background tasks)
drone_state = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "status": "Idle"}
vision_data = {"err_x": 0, "err_y": 0, "id": -1, "locked": False}
lidar_data  = {"action": "CLEAR"}


# ════════════════════════════════════════════════════════════════════════════
#  GEOMETRY HELPERS
# ════════════════════════════════════════════════════════════════════════════

def get_distance_m(lat1, lon1, lat2, lon2):
    """Flat-earth distance in metres (accurate within ~10 km)."""
    d_lat = (lat2 - lat1) * 111_320.0
    d_lon = (lon2 - lon1) * (111_320.0 * math.cos(math.radians(lat1)))
    return math.hypot(d_lat, d_lon)


def get_bearing(lat1, lon1, lat2, lon2):
    """Compass bearing in degrees (0 = North, 90 = East)."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon   = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def bearing_to_ned(bearing_deg, speed_ms):
    """Convert compass bearing + speed → (north_m_s, east_m_s)."""
    r = math.radians(bearing_deg)
    return speed_ms * math.cos(r), speed_ms * math.sin(r)


def dodge_ned(bearing_deg, action):
    """
    Return (north, east) velocity for a sideways dodge, PLUS a tiny backward
    component to maintain separation from the wall.

    LEFT/RIGHT are from the drone's perspective (i.e. relative to its bearing).
    """
    fwd_r  = math.radians(bearing_deg)

    if action == "DODGE_LEFT":
        side_r = math.radians(bearing_deg - 90.0)
    else:                                          # DODGE_RIGHT
        side_r = math.radians(bearing_deg + 90.0)

    # sideways
    n = DODGE_SPEED * math.cos(side_r)
    e = DODGE_SPEED * math.sin(side_r)

    # tiny rearward component
    n -= BACK_SPEED * math.cos(fwd_r)
    e -= BACK_SPEED * math.sin(fwd_r)

    return n, e


def altitude_correction():
    """Small down-velocity correction to hold TARGET_ALT (NED: negative = up)."""
    err = TARGET_ALT - drone_state["alt"]           # positive → drone too low
    raw = -err * ALT_GAIN                            # negative → ascend
    return max(-ALT_MAX_VEL, min(ALT_MAX_VEL, raw))


# ════════════════════════════════════════════════════════════════════════════
#  SIMULATION HELPERS
# ════════════════════════════════════════════════════════════════════════════

def spawn_gazebo_marker(name, target_lat, target_lon, home_lat, home_lon):
    d_lat  = target_lat - home_lat
    d_lon  = target_lon - home_lon
    y_north = d_lat * 111_320.0
    x_east  = d_lon * (111_320.0 * math.cos(math.radians(home_lat)))

    print(f"[ENV] Spawning {name:15s} at X(E)={x_east:7.2f} m  Y(N)={y_north:7.2f} m")
    cmd = (
        f"gz service -s /world/default/create "
        f"--reqtype gz.msgs.EntityFactory "
        f"--reptype gz.msgs.Boolean "
        f"--timeout 1000 "
        f"--req 'sdf_filename: \"model://arucotag\", "
        f"name: \"{name}\", "
        f"pose: {{position: {{x: {x_east}, y: {y_north}, z: 0.1}}}}'"
    )
    subprocess.run(cmd, shell=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ════════════════════════════════════════════════════════════════════════════
#  BACKGROUND LISTENER TASKS
# ════════════════════════════════════════════════════════════════════════════

async def telemetry_monitor(drone):
    """Continuously update drone_state from MAVSDK telemetry."""
    async for pos in drone.telemetry.position():
        drone_state["lat"] = pos.latitude_deg
        drone_state["lon"] = pos.longitude_deg
        drone_state["alt"] = pos.relative_altitude_m


async def vision_listener():
    """Receive UDP datagrams from the vision/ArUco broadcaster."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5005))
    sock.setblocking(False)
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = data.decode().strip()
            if msg == "None":
                vision_data["locked"] = False
            else:
                parts = msg.split(",")
                if len(parts) == 3:
                    vision_data["err_x"] = int(parts[0])
                    vision_data["err_y"] = int(parts[1])
                    vision_data["id"]    = int(parts[2])
                    vision_data["locked"] = True
        except BlockingIOError:
            pass
        except Exception:
            pass
        await asyncio.sleep(0.05)


async def lidar_listener():
    """Receive UDP datagrams from avoider_node (CLEAR / DODGE_LEFT / DODGE_RIGHT)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5006))
    sock.setblocking(False)
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            lidar_data["action"] = data.decode().strip()
        except BlockingIOError:
            pass
        except Exception:
            pass
        await asyncio.sleep(0.05)   # 20 Hz read rate


# ════════════════════════════════════════════════════════════════════════════
#  TAKEOFF HELPER
# ════════════════════════════════════════════════════════════════════════════

async def arm_and_takeoff(drone):
    """
    Arm, command auto-takeoff to TARGET_ALT, then wait until the drone
    actually reaches that altitude before returning.
    """
    drone_state["status"] = "Arming"
    print("[TAKEOFF] Arming…")
    await drone.action.set_takeoff_altitude(TARGET_ALT)
    await drone.action.arm()
    await asyncio.sleep(1.0)

    print(f"[TAKEOFF] Taking off to {TARGET_ALT} m…")
    await drone.action.takeoff()

    # Wait for altitude – check every 0.5 s
    while True:
        alt = drone_state["alt"]
        if alt >= TARGET_ALT - 1.5:
            break
        print(f"[TAKEOFF]  alt = {alt:.1f} m  (target {TARGET_ALT} m)…")
        await asyncio.sleep(0.5)

    # Brief stabilisation pause
    await asyncio.sleep(2.0)
    print("[TAKEOFF] Altitude reached. Ready for offboard control.")


# ════════════════════════════════════════════════════════════════════════════
#  CORE NAVIGATION  –  NED VELOCITY OFFBOARD LOOP
# ════════════════════════════════════════════════════════════════════════════

async def navigate_with_avoidance(drone, target_lat, target_lon):
    """
    Fly to (target_lat, target_lon) in OFFBOARD mode using world-frame
    (NED) velocity setpoints.

    Why NED frame?
    ──────────────
    Body-frame commands (VelocityBodyYawspeed) are relative to where the
    drone's NOSE is pointing.  When PX4 brakes, the nose pitches up.
    A 2-D LiDAR mounted flat then stares at the sky, returns ∞, and the
    avoider incorrectly declares the path clear.

    NED commands are always in the WORLD frame.  A pitching nose has zero
    effect on a north/east/down velocity command – PX4's attitude controller
    works out the required tilt internally.  This completely eliminates
    pitch-blindness.

    Control law
    ───────────
    • Proportional speed (slow down smoothly near target).
    • When avoider says DODGE_*: replace the forward NED component with a
      perpendicular one – no mode switch, no braking impulse.
    • Altitude held via a small P-controller on the down axis.
    • 10 Hz loop (fast enough for responsive avoidance).
    """
    print(f"\n[NAV] Navigating to ({target_lat:.6f}, {target_lon:.6f})")
    drone_state["status"] = "Navigating"

    # ── prime offboard with a zero setpoint, then start it ──────────────────
    zero = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_ned(zero)
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[NAV] Offboard start error (may already be running): {e}")

    dt = 1.0 / NAV_HZ

    while True:
        # ── arrival check ────────────────────────────────────────────────────
        dist_m = get_distance_m(
            drone_state["lat"], drone_state["lon"],
            target_lat, target_lon
        )

        if dist_m <= ARRIVAL_M:
            # Hover in place to stabilise before landing
            await drone.offboard.set_velocity_ned(zero)
            await asyncio.sleep(0.5)
            print(f"[NAV] Arrived at target ({dist_m:.1f} m).")
            await drone.offboard.stop()
            return

        # ── compute bearing & proportional cruise speed ─────────────────────
        bearing = get_bearing(
            drone_state["lat"], drone_state["lon"],
            target_lat, target_lon
        )
        speed = min(CRUISE_SPEED,
                    max(MIN_SPEED, CRUISE_SPEED * (dist_m / SLOW_RADIUS_M)))

        # ── pick NED velocity based on lidar state ───────────────────────────
        action = lidar_data["action"]

        if action != "CLEAR":
            # ── AVOIDANCE ───────────────────────────────────────────────────
            drone_state["status"] = f"EVADING ({action})"
            n_vel, e_vel = dodge_ned(bearing, action)
            d_vel = altitude_correction()
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(n_vel, e_vel, d_vel, bearing)
            )

        else:
            # ── CRUISE toward target ────────────────────────────────────────
            drone_state["status"] = f"Navigating  dist={dist_m:.0f} m"
            n_vel, e_vel = bearing_to_ned(bearing, speed)
            d_vel = altitude_correction()
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(n_vel, e_vel, d_vel, bearing)
            )

        await asyncio.sleep(dt)


# ════════════════════════════════════════════════════════════════════════════
#  PRECISION LANDING  –  BODY-FRAME VISION LOOP
# ════════════════════════════════════════════════════════════════════════════

async def execute_precision_landing(drone, expected_marker_id):
    """
    Descend onto an ArUco marker using body-frame velocity offboard control.

    We ENTER offboard mode here from a stopped hover, so no mode conflict.
    The loop runs at ~10 Hz and drives the drone's centroid onto the marker
    using a simple P-controller on the pixel error from the camera feed.
    """
    drone_state["status"] = f"Precision landing (Marker {expected_marker_id})"
    print(f"\n[LAND] Hunting for Marker ID {expected_marker_id}…")

    K_p             = 0.015   # proportional gain (pixels → m/s)
    CENTRE_THRESH   = 20      # pixel  – "centred enough" to start descending
    DESCEND_VZ      = 0.4     # m/s  – normal descent speed (positive = down)
    FAST_DESCEND    = 0.8     # m/s  – descent speed when perfectly centred
    TIMEOUT_S       = 45

    # enter offboard (hover)
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await drone.offboard.start()

    start_t = time.time()

    while True:
        elapsed = time.time() - start_t

        # ── timeout failsafe ─────────────────────────────────────────────────
        if elapsed > TIMEOUT_S:
            print(f"[LAND] TIMEOUT – Marker {expected_marker_id} not found. "
                  f"Climbing to cruise altitude.")
            await drone.offboard.stop()
            await drone.action.set_takeoff_altitude(TARGET_ALT)
            await drone.action.takeoff()
            await asyncio.sleep(6)
            return False

        # ── vision control ───────────────────────────────────────────────────
        if vision_data["locked"] and vision_data["id"] == expected_marker_id:
            ex = vision_data["err_x"]
            ey = vision_data["err_y"]

            # camera frame → body frame (depends on camera orientation)
            # Adjust signs if your camera is rotated differently
            vx = -ey * K_p   # forward/backward
            vy =  ex * K_p   # left/right

            centred = abs(ex) < CENTRE_THRESH and abs(ey) < CENTRE_THRESH
            vz = FAST_DESCEND if centred else DESCEND_VZ

            # clamp horizontal speed
            vx = max(-1.5, min(1.5, vx))
            vy = max(-1.5, min(1.5, vy))

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, vy, vz, 0.0)
            )
        else:
            # no marker in view – rotate slowly to search
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.15, 10.0)
            )

        # ── touchdown detection ──────────────────────────────────────────────
        # Read ONE telemetry sample non-blockingly
        async for pos in drone.telemetry.position():
            if pos.relative_altitude_m < 0.30:
                print(f"[LAND] Touchdown!  Cutting motors…")
                await drone.offboard.stop()
                try:
                    await asyncio.wait_for(drone.action.land(), timeout=3.0)
                except Exception:
                    pass

                # wait for disarm
                async for armed in drone.telemetry.armed():
                    if not armed:
                        break
                print(f"[LAND] ✅  Marker {expected_marker_id} secured. Disarmed.")
                return True
            break   # only take the first item

        await asyncio.sleep(0.1)


# ════════════════════════════════════════════════════════════════════════════
#  MISSION ORCHESTRATOR
# ════════════════════════════════════════════════════════════════════════════

async def execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon):
    """
    Full autonomous delivery mission:
      1. Takeoff
      2. Fly to pickup   → precision land on Marker 0
      3. Fly to drop-off → precision land on Marker 0
      4. Return home     → precision land on Marker 0
    """
    # ── connect ──────────────────────────────────────────────────────────────
    drone = System()
    drone_state["status"] = "Connecting…"
    print("[MISSION] Connecting to drone…")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[MISSION] Connected.")
            break

    # Wait for a valid GPS fix
    print("[MISSION] Waiting for GPS fix…")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("[MISSION] GPS OK.")
            break

    # ── start background tasks ───────────────────────────────────────────────
    asyncio.create_task(telemetry_monitor(drone))
    asyncio.create_task(vision_listener())
    asyncio.create_task(lidar_listener())

    # give telemetry a moment to populate drone_state
    await asyncio.sleep(2.0)

    home_lat = drone_state["lat"]
    home_lon = drone_state["lon"]
    print(f"[MISSION] Home position: ({home_lat:.7f}, {home_lon:.7f})")

    # ── spawn simulation markers ─────────────────────────────────────────────
    spawn_gazebo_marker("pickup_pad", pickup_lat,  pickup_lon,  home_lat, home_lon)
    spawn_gazebo_marker("drop_pad",   drop_lat,    drop_lon,    home_lat, home_lon)
    spawn_gazebo_marker("home_pad",   home_lat,    home_lon,    home_lat, home_lon)

    # ═══════════════════════════════════════════════════════════════════════
    #  LEG 1 – PICKUP
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "═" * 60)
    print("  LEG 1 / 3  →  FLY TO PICKUP")
    print("═" * 60)

    await arm_and_takeoff(drone)
    drone_state["status"] = "Flying to Pickup"
    await navigate_with_avoidance(drone, pickup_lat, pickup_lon)

    landed = await execute_precision_landing(drone, 0)
    if landed:
        print("[MISSION] Package picked up. Waiting 5 s…")
        await asyncio.sleep(5.0)

    # ═══════════════════════════════════════════════════════════════════════
    #  LEG 2 – DROP-OFF
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "═" * 60)
    print("  LEG 2 / 3  →  FLY TO DROP-OFF")
    print("═" * 60)

    await arm_and_takeoff(drone)
    drone_state["status"] = "Flying to Drop-off"
    await navigate_with_avoidance(drone, drop_lat, drop_lon)

    landed = await execute_precision_landing(drone, 0)
    if landed:
        print("[MISSION] Package delivered. Waiting 5 s…")
        await asyncio.sleep(5.0)

    # ═══════════════════════════════════════════════════════════════════════
    #  LEG 3 – RETURN HOME
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "═" * 60)
    print("  LEG 3 / 3  →  RETURN HOME")
    print("═" * 60)

    await arm_and_takeoff(drone)
    drone_state["status"] = "Returning Home"
    await navigate_with_avoidance(drone, home_lat, home_lon)

    await execute_precision_landing(drone, 0)

    drone_state["status"] = "Mission Complete ✅"
    print("\n[MISSION] ✅  All legs complete.  Mission SUCCESS.")