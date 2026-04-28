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
import asyncio
import math
import socket
import subprocess
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw

TARGET_ALT = 10
drone_state = {"lat": 0.0, "lon": 0.0, "status": "Idle"}

vision_data = {"err_x": 0, "err_y": 0, "id": -1, "locked": False}
lidar_data = {"action": "CLEAR"}

def get_distance(lat1, lon1, lat2, lon2):
    return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

def get_bearing(lat1, lon1, lat2, lon2):
    lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
    lon_diff = math.radians(lon2 - lon1)
    x = math.sin(lon_diff) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon_diff))
    initial_bearing = math.atan2(x, y)
    return (math.degrees(initial_bearing) + 360) % 360

def spawn_gazebo_marker(name, target_lat, target_lon, home_lat, home_lon):
    d_lat = target_lat - home_lat
    d_lon = target_lon - home_lon
    y_north = d_lat * 111320.0
    x_east = d_lon * (111320.0 * math.cos(math.radians(home_lat)))
    print(f"[ENVIRONMENT] Spawning {name} at Gazebo X: {x_east:.2f}, Y: {y_north:.2f}")
    cmd = f"gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"model://arucotag\", name: \"{name}\", pose: {{position: {{x: {x_east}, y: {y_north}, z: 0.1}}}}'"
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

async def telemetry_monitor(drone):
    async for position in drone.telemetry.position():
        drone_state["lat"] = position.latitude_deg
        drone_state["lon"] = position.longitude_deg

async def vision_listener():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("127.0.0.1", 5005))
    udp_sock.setblocking(False)
    while True:
        try:
            data, _ = udp_sock.recvfrom(1024)
            msg = data.decode()
            if msg == "None":
                vision_data["locked"] = False
            else:
                parts = msg.split(',')
                if len(parts) == 3:
                    vision_data["err_x"], vision_data["err_y"], vision_data["id"] = int(parts[0]), int(parts[1]), int(parts[2])
                    vision_data["locked"] = True
        except BlockingIOError: pass
        except Exception: pass
        await asyncio.sleep(0.05)

async def lidar_listener():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("127.0.0.1", 5006))
    udp_sock.setblocking(False)
    while True:
        try:
            data, _ = udp_sock.recvfrom(1024)
            lidar_data["action"] = data.decode()
        except BlockingIOError: pass
        except Exception: pass
        await asyncio.sleep(0.1)

async def navigate_with_avoidance(drone, target_lat, target_lon):
    drone_state["status"] = "Navigating (Velocity Field)"
    print(f"\n[NAV] Initiating continuous velocity field navigation...")

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    while True:
        # 1. Check Arrival
        if get_distance(drone_state["lat"], drone_state["lon"], target_lat, target_lon) < 0.000015:
            break

        # 2. Get Heading to Target
        target_yaw = get_bearing(drone_state["lat"], drone_state["lon"], target_lat, target_lon)
        yaw_rad = math.radians(target_yaw)

        # 3. Velocity Blending (No braking, just redirecting)
        action = lidar_data["action"]
        if action == "CLEAR":
            v_forward = 4.0
            v_right = 0.0
        elif action == "DODGE_LEFT":
            v_forward = 1.0   # Slow down safely, keep moving forward
            v_right = -3.5    # Strong lateral push
        elif action == "DODGE_RIGHT":
            v_forward = 1.0
            v_right = 3.5

        # 4. Rotation Matrix (Convert Body Velocity to North/East Velocity)
        v_north = v_forward * math.cos(yaw_rad) - v_right * math.sin(yaw_rad)
        v_east = v_forward * math.sin(yaw_rad) + v_right * math.cos(yaw_rad)

        # 5. Send continuous smooth setpoint
        await drone.offboard.set_velocity_ned(VelocityNedYaw(v_north, v_east, 0.0, target_yaw))
        await asyncio.sleep(0.1) # Smooth 10Hz loop

    await drone.offboard.stop()
    print("[NAV] Destination reached.")

async def execute_precision_landing(drone, expected_marker_id):
    drone_state["status"] = f"Landing Sequence (Pad {expected_marker_id})"
    print(f"\n[AUTONOMY] Hunting for Marker ID: {expected_marker_id}...")
    
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    K_p = 0.015 
    start_time = time.time()
    TIMEOUT_SECONDS = 45 

    while True:
        if time.time() - start_time > TIMEOUT_SECONDS:
            print(f"[FAILSAFE] Marker {expected_marker_id} not found. Aborting land!")
            await drone.offboard.stop()
            await drone.action.set_takeoff_altitude(TARGET_ALT)
            await drone.action.takeoff()
            await asyncio.sleep(5)
            return False

        if vision_data["locked"] and vision_data["id"] == expected_marker_id:
            vx, vy = -vision_data["err_y"] * K_p, vision_data["err_x"] * K_p
            vz = 0.4 
            if abs(vision_data["err_x"]) < 20 and abs(vision_data["err_y"]) < 20:
                vx, vy, vz = 0.0, 0.0, 0.8
            vx, vy = max(min(vx, 1.5), -1.5), max(min(vy, 1.5), -1.5)
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        else:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.2, 0.0))

        async for pos in drone.telemetry.position():
            if pos.relative_altitude_m < 0.3:
                print(f"[AUTONOMY] Touchdown sequence initiated...")
                await drone.offboard.stop()
                try: await asyncio.wait_for(drone.action.land(), timeout=2.0)
                except Exception: pass
                async for is_armed in drone.telemetry.armed():
                    if not is_armed: break
                print(f"[AUTONOMY] Pad {expected_marker_id} secured. Motors disarmed.")
                return True
            break
        await asyncio.sleep(0.1)

async def execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon):
    drone = System()
    drone_state["status"] = "Connecting..."
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected: break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok: break

    asyncio.create_task(telemetry_monitor(drone))
    asyncio.create_task(vision_listener())
    asyncio.create_task(lidar_listener())
    await asyncio.sleep(1)
    
    home_lat, home_lon = drone_state["lat"], drone_state["lon"]

    spawn_gazebo_marker("pickup_pad", pickup_lat, pickup_lon, home_lat, home_lon)
    spawn_gazebo_marker("drop_pad", drop_lat, drop_lon, home_lat, home_lon)
    spawn_gazebo_marker("home_pad", home_lat, home_lon, home_lat, home_lon)

    await drone.action.arm()
    await drone.action.set_takeoff_altitude(TARGET_ALT)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await navigate_with_avoidance(drone, pickup_lat, pickup_lon)
    if await execute_precision_landing(drone, 0): await asyncio.sleep(5) 

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)
    await navigate_with_avoidance(drone, drop_lat, drop_lon)
    if await execute_precision_landing(drone, 0): await asyncio.sleep(5)

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)
    await navigate_with_avoidance(drone, home_lat, home_lon)
    await execute_precision_landing(drone, 0)
    
    drone_state["status"] = "Mission Complete"