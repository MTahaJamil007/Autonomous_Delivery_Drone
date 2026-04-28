# Drone Program Runbook

This document lists the startup sequence for all components.

## Terminal 1: Physics Engine and Flight Controller

Launch Gazebo Harmonic and PX4 SITL.

1. Open a new terminal.
2. Run:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Verification:
- Gazebo GUI opens.
- Drone model spawns.
- Terminal shows: `INFO [commander] Ready for takeoff!`

## Terminal 2: Vision Bridge (OpenCV)

Run ArUco detection from the downward camera feed.

1. Open a new terminal.
2. Activate the vision environment.
3. Start the vision bridge.

```bash
cd ~/DroneProgram
source vision_env/bin/activate
python vision_bridge.py
```

Verification:
- Camera window appears.
- Terminal shows: `Vision Broadcaster Online on Port 5005!`

## Terminal 3: ROS 2 Middleware Bridge

Bridge Gazebo LiDAR output to ROS 2 `LaserScan`.

1. Open a new terminal.
2. Run:

```bash
ros2 run ros_gz_bridge parameter_bridge /lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

Verification:
- Terminal shows: `Creating GZ->ROS Bridge`
- Then remains mostly silent (normal behavior).

## Terminal 4: ROS 2 Spatial Reflex System (Avoidance Node)

Start obstacle avoidance logic.

1. Open a new terminal.
2. Source the ROS 2 workspace.
3. Run the avoider node.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run obstacle_avoider avoider
```

Verification:
- Terminal shows: `ROS 2 Spatial Reflex System Online! Broadcasting on Port 5006...`

## Terminal 5: Command Center and Drone Logic

Start the FastAPI backend and web command center.

1. Open a new terminal.
2. Run:

```bash
cd ~/DroneProgram/drone_web
uvicorn app:app --reload --port 5000
```

Verification:
- Terminal shows: `Application startup complete.`
- Server is available at: `http://127.0.0.1:5000`

## Battery Override Commands

Use these in PX4 shell when needed:

```bash
param set COM_LOW_BAT_ACT -1
param set BAT_LOW_THR 0
param set BAT_CRIT_THR 0
param set BAT_EMER_THR 0
param save
```

Then confirm (or reapply):

```bash
param set COM_LOW_BAT_ACT -1
```