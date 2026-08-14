# Drone Delivery System - Run Guide

## Prerequisites
- PX4 SITL installed in `~/PX4-Autopilot`
- Gazebo simulator installed
- ROS2 environment sourced
- Python dependencies installed (`pip3 install -r requirements.txt`)

## Quick Start

### Option A: Automated Start (Recommended)
```bash
cd ~/DroneProgram
./RUN_SYSTEM.sh
```
This opens all required terminals automatically.

### Option B: Manual Start

#### 1. Reset Database (First Time / Clean Start)
```bash
cd ~/DroneProgram
python3 reset_fleet_database.py
```

#### 2. Start PX4 SITL with Gazebo (Terminal 1)
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
Wait for: `[mavlink] mode: Onboard`

#### 3. Start Obstacle Memory Service (Terminal 2)
```bash
cd ~/DroneProgram
python3 obstacle_memory_service.py
```

#### 4. Start Vision Bridge (Terminal 3)
```bash
cd ~/DroneProgram
python3 vision_bridge.py
```
Camera window should open.

#### 5. Start ROS2 LiDAR Bridge (Terminal 4)
```bash
ros2 run ros_gz_bridge parameter_bridge /lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

#### 6. Start Avoider Node (Terminal 5)
```bash
cd ~/DroneProgram
python3 avoider_node.py
```
Wait for: `Avoider V4 Online`

#### 7. Start Fleet Dispatch Web Server (Terminal 6)
```bash
cd ~/DroneProgram/fleet_dispatch
python3 app.py
```

#### 8. Access Web Interface
Open browser: `http://localhost:5000`

## System Components

- **PX4 SITL**: Flight controller simulator (MAVLink port 14540)
- **Gazebo**: 3D simulation environment
- **Obstacle Memory Service**: Fleet-wide obstacle sharing (HTTP 5050)
- **Vision Bridge**: ArUco marker tracking (UDP 5005)
- **ROS2 LiDAR Bridge**: Converts Gazebo LiDAR to ROS2 topic
- **Avoider Node**: LiDAR obstacle avoidance logic (UDP 5006)
- **Fleet Dispatch**: Multi-drone mission control (HTTP 5000)

## Troubleshooting

### Reset Database (Safe Option)
```bash
cd ~/DroneProgram
python3 reset_fleet_database.py
```

### Kill Stuck Processes
```bash
pkill -9 -f px4_sitl
pkill -9 -f gz
pkill -9 -f vision_bridge
pkill -9 -f avoider_node
pkill -9 -f ros_gz_bridge
pkill -9 -f fleet_dispatch
```

### Restart Everything
```bash
cd ~/DroneProgram
./RUN_SYSTEM.sh
```

### Check System Status
```bash
# Check if PX4 is running
ps aux | grep px4_sitl

# Check if all Python processes are running
ps aux | grep -E "vision_bridge|avoider_node|obstacle_memory"

# Check if ROS2 bridge is running
ps aux | grep ros_gz_bridge
```

## Mission Flow
1. Drone arms and takes off to 10m altitude
2. Navigates to pickup using NED velocity control
3. Avoids obstacles via LiDAR (sideways dodge maneuvers)
4. Precision lands on ArUco marker ID 0 at pickup
5. Takes off and navigates to dropoff location
6. Precision lands on dropoff marker
7. Takes off and returns to home base
8. Precision lands on home marker
9. Mission complete

## Key Features
- **NED Velocity Control**: World-frame navigation prevents pitch-blindness
- **Time-Hysteresis Avoider**: Prevents ping-pong oscillations at walls
- **Fleet Obstacle Sharing**: Drones share obstacle data via HTTP service
- **Precision Landing**: Vision-guided landing on ArUco markers
- **Multi-Drone Support**: Fleet management with database persistence

## Ports & Services
- **14540**: MAVLink communication (PX4 SITL)
- **5005**: Vision data UDP broadcast (ArUco tracking)
- **5006**: LiDAR avoidance commands UDP
- **5000**: Fleet Dispatch web interface (HTTP)
- **5050**: Obstacle Memory Service (HTTP)

## Important Notes
- Wait 10 seconds after starting all processes before launching missions
- ROS2 LiDAR bridge is CRITICAL - missions will fail without it
- Avoider node must see "Avoider V4 Online" before missions
- Camera window from vision_bridge should display with crosshairs
- All drones in fleet share discovered obstacles automatically
