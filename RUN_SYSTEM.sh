#!/bin/bash
# Complete system startup script
# Run this to start the entire drone delivery system

echo "======================================"
echo "   DRONE DELIVERY SYSTEM STARTUP"
echo "======================================"
echo ""
echo "This will open 5 terminal windows."
echo "Press Ctrl+C in THIS terminal to stop all processes."
echo ""
read -p "Press ENTER to start..."

# Kill any existing processes
pkill -f "px4_sitl"
pkill -f "vision_bridge.py"
pkill -f "avoider_node.py"
pkill -f "ros_gz_bridge"
pkill -f "fleet_dispatch"
sleep 2

echo ""
echo "Starting processes..."
echo ""

# Terminal 1: PX4 SITL + Gazebo
gnome-terminal --title="1. PX4 SITL + Gazebo" -- bash -c "
cd ~/PX4-Autopilot
echo '===== TERMINAL 1: PX4 SITL + GAZEBO ====='
echo 'Wait for: [mavlink] mode: Onboard'
echo ''
make px4_sitl gz_x500
" &
sleep 5

# Terminal 2: Obstacle Memory Service
gnome-terminal --title="2. Obstacle Memory Service" -- bash -c "
cd ~/DroneProgram
echo '===== TERMINAL 2: OBSTACLE MEMORY SERVICE ====='
echo 'Fleet-wide obstacle sharing on http://127.0.0.1:5050'
echo ''
python3 obstacle_memory_service.py
" &
sleep 2

# Terminal 3: Vision Bridge
gnome-terminal --title="3. Vision Bridge" -- bash -c "
cd ~/DroneProgram
echo '===== TERMINAL 3: VISION BRIDGE ====='
echo 'Wait for: Drone Downward Camera window'
echo ''
python3 vision_bridge.py
" &
sleep 2

# Terminal 4: ROS2 LiDAR Bridge (CRITICAL)
gnome-terminal --title="4. ROS2 LiDAR Bridge" -- bash -c "
echo '===== TERMINAL 4: ROS2 LIDAR BRIDGE ====='
echo 'CRITICAL: Required for obstacle avoidance'
echo ''
ros2 run ros_gz_bridge parameter_bridge /lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
" &
sleep 2

# Terminal 5: Avoider Node (CRITICAL)
gnome-terminal --title="5. Avoider Node" -- bash -c "
cd ~/DroneProgram
echo '===== TERMINAL 5: AVOIDER NODE ====='
echo 'CRITICAL: Required for obstacle avoidance'
echo ''
python3 avoider_node.py
" &
sleep 2

# Terminal 6: Fleet Dispatch
gnome-terminal --title="6. Fleet Dispatch" -- bash -c "
cd ~/DroneProgram/fleet_dispatch
echo '===== TERMINAL 6: FLEET DISPATCH ====='
echo 'Open browser: http://localhost:5000'
echo ''
python3 app.py
" &

echo ""
echo "✅ All processes started!"
echo ""
echo "📋 CHECKLIST:"
echo "  1. Terminal 1: Wait for 'mavlink mode: Onboard'"
echo "  2. Terminal 2: Obstacle memory on http://127.0.0.1:5050"
echo "  3. Terminal 3: Camera window should open"
echo "  4. Terminal 4: Should show ROS2 bridge running"
echo "  5. Terminal 5: Should show 'Avoider V4' startup"
echo "  6. Open browser: http://localhost:5000"
echo ""
echo "🗺️  FLEET OBSTACLE SHARING:"
echo "  • View obstacles: http://127.0.0.1:5050/obstacles"
echo "  • Statistics: http://127.0.0.1:5050/stats"
echo "  • Each drone shares obstacles with the entire fleet"
echo ""
echo "⚠️  IMPORTANT: Wait 10 seconds for all systems to initialize"
echo ""
echo "Press Ctrl+C here to stop all processes"

# Wait and cleanup on exit
trap "echo 'Stopping all processes...'; pkill -f px4_sitl; pkill -f vision_bridge; pkill -f avoider_node; pkill -f ros_gz_bridge; pkill -f fleet_dispatch; exit" INT
wait
