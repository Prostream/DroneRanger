#!/bin/bash
# Script to run Gazebo simulation with PX4 and drone autonomy

set -e

echo "🌍 Starting DroneRanger Simulation Environment..."

# Configuration
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
GAZEBO_WORLD="${1:-empty}"
MISSION="${2:-simple_takeoff}"

# Check if PX4 directory exists
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ Error: PX4-Autopilot directory not found at: $PX4_DIR"
    echo "Please set PX4_DIR environment variable or install PX4-Autopilot"
    exit 1
fi

# Source ROS2 and workspace
echo "📦 Sourcing ROS2 and workspace..."
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "⚠️ Warning: Workspace not built. Run build_project.sh first"
fi

echo "🚁 Starting PX4 SITL with Gazebo..."
cd "$PX4_DIR"

# Start PX4 SITL in background
make px4_sitl gazebo-classic_iris_empty &
PX4_PID=$!

echo "⏳ Waiting for PX4 to start..."
sleep 10

echo "🎮 Starting mission: $MISSION"
cd - > /dev/null

# Start the mission
case $MISSION in
    "simple_takeoff")
        ros2 launch drone_autonomy simple_takeoff.launch.py
        ;;
    *)
        echo "🎯 Available missions: simple_takeoff"
        ros2 run drone_autonomy simple_takeoff
        ;;
esac

# Cleanup
echo "🧹 Cleaning up..."
kill $PX4_PID 2>/dev/null || true
echo "✅ Simulation ended"