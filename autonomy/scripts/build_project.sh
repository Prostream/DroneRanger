#!/bin/bash
# Build script for DroneRanger Autonomy project

set -e

echo "🚀 Building DroneRanger Autonomy Project..."

# Check if we're in ROS2 workspace
if [ ! -f "src/drone_autonomy/package.xml" ]; then
    echo "❌ Error: Please run this script from the ROS2 workspace root"
    echo "Expected to find: src/drone_autonomy/package.xml"
    exit 1
fi

# Source ROS2 setup
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Build the project
echo "🔨 Building with colcon..."
colcon build --packages-select drone_autonomy --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Check build status
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "📋 To use the built package, run:"
    echo "    source install/setup.bash"
    echo ""
    echo "🎯 Available executables:"
    echo "    ros2 run drone_autonomy simple_takeoff"
    echo "    ros2 launch drone_autonomy simple_takeoff.launch.py"
else
    echo "❌ Build failed!"
    exit 1
fi