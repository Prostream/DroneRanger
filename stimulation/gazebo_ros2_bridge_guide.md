# Gazebo-ROS2 Direct Bridge Complete Installation & Setup Guide

## System Requirements

### Version Matrix
| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 22.04 LTS | WSL2 or Native |
| ROS2 | Humble | LTS Release |
| Gazebo | Sim 8.9.0 (Harmonic) | Comes with PX4 |
| gz-transport | 13.x | For Harmonic |
| PX4-Autopilot | 1.15+ | Latest stable |
| Python | 3.10 | Default with Ubuntu 22.04 |

### Hardware Requirements
- **CPU**: Intel i7/AMD Ryzen 7 or better
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space
- **GPU**: Dedicated GPU recommended for Gazebo visualization

---

## Installation Steps

### Step 1: Install ROS2 Humble

```bash
# Setup sources
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop Full
sudo apt update
sudo apt install ros-humble-desktop-full -y

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Required Message Packages

```bash
# Essential message packages for bridge
sudo apt install -y \
    ros-humble-actuator-msgs \
    ros-humble-gps-msgs \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rosgraph-msgs \
    ros-humble-std-msgs

# Install Python dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep
```

### Step 3: Install PX4 with Gazebo (skip if already have)

```bash
# Create application directory
mkdir -p ~/app
cd ~/app

# Clone PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies (includes Gazebo Harmonic)
bash ./Tools/setup/ubuntu.sh

# Build PX4 with SITL
make px4_sitl gz_x500

# Verify Gazebo installation
gz sim --version
# Should show: Gazebo Sim, version 8.x.x
```

### Step 4: Build Custom ros_gz_bridge

```bash
# Create ROS2 workspace for bridge
mkdir -p ~/ros_gz_ws/src
cd ~/ros_gz_ws/src

# Clone ros_gz repository for Humble
git clone https://github.com/gazebosim/ros_gz.git -b humble

# Install dependencies
cd ~/ros_gz_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the bridge
source /opt/ros/humble/setup.bash
colcon build --packages-select ros_gz_bridge ros_gz_interfaces

# Source the workspace
source install/setup.bash
```

### Step 5: Launch Scripts

# Download the lunching script <start_gz_bridge.sh>

```bash

chmod +x ~/ros_gz_ws/start_gz_bridge.sh

./start_gz_bridge.sh
```


## Configuration

### Bridge Configuration File (Optional)

```yaml
# Save as ~/ros_gz_ws/config/gz_x500_bridge.yaml
# Clock
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# IMU
- ros_topic_name: "/gz/imu"
  gz_topic_name: "/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

# Additional sensors...
```

To use config file instead of command line arguments:
```bash
~/ros_gz_ws/install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge \
  --ros-args -p config_file:=~/ros_gz_ws/config/gz_x500_bridge.yaml
```

---

## Running the System

### Standard Startup Sequence

#### Terminal 1: Start PX4 with Gazebo
```bash
cd ~/app/PX4-Autopilot
make px4_sitl gz_x500

# Wait for these messages:
# INFO [commander] Ready for takeoff!
# INFO [mavlink] MAVLink: accepting TCP connection on port 4560
```

#### Terminal 2: Start Gazebo-ROS2 Bridge
```bash
~/ros_gz_ws/start_gz_bridge.sh

# Should see:
# [INFO] Creating GZ->ROS Bridge: [/clock ...
# [INFO] Creating GZ->ROS Bridge: [/world/default/...
```

#### Terminal 3: Verify and Monitor
```bash
# Check topics
ros2 topic list --no-daemon

# Monitor sensor data
ros2 topic echo /clock --no-daemon
ros2 topic echo /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu --no-daemon
```

### Optional: Start MicroXRCE-DDS Agent (for PX4 data)
```bash
# If you also want PX4 processed data
MicroXRCEAgent udp4 -p 8888
```

---

## Verification & Testing

### Pre-flight Checklist

```bash
# 1. Verify Gazebo version
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# 2. Check Gazebo is running and has topics
gz topic --list | grep sensor
# Should list sensor topics

# 3. Check library dependencies
ldd ~/ros_gz_ws/install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge | grep "not found"
# Should return empty (no missing libraries)

# 4. Verify ROS2 installation
ros2 doctor
# Should show no critical errors
```

### Runtime Verification

```bash
# 1. Check bridge node exists
ros2 node list --no-daemon | grep ros_gz_bridge
# Should show: /ros_gz_bridge

# 2. Verify clock data rate
ros2 topic hz /clock --no-daemon
# Should show ~1000 Hz

# 3. Check sensor data
ros2 topic echo /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu --no-daemon --once
# Should display IMU message

# 4. Run test script
~/ros_gz_ws/test_gz_bridge.sh
```

### Data Flow Test

```bash
# Compare raw Gazebo data with PX4 processed data
# Terminal 1: Raw from Gazebo (via bridge)
ros2 topic echo /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu --no-daemon

# Terminal 2: Processed from PX4 (if MicroXRCE is running)
ros2 topic echo /fmu/out/sensor_combined --no-daemon
```

---

## Architecture Overview

### Data Flow Paths

#### Path 1: Direct Gazebo to ROS2 (This Guide)
```
Gazebo Sensors (Raw Data)
    ↓ [gz-transport13]
gz_ros2_bridge (Direct Bridge)
    ↓ [DDS]
ROS2 Topics (/world/default/...)
    ↓
Your Application
```

#### Path 2: Through PX4 (Alternative)
```
Gazebo Sensors
    ↓ [MAVLink]
PX4 SITL (Processing)
    ↓ [uXRCE-DDS]
MicroXRCE-DDS Agent
    ↓ [DDS]
ROS2 Topics (/fmu/out/*)
```

## Quick Reference

### Essential Commands

```bash
# Start PX4/Gazebo
cd ~/app/PX4-Autopilot && make px4_sitl gz_x500

# Start Bridge
~/ros_gz_ws/start_gz_bridge.sh

# Check Topics
ros2 topic list --no-daemon

# View Clock
ros2 topic echo /clock --no-daemon

# View IMU
ros2 topic echo /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu --no-daemon

# Check Hz
ros2 topic hz /clock --no-daemon

# Test System
~/ros_gz_ws/test_gz_bridge.sh
```


### Critical Notes

1. **ALWAYS use full path** for parameter_bridge executable
2. **Model name is x500_0** not x500 (note the _0 suffix)
3. **No MicroXRCE-DDS Agent needed** for direct bridge
4. **Use --no-daemon** flag with ros2 commands in WSL2
5. **Source order matters**: Always source `/opt/ros/humble/setup.bash` first, then `install/setup.bash`
6. **Gazebo must be running** before starting the bridge
7. **Bridge creates bidirectional connections** by default (@ symbol)
