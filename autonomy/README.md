# DroneRanger Autonomy System

A modular drone autonomous flight system based on ROS2 and PX4, providing obstacle avoidance, path planning, and mission execution capabilities.

## Project Features

🚁 **Modular Design**: Clear functional separation for easy maintenance and expansion
🛡️ **Safety First**: Built-in safety monitoring and emergency response mechanisms
🎯 **Mission-Driven**: Supports a variety of flight missions and configurations
🔧 **Easy to Test**: Comprehensive testing framework and simulation environment
📈 **Extensible**: Easy to add new sensors, algorithms, and missions

## System Architecture

```
drone_autonomy/
├── core/ # Core Control Module
├── perception/ # Perception and Sensor Module
├── planning/ # Path Planning and Navigation
├── control/ # Flight Control
├── missions/ # Specific Mission Implementation
└── utils/ # Tools and Utilities
```

## Quick Start

### 1. Environment Requirements

- Ubuntu 22.04 / ROS2 Humble
- PX4-Autopilot (SITL)
- Gazebo Classic
- Python 3.8+

### 2. Installing Dependencies

```bash
# Install ROS2 Dependencies
sudo apt update
sudo apt install ros-humble-px4-msgs ros-humble-gazebo-ros-pkgs

# Install Python Dependencies
pip install numpy scipy
```

### 3. Building the Project

```bash
# Enter the workspace
cd /path/to/autonomy/new_structure

# Build the Project
./scripts/build_project.sh

# Source the Environment
source install/setup.bash
```

### 4. Running a Simple Test

```bash
# Method 1: Using a Portable Script
./scripts/run_simulation.sh

# Method 2: Manual Startup
# Terminal 1: Start PX4 SITL
cd $HOME/PX4-Autopilot
make px4_sitl gazebo-classic_iris_empty

# Terminal 2: Run the mission
ros2 launch drone_autonomy simple_takeoff.launch.py
```

## Usage Examples

### Simple Takeoff Mission

```bash
# Using default parameters
ros2 run drone_autonomy simple_takeoff

# Customizing parameters
ros2 launch drone_autonomy simple_takeoff.launch.py ​​takeoff_altitude:=5.0 hover_duration:=15.0
```

### Mission Parameter Configuration

Edit the configuration file `config/mission_configs/simple_takeoff.yaml`:

```yaml
Parameters:
takeoff_altitude: 3.0 # Takeoff altitude (meters)
hover_duration: 10.0 # Hovering duration (seconds)
safety_timeout: 30.0 # Safety timeout (seconds)
```

## Core Module Description

### 1. Core Control Module (core/)

**DroneController**: Basic Flight Controller
- Basic Flight Commands (Takeoff, Landing, Hover)
- PX4 Offboard Mode Control
- Coordinate System Conversion (ENU ↔ NED)
- Position and Velocity Control

```python
from drone_autonomy.core import DroneController

class MyMission(DroneController):
def __init__(self):
super().__init__('my_mission')

def control_loop_callback(self):
# Custom Control Logic
self.goto_position_enu(x=5.0, y=0.0, z=3.0)
```

### 2. Mission System (missions/)

**SimpleTakeoffMission**: Basic Takeoff Mission Example
- State Machine Design
- Safety Checks
- Parameterized Configuration

### 3. Configuration System

Supports YAML configuration files for easy parameter adjustment:
- `config/drone_params.yaml`: Basic drone parameters
- `config/mission_configs/`: Specific mission configurations

## Development Guide

### Adding a New Mission

1. Create a new mission file in `drone_autonomy/missions/`
2. Inherit the `DroneController` base class
3. Implement the state machine logic
4. Add this to the `setup.py` entry point

```python
# Example: my_mission.py
from ..core.drone_controller import DroneController

class MyMission(DroneController):
def __init__(self):
super().__init__('my_mission')
self.mission_state = 'INIT'

def control_loop_callback(self):
self.publish_offboard_control_mode()
# Add your logic...
```

### Adding a New Sensor

1. Add a sensor interface to `drone_autonomy/perception/`
2. Implement data processing and fusion
3. Integrate with the control system

### Test Development

```bash
# Run unit tests
python -m pytest src/drone_autonomy/test/

# Run integration tests
ros2 test drone_autonomy
```

## Troubleshooting

### Common Problems

1. **Unable to connect to PX4**: Confirm that PX4 SITL is running and check the port configuration.
2. **Drone will not take off**: Check that Offboard mode is enabled and verify GPS/EKF status.
3. **Coordinate system error**: Pay attention to the ENU (Gazebo) and NED (PX4) coordinate system conversions.

### Log Analysis

```bash
# View ROS2 logs
ros2 topic echo /fmu/out/vehicle_status

# View drone position
ros2 topic echo /fmu/out/vehicle_odometry
```

## Expansion Plan

- [ ] Add obstacle avoidance algorithms (RRT*, A*)
- [ ] Integrated visual SLAM
- [ ] Multi-machine collaboration
- [ ] Unity simulation interface
- [ ] Real-machine testing framework

## Contributions

Issues and pull requests are welcome!

## License

MIT License

## Contact Information

Project Maintainer: DroneRanger Team