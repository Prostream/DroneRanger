# DroneRanger Autonomy Design

## New File Structure

```
autonomy/
├── README.md # Project Description
├── requirements.txt # Python Dependencies
├── docker/ # Docker Configuration
│ ├── Dockerfile
│ └── docker-compose.yml
├── config/ # Configuration Files
│ ├── drone_params.yaml # Drone Parameter Configuration
│ ├── mission_configs/ # Mission Configuration
│ │ ├── simple_takeoff.yaml
│ │ ├── obstacle_course.yaml
│ │ └── house_mission.yaml
│ └── gazebo/ # Gazebo Configuration
│ ├── worlds/
│ │ ├── empty_world.world
│ │ ├── house_with_obstacles.world
│ │ └── test_arena.world
│ └── models/ # Custom models
├── src/
│ └── drone_autonomy/ # Main ROS2 package
│ ├── package.xml
│ ├── setup.py
│ ├── drone_autonomy/
│ │ ├── __init__.py
│ │ ├── core/ # Core modules
│ │ │ ├── __init__.py
│ │ │ ├── drone_controller.py # Drone controller base class
│ │ │ ├── mission_executor.py # Mission executor
│ │ │ └── coordinate_transforms.py # Coordinate transformation tools
│ │ ├── perception/ # Perception module
│ │ │ ├── __init__.py
│ │ │ ├── obstacle_detector.py # Obstacle Detection
│ │ │ └── sensor_fusion.py # Sensor Fusion
│ │ ├── planning/ # Path Planning Module
│ │ │ ├── __init__.py
│ │ │ ├── path_planner.py # Path Planner
│ │ │ ├── obstacle_avoidance.py # Obstacle Avoidance Algorithm
│ │ │ └── trajectory_generator.py # Trajectory Generation
│ │ ├── control/ # Control Module
│ │ │ ├── __init__.py
│ │ │ ├── offboard_controller.py # Offboard Control
│ │ │ └── safety_monitor.py # Safety Monitoring
│ │ ├── missions/ # Specific Mission Implementation
│ │ │ ├── __init__.py
│ │ │ ├── simple_takeoff.py # Simple takeoff mission
│ │ │ ├── waypoint_mission.py # Waypoint mission
│ │ │ └── house_inspection.py # House inspection mission
│ │ └── utils/ # Utility modules
│ │ ├── __init__.py
│ │ ├── logger.py # Logging tool
│ │ ├── data_recorder.py # Data logging
│ │ └── visualization.py # Visualization tool
│ ├── launch/ # Launch file
│ │ ├── basic_takeoff.launch.py
│ │ ├── obstacle_course.launch.py
│ │ ├── house_mission.launch.py
│ │ └── simulation.launch.py
│ └── test/ # Test files
│ ├── test_drone_controller.py
│ ├── test_path_planner.py
│ └── integration_tests.py
├── scripts/ # Useful scripts
│ ├── setup_environment.sh # Environment setup
│ ├── build_project.sh # Compile script
│ ├── run_simulation.sh # Run simulation
│ └── run_tests.sh # Run tests
├── logs/ # Log files
├── data/ # Data files
│ ├── flight_logs/
│ └── test_results/
└── docs/ # Documentation
├── api_reference.md
├── user_guide.md
└── development_guide.md
```

## Module Functionality

### Core Module (core/)
- **drone_controller.py**: Base drone controller class, providing basic flight control interfaces
- **mission_executor.py**: Mission executor, responsible for scheduling and executing various flight missions
- **coordinate_transforms.py**: Coordinate system conversion tool (ENU ↔ NED)

### Perception Module (perception/)
- **obstacle_detector.py**: Obstacle detection, processing data from various sensors
- **sensor_fusion.py**: Multi-sensor data fusion

### Planning Module (planning/)
- **path_planner.py**: Path planning algorithm (A*, RRT, etc.)
- **obstacle_avoidance.py**: Real-time obstacle avoidance algorithm
- **trajectory_generator.py**: Smooth trajectory generation

### Control Module (control/)
- **offboard_controller.py**: PX4 offboard mode control
- **safety_monitor.py**: Flight Safety Monitoring

### Mission Module (missions/)
- Specific flight mission implementation, extensible

### Tool Module (utils/)
- Auxiliary tools such as logging, data logging, and visualization