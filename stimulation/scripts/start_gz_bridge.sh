#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros_gz_ws/install/setup.bash

# Set environment
export ROS_DOMAIN_ID=0

# Run bridge with all sensors
~/ros_gz_ws/install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /world/default/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer \
  /world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
  /world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure \
  /world/default/model/x500_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
