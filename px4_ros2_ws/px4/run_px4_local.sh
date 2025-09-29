#!/bin/bash
# Usage: ./run_px4_local.sh <LAT> <LON> <ALT>

# if [ $# -lt 3 ]; then
#     echo "Usage: $0 <LAT> <LON> <ALT>"
#     exit 1
# fi

LAT=$1
LON=$2
ALT=$3

LAT=37.4873108
LON=-122.2853061
ALT=10.0

echo "🌍 Starting PX4 at:"
echo "   Latitude: $LAT"
echo "   Longitude: $LON"
echo "   Altitude: $ALT"

PX4_HOME_LAT=$LAT PX4_HOME_LON=$LON PX4_HOME_ALT=$ALT PX4_SITL_RTPS=1 \
make px4_sitl_default gazebo-classic gazebo
