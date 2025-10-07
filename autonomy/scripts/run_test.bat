@echo off
REM Windows batch script to run simple takeoff test

echo Starting DroneRanger Test on Windows...

REM Check if we're in the right directory
if not exist "src\drone_autonomy\package.xml" (
    echo Error: Please run this script from the ROS2 workspace root
    echo Expected to find: src\drone_autonomy\package.xml
    pause
    exit /b 1
)

REM Setup ROS2 environment (adjust path as needed)
echo Setting up ROS2 environment...
call C:\dev\ros2_humble\local_setup.bat
if exist "install\setup.bat" (
    call install\setup.bat
) else (
    echo Warning: Workspace not built. Run colcon build first
)

echo.
echo Starting simple takeoff test...
echo Make sure PX4 SITL is running in another terminal!
echo.
pause

REM Run the simple takeoff mission
ros2 launch drone_autonomy simple_takeoff.launch.py takeoff_altitude:=2.0 hover_duration:=8.0

echo.
echo Test completed!
pause