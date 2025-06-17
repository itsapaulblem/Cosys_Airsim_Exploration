@echo off
REM AirSim ROS2 Docker Container - Simple Version with RViz2
REM This script runs the AirSim ROS2 wrapper with RViz2 visualization

echo Starting AirSim ROS2 Docker Container with RViz2...

REM Check if Docker is running
docker version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Docker is not running or not installed.
    echo Please start Docker Desktop and try again.
    pause
    exit /b 1
)

REM Stop any existing container
docker stop airsim-ros2-wrapper 2>nul
docker rm airsim-ros2-wrapper 2>nul

REM Run the container with RViz2 enabled
docker run -it --rm ^
    --name airsim-ros2-wrapper ^
    -p 7400:7400/udp ^
    -p 7401:7401/udp ^
    -p 7402:7402/udp ^
    -e ROS_DOMAIN_ID=0 ^
    -e AIRSIM_HOST_IP=host.docker.internal ^
    -e AIRSIM_HOST_PORT=41451 ^
    -e LAUNCH_RVIZ=true ^
    -e DISPLAY=host.docker.internal:0.0 ^
    airsim-ros2-wrapper:simple

pause 