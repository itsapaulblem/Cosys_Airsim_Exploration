@echo off
echo Building and running AirSim ROS2 Wrapper (Simple Version - No rosdep)...

cd /d "%~dp0"

echo Building Docker image...
docker build -f Dockerfile.simple -t airsim-ros2-wrapper:simple ../..

if %ERRORLEVEL% neq 0 (
    echo Docker build failed!
    pause
    exit /b 1
)

echo Starting container...
docker run -it --rm ^
    --name airsim-ros2-wrapper ^
    --network host ^
    -e ROS_DOMAIN_ID=0 ^
    -e DISPLAY=host.docker.internal:0.0 ^
    airsim-ros2-wrapper:simple

pause 