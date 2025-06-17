@echo off
echo Building and running AirSim ROS2 Wrapper (Minimal Optimized Version)...
echo This version reduces build context from 22GB to ~500MB!

cd /d "%~dp0"

echo Building Docker image (much faster now)...
docker build -f Dockerfile.minimal -t airsim-ros2-wrapper:minimal ..\..\

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
    -e AIRSIM_HOST_IP=host.docker.internal ^
    -e AIRSIM_HOST_PORT=41451 ^
    airsim-ros2-wrapper:minimal

pause 