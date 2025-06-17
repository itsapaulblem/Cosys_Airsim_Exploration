@echo off
echo Testing AirSim ROS2 Wrapper Docker build...

cd /d "%~dp0"

echo Building Docker image...
docker build -f Dockerfile -t airsim-ros2-wrapper:test ../..

if %ERRORLEVEL% neq 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo Build successful!
echo Testing container startup...
docker run --rm airsim-ros2-wrapper:test /bin/bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && ros2 pkg list | grep airsim"

if %ERRORLEVEL% neq 0 (
    echo Container test failed!
    pause
    exit /b 1
)

echo All tests passed!
pause 