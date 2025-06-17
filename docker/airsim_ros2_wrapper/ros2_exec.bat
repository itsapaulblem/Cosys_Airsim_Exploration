@echo off
REM AirSim ROS2 Docker - Easy ROS2 Command Execution
REM Usage: ros2_exec.bat "ros2 topic list"
REM Usage: ros2_exec.bat (for interactive shell)

set CONTAINER_NAME=airsim-ros2-wrapper

REM Check if container is running
docker ps --filter "name=%CONTAINER_NAME%" --format "table {{.Names}}" | findstr %CONTAINER_NAME% >nul
if %errorlevel% neq 0 (
    echo Error: Container '%CONTAINER_NAME%' is not running.
    echo Please start the container first with run_simple.bat or run_minimal.bat
    pause
    exit /b 1
)

if "%~1"=="" (
    REM No arguments - start interactive bash session
    echo Starting interactive bash session in %CONTAINER_NAME%...
    echo Type 'exit' to return to Windows command prompt.
    echo.
    docker exec -it %CONTAINER_NAME% bash
) else (
    REM Execute the provided command
    docker exec -it %CONTAINER_NAME% bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && %*"
) 