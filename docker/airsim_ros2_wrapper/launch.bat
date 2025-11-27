@echo off
REM 🎯 Ultra-Clean Multi-Node ROS2 System Launcher (Windows)
REM Advanced orchestration for AirSim multi-vehicle simulation

setlocal enabledelayedexpansion

REM Default configuration
set "DEFAULT_LAUNCH_MODE=multi"
set "DEFAULT_AIRSIM_HOST=host.docker.internal"
set "DEFAULT_AIRSIM_PORT=41451"
set "DEFAULT_ENABLE_COORDINATION=true"
set "DEFAULT_RPC_TIMEOUT=10.0"
set "DEFAULT_LAUNCH_RVIZ=false"

REM Parse command line arguments
set "LAUNCH_MODE=%~1"
set "LAUNCH_RVIZ=%~2"

if "%LAUNCH_MODE%"=="" set "LAUNCH_MODE=%DEFAULT_LAUNCH_MODE%"
if "%LAUNCH_RVIZ%"=="" set "LAUNCH_RVIZ=%DEFAULT_LAUNCH_RVIZ%"

REM Handle help requests
if "%~1"=="-h" goto :show_help
if "%~1"=="--help" goto :show_help
if "%~1"=="help" goto :show_help

REM Validate launch mode
if "%LAUNCH_MODE%"=="multi" goto :valid_mode
if "%LAUNCH_MODE%"=="legacy" goto :valid_mode
if "%LAUNCH_MODE%"=="custom" goto :valid_mode

echo ❌ Invalid launch mode: %LAUNCH_MODE%
echo Valid options: multi, legacy, custom
echo.
goto :show_help

:valid_mode

echo 🎯 Ultra-Clean Multi-Node ROS2 System Launcher
echo ===============================================
echo.

REM Docker availability check
echo 🔍 Checking Docker availability...
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ Docker is not running. Please start Docker Desktop.
    pause
    exit /b 1
)
echo ✓ Docker is running
echo.

REM Navigate to script directory
cd /d "%~dp0"
echo 📁 Working directory: %cd%

REM Verify docker-compose.yml exists
if not exist "docker-compose.yml" (
    echo ❌ docker-compose.yml not found!
    pause
    exit /b 1
)

REM Set environment variables for docker-compose
set "LAUNCH_MODE=%LAUNCH_MODE%"
set "LAUNCH_RVIZ=%LAUNCH_RVIZ%"
if "%AIRSIM_HOST_IP%"=="" set "AIRSIM_HOST_IP=%DEFAULT_AIRSIM_HOST%"
if "%AIRSIM_HOST_PORT%"=="" set "AIRSIM_HOST_PORT=%DEFAULT_AIRSIM_PORT%"
if "%RPC_TIMEOUT%"=="" set "RPC_TIMEOUT=%DEFAULT_RPC_TIMEOUT%"

echo 🎯 Configuration Summary:
    echo.
    echo 📊 Container Status:
    docker-compose ps
    echo.
    echo 💡 Useful Commands:
    echo    View logs:     docker-compose logs -f
    echo    Stop:          docker-compose stop
    echo    Restart:       docker-compose restart
    echo    Enter shell:   docker exec -it airsim-ros2-vnc bash
    echo.
    echo 🚁 Ready for ROS2 development!
) else (
    echo ❌ Failed to start container
    echo 📋 Check logs with: docker-compose logs
=======
    
    REM Wait for container to be ready
    echo ⏳ Waiting for container initialization...
    timeout /t 3 /nobreak >nul
    
    REM Check container health
    docker-compose ps | findstr "Up" >nul
    if !errorlevel! equ 0 (
        echo ✓ Container is healthy and running
        echo.
        
        echo 🖥️  Access Information:
        echo    VNC URL:      localhost:5901
        echo    VNC Password: ubuntu
        echo    Container:    ros2-multi-node
        echo.
        
        echo 📊 Container Status:
        docker-compose ps
        echo.
        
        echo 💡 Useful Commands:
        echo    View logs:        docker-compose logs -f
        echo    View ROS logs:    docker-compose logs -f ^| findstr "ros2-multi-node"
        echo    Enter container:  docker exec -it ros2-multi-node bash
        echo    Stop system:      docker-compose stop
        echo    Full cleanup:     docker-compose down -v
        echo.
        
        if "%LAUNCH_MODE%"=="multi" (
            echo 🎯 Multi-Node Verification Commands:
            echo    Check nodes:      docker exec -it ros2-multi-node ros2 node list
            echo    Check topics:     docker exec -it ros2-multi-node ros2 topic list
            echo    Check TF tree:    docker exec -it ros2-multi-node ros2 run tf2_tools view_frames
            echo    Test vehicle:     docker exec -it ros2-multi-node ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
            echo.
        )
        
        if "%LAUNCH_RVIZ%"=="true" (
            echo 📊 RViz2 Configuration Tips:
            echo    • Fixed Frame: Set to 'world_ned' for proper multi-vehicle view
            echo    • Add TF Display: See individual vehicle coordinate frames
            echo    • Vehicle Topics: Add /Droan1/odom_local_ned, /PX4_Drone2/imu
            echo.
        )
        
        echo 🚁 Ultra-clean multi-node ROS2 system ready!
        
        REM Show follow-up actions based on mode
        if "%LAUNCH_MODE%"=="custom" (
            echo.
            echo 🛠️  Enter the container to run manual commands:
            echo    docker exec -it ros2-multi-node bash
        ) else (
            echo.
            echo 📋 Monitor the system:
            echo    docker-compose logs -f
        )
        
    ) else (
        echo ⚠️  Container started but may not be fully healthy
        echo 📋 Check logs for details: docker-compose logs
    )
    
) else (
    echo.
    echo ❌ Failed to start container
    echo 📋 Troubleshooting:
    echo    • Check logs: docker-compose logs
    echo    • Verify AirSim is running if using multi/legacy modes
    echo    • Ensure ports 5901, 7400, 7401 are available
    echo    • For Windows: Check Docker Desktop WSL2 integration
    pause
    exit /b 1
)

echo.
echo Press any key to continue...
pause >nul
exit /b 0

:show_help
echo 🎯 Ultra-Clean Multi-Node ROS2 System Launcher
echo ===============================================
echo.
echo Usage: %~nx0 [MODE] [RVIZ]
echo.
echo Launch Modes:
echo   multi    - 🎯 Ultra-clean multi-node (default, recommended)
echo   legacy   - 🔄 Legacy monolithic (backward compatibility)
echo   custom   - 🛠️  Manual control with container shell access
echo.
echo RViz Options:
echo   true     - 📊 Auto-launch RViz2 for visualization
echo   false    - Terminal-only mode (default)
echo.
echo Examples:
echo   %~nx0                    # Launch multi-node without RViz
echo   %~nx0 multi true         # Launch multi-node with RViz
echo   %~nx0 legacy false       # Launch legacy mode
echo   %~nx0 custom             # Custom shell access
echo.
echo Environment Variables (set before running):
echo   AIRSIM_HOST_IP        - AirSim server IP (%DEFAULT_AIRSIM_HOST%)
echo   AIRSIM_HOST_PORT      - AirSim API port (%DEFAULT_AIRSIM_PORT%)
echo   ENABLE_COORDINATION   - Global coordination node (%DEFAULT_ENABLE_COORDINATION%)
echo   RPC_TIMEOUT          - Discovery timeout (%DEFAULT_RPC_TIMEOUT%)
echo.
pause
exit /b 0
