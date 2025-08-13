@echo off
REM AirSim ROS2 VNC Container Launcher
REM Simplified launcher for the VNC-enabled ROS2 environment

echo 🚁 AirSim ROS2 VNC Container Launcher
echo =====================================

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ Docker is not running. Please start Docker Desktop.
    pause
    exit /b 1
)

REM Navigate to script directory
cd /d "%~dp0"

echo 📁 Working directory: %cd%

REM Check if docker-compose.yml exists
if not exist "docker-compose.yml" (
    echo ❌ docker-compose.yml not found!
    pause
    exit /b 1
)

echo 🔧 Starting AirSim ROS2 VNC container...

REM Start the container
docker-compose up -d

if %errorlevel% equ 0 (
    echo.
    echo ✅ Container started successfully!
    echo.
    echo 🖥️  VNC Access:
    echo    URL: localhost:5901
    echo    Password: airsim
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
    pause
    exit /b 1
)

pause