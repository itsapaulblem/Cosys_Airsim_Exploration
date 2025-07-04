@echo off
echo ================================================================
echo    PX4-AirSim Multi-Drone Quick Start
echo    Complete Workflow: Settings -> Docker -> Launch
echo ================================================================
echo.

REM Check if argument provided
if "%~1"=="" (
    echo Usage: %~nx0 [number_of_drones]
    echo.
    echo Example: %~nx0 3    ^(for 3 drones^)
    echo.
    echo This script will:
    echo   1. Generate AirSim settings.json for N drones
    echo   2. Generate Docker Compose configuration
    echo   3. Launch all containers
    echo   4. Show status and next steps
    echo.
    pause
    exit /b 1
)

set NUM_DRONES=%~1

echo [STEP 1] Generating AirSim settings for %NUM_DRONES% drone(s)...
echo ================================================================
python generate_settings.py %NUM_DRONES%

if %errorlevel% neq 0 (
    echo [ERROR] Failed to generate AirSim settings!
    pause
    exit /b 1
)

echo.
echo [STEP 2] Generating Docker Compose configuration...
echo ================================================================
python simple_generator.py

if %errorlevel% neq 0 (
    echo [ERROR] Failed to generate Docker Compose configuration!
    pause
    exit /b 1
)

echo.
echo [STEP 3] Launching Docker containers...
echo ================================================================
call launch_generated.bat

echo.
echo ================================================================
echo    🎉 Quick Start Complete!
echo ================================================================
echo.
echo ✅ AirSim settings.json generated
echo ✅ Docker Compose configuration created  
echo ✅ %NUM_DRONES% PX4 container(s) launched
echo.
echo 📋 What's Running:
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | findstr px4

echo.
echo 🚀 Next Steps:
echo   1. Launch your AirSim/Unreal environment
echo   2. Drones will auto-connect to the containers
echo   3. Check GPS status: python diagnose_gps_docker.py
echo   4. Use QGroundControl on ports 14550-%QGC_END%
echo.
echo 🔧 Management Commands:
echo   View logs:  docker-compose -f docker-compose.generated.yml logs
echo   Stop all:   docker-compose -f docker-compose.generated.yml down
echo   Restart:    %~nx0 %NUM_DRONES%
echo.
echo 📖 Full documentation: WORKFLOW_GUIDE.md
echo.
pause 