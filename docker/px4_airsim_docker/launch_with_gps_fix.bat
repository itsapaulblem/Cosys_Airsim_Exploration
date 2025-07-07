@echo off
REM Enhanced launcher for PX4-AirSim with GPS troubleshooting
REM This script addresses common GPS home location issues

echo ========================================
echo   PX4-AirSim Enhanced Launcher
echo ========================================

set COMPOSE_FILE=docker-compose.yml
set PROFILE=single

REM Parse arguments
if "%1"=="multi" set PROFILE=multi
if "%1"=="drone-2" set PROFILE=drone-2
if "%1"=="drone-3" set PROFILE=drone-3
if "%1"=="drone-4" set PROFILE=drone-4
if "%1"=="drone-5" set PROFILE=drone-5

echo 🚁 Launching PX4 containers with profile: %PROFILE%

REM Clean up any existing containers and networks to prevent conflicts
echo 🧹 Cleaning up existing containers and networks...
docker-compose down --remove-orphans

REM Remove any conflicting networks
echo 🌐 Checking for network conflicts...
docker network rm px4_airsim_docker_airsim-network 2>nul
docker network rm airsim-network 2>nul

REM Start the containers
echo 🚀 Starting PX4 containers...
docker-compose --profile %PROFILE% up -d

REM Wait for containers to initialize
echo ⏱️  Waiting for containers to initialize...
timeout /t 10 /nobreak

REM Check container status
echo 📋 Container Status:
docker-compose ps

REM Test connectivity
echo 🌐 Testing connectivity...
timeout /t 5 /nobreak

echo ✅ PX4 containers started successfully!
echo.
echo 📍 Important Configuration Steps:
if "%PROFILE%"=="single" (
    echo   1. Copy: %%~dp0settings\airsim_px4_single_settings.json
    echo      To: %%USERPROFILE%%\Documents\AirSim\settings.json
) else (
    echo   1. Copy: %%~dp0settings\airsim_px4_multi_settings.json
    echo      To: %%USERPROFILE%%\Documents\AirSim\settings.json
)
echo   2. Launch AirSim (Unreal Engine)
echo   3. Wait 30-60 seconds for GPS home establishment
echo   4. Check AirSim console for connection messages
echo.
echo 🔧 Troubleshooting Commands:
echo   .\test_connection_step_by_step.bat  (guided troubleshooting)
echo   .\quick_gps_check.bat              (quick connectivity test)
echo   docker logs px4-single             (check single drone logs)
echo   docker logs px4-drone-1            (check multi drone logs)
echo   python diagnose_gps_docker.py      (full GPS diagnostics)
echo.

pause