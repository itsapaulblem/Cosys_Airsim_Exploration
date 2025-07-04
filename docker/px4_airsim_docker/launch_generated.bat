@echo off
echo ================================================
echo    Generated PX4 AirSim Docker Launcher
echo    Generated for 5 drone(s)
echo ================================================
echo.

echo [INFO] Creating shared data directory...
if not exist "shared_data" mkdir shared_data

echo [INFO] Building Docker images...
docker-compose -f docker-compose.generated.yml build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker images!
    pause
    exit /b 1
)

echo [INFO] Cleaning up existing containers...
docker-compose -f docker-compose.generated.yml down > nul 2>&1

echo [INFO] Starting 5 drone instance(s)...
docker-compose -f docker-compose.generated.yml up -d

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start drone instances!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    5 Drone(s) Started Successfully!
echo ================================================
echo.

echo Port Configuration:
echo   PX4_Drone1: TCP=4561 ^| MAVLink=14541/14581 ^| QGC=14550
echo   PX4_Drone2: TCP=4562 ^| MAVLink=14542/14582 ^| QGC=14551
echo   PX4_Drone3: TCP=4563 ^| MAVLink=14543/14583 ^| QGC=14552
echo   PX4_Drone4: TCP=4564 ^| MAVLink=14544/14584 ^| QGC=14553
echo   PX4_Drone5: TCP=4565 ^| MAVLink=14545/14585 ^| QGC=14554

echo.
echo Management Commands:
echo   View logs:        docker-compose -f docker-compose.generated.yml logs
echo   Stop all:         docker-compose -f docker-compose.generated.yml down
echo   Container status: docker-compose -f docker-compose.generated.yml ps
echo.
pause
