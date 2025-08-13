@echo off
echo ================================================
echo    Multi-Drone PX4 AirSim Setup
echo ================================================
echo.

if "%~1"=="" (
    echo Usage: %~nx0 [drone_count] [options]
    echo.
    echo Examples:
    echo   %~nx0 2          - Start 2 drones
    echo   %~nx0 3          - Start 3 drones  
    echo   %~nx0 all        - Start all 5 drones
    echo.
    echo Individual drones:
    echo   %~nx0 drone-1    - Start only drone 1
    echo   %~nx0 drone-2    - Start only drone 2
    echo   etc.
    echo.
    pause
    exit /b 1
)

set DRONE_CONFIG=%~1

echo [INFO] Stopping any existing containers...
docker-compose down

echo [INFO] Building Docker images...
docker-compose build
if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker images!
    pause
    exit /b 1
)

echo [INFO] Starting drones with configuration: %DRONE_CONFIG%
if "%DRONE_CONFIG%"=="2" (
    docker-compose --profile drone-1 --profile drone-2 up -d
) else if "%DRONE_CONFIG%"=="3" (
    docker-compose --profile drone-1 --profile drone-2 --profile drone-3 up -d
) else if "%DRONE_CONFIG%"=="4" (
    docker-compose --profile drone-1 --profile drone-2 --profile drone-3 --profile drone-4 up -d
) else if "%DRONE_CONFIG%"=="5" (
    docker-compose --profile drone-1 --profile drone-2 --profile drone-3 --profile drone-4 --profile drone-5 up -d
) else if "%DRONE_CONFIG%"=="all" (
    docker-compose --profile all up -d
) else if "%DRONE_CONFIG%"=="drone-1" (
    docker-compose --profile drone-1 up -d
) else if "%DRONE_CONFIG%"=="drone-2" (
    docker-compose --profile drone-2 up -d
) else if "%DRONE_CONFIG%"=="drone-3" (
    docker-compose --profile drone-3 up -d
) else if "%DRONE_CONFIG%"=="drone-4" (
    docker-compose --profile drone-4 up -d
) else if "%DRONE_CONFIG%"=="drone-5" (
    docker-compose --profile drone-5 up -d
) else (
    echo [ERROR] Invalid configuration: %DRONE_CONFIG%
    echo Valid options: 2, 3, 4, 5, all, drone-1, drone-2, drone-3, drone-4, drone-5
    pause
    exit /b 1
)

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start drones!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    Multi-Drone Setup Complete!
echo ================================================
echo.
echo Port Configuration:
echo   Drone 1: AirSim=4561, QGC=14550, MAVLink=14541/14581
echo   Drone 2: AirSim=4562, QGC=14551, MAVLink=14542/14582
echo   Drone 3: AirSim=4563, QGC=14552, MAVLink=14543/14583
echo   Drone 4: AirSim=4564, QGC=14553, MAVLink=14544/14584
echo   Drone 5: AirSim=4565, QGC=14554, MAVLink=14545/14585
echo.
echo Management Commands:
echo   View logs:        docker-compose logs -f
echo   Stop all:         docker-compose down
echo   Container status: docker-compose ps
echo   Individual logs:  docker-compose logs -f px4-drone-[N]
echo.
pause