@echo off
setlocal enabledelayedexpansion

:: AirSim Full Stack Docker Launcher
:: Launches PX4, ROS2, and network services in the shared airsim-network

if "%~1"=="" (
    echo.
    echo ================================================
    echo   AirSim Full Stack Docker Launcher
    echo ================================================
    echo.
    echo Usage: %~nx0 [stack_type] [num_drones]
    echo.
    echo Stack Types:
    echo   px4-only     - Launch only PX4 instances
    echo   ros2-only    - Launch only ROS2 wrapper
    echo   ros2-vnc     - Launch ROS2 wrapper with VNC GUI
    echo   full-stack   - Launch PX4 + ROS2 wrapper
    echo   full-vnc     - Launch PX4 + ROS2 wrapper with VNC
    echo.
    echo Examples:
    echo   %~nx0 px4-only 2        - Launch 2 PX4 drones only
    echo   %~nx0 ros2-only         - Launch ROS2 wrapper only
    echo   %~nx0 full-stack 3      - Launch 3 PX4 drones + ROS2 wrapper
    echo   %~nx0 full-vnc 1        - Launch 1 PX4 drone + ROS2 with VNC
    echo.
    pause
    exit /b 1
)

set STACK_TYPE=%1
set NUM_DRONES=%2

if "%NUM_DRONES%"=="" set NUM_DRONES=1

echo.
echo ================================================
echo   AirSim Full Stack Docker Launcher
echo ================================================
echo.
echo [INFO] Stack Type: %STACK_TYPE%
echo [INFO] Number of Drones: %NUM_DRONES%
echo [INFO] Shared Network: airsim-shared-network
echo.

:: Determine profiles to use
set PROFILES=
if "%STACK_TYPE%"=="px4-only" (
    if %NUM_DRONES%==1 set PROFILES=--profile drone-1
    if %NUM_DRONES%==2 set PROFILES=--profile drone-2
    if %NUM_DRONES%==3 set PROFILES=--profile drone-3
    if %NUM_DRONES%==4 set PROFILES=--profile drone-4
    if %NUM_DRONES%==5 set PROFILES=--profile drone-5
) else if "%STACK_TYPE%"=="ros2-only" (
    set PROFILES=--profile ros2
) else if "%STACK_TYPE%"=="ros2-vnc" (
    set PROFILES=--profile ros2-vnc
) else if "%STACK_TYPE%"=="full-stack" (
    if %NUM_DRONES%==1 set PROFILES=--profile drone-1 --profile ros2
    if %NUM_DRONES%==2 set PROFILES=--profile drone-2 --profile ros2
    if %NUM_DRONES%==3 set PROFILES=--profile drone-3 --profile ros2
    if %NUM_DRONES%==4 set PROFILES=--profile drone-4 --profile ros2
    if %NUM_DRONES%==5 set PROFILES=--profile drone-5 --profile ros2
) else if "%STACK_TYPE%"=="full-vnc" (
    if %NUM_DRONES%==1 set PROFILES=--profile drone-1 --profile ros2-vnc
    if %NUM_DRONES%==2 set PROFILES=--profile drone-2 --profile ros2-vnc
    if %NUM_DRONES%==3 set PROFILES=--profile drone-3 --profile ros2-vnc
    if %NUM_DRONES%==4 set PROFILES=--profile drone-4 --profile ros2-vnc
    if %NUM_DRONES%==5 set PROFILES=--profile drone-5 --profile ros2-vnc
) else (
    echo [ERROR] Invalid stack type: %STACK_TYPE%
    echo Valid types: px4-only, ros2-only, ros2-vnc, full-stack, full-vnc
    pause
    exit /b 1
)

if "%PROFILES%"=="" (
    echo [ERROR] Invalid number of drones: %NUM_DRONES% ^(max 5^)
    pause
    exit /b 1
)

echo [INFO] Using Docker Compose profiles: %PROFILES%
echo [INFO] Building and starting containers...
echo.

echo Building Docker images...
docker-compose build

echo Starting containers...
docker-compose %PROFILES% up -d

if %ERRORLEVEL% neq 0 (
    echo [ERROR] Failed to start containers!
    pause
    exit /b 1
)

echo.
echo ================================================
echo   Containers Started Successfully!
echo ================================================
echo.

:: Display network information
echo Network Configuration:
echo   Network Name: airsim-shared-network
echo   Subnet: 172.25.0.0/16
echo   Gateway: 172.25.0.1
echo.

:: Show container IPs and ports (1-based)
echo Container Network Assignments ^(1-based^):
if "%STACK_TYPE%"=="px4-only" or "%STACK_TYPE%"=="full-stack" or "%STACK_TYPE%"=="full-vnc" (
    echo   PX4 Containers:
    for /l %%i in (1,1,%NUM_DRONES%) do (
        set /a PORT=4560+%%i
        set /a IP_LAST=9+%%i
        set /a UDP_LOCAL=14540+%%i
        set /a UDP_REMOTE=14580+%%i
        echo     px4-drone-%%i: 172.25.0.!IP_LAST! ^(TCP !PORT!, UDP !UDP_LOCAL!/!UDP_REMOTE!^)
    )
    echo.
)

if "%STACK_TYPE%"=="ros2-only" or "%STACK_TYPE%"=="full-stack" (
    echo   ROS2 Wrapper: 172.25.0.20 ^(UDP 7400-7402^)
    echo.
)

if "%STACK_TYPE%"=="ros2-vnc" or "%STACK_TYPE%"=="full-vnc" (
    echo   ROS2 VNC Wrapper: 172.25.0.21
    echo     VNC Server: localhost:5901
    echo     Web VNC: http://localhost:6901/vnc.html
    echo     Password: airsim123
    echo.
)

:: Generate AirSim settings if PX4 is included
if "%STACK_TYPE%"=="px4-only" or "%STACK_TYPE%"=="full-stack" or "%STACK_TYPE%"=="full-vnc" (
    echo AirSim Settings.json for %NUM_DRONES% drones ^(1-based^):
    echo Copy and paste this into your AirSim settings.json file:
    echo.
    echo {
    echo   "SettingsVersion": 2.0,
    echo   "SimMode": "Multirotor",
    echo   "Vehicles": {
    
    for /l %%i in (1,1,%NUM_DRONES%) do (
        set /a PORT=4560+%%i
        set /a LOCAL_PORT=14540+%%i
        set /a REMOTE_PORT=14580+%%i
        set /a X_POS=%%i*5-5
        if %%i lss %NUM_DRONES% (
            echo     "PX4_Drone%%i": {
            echo       "VehicleType": "PX4Multirotor",
            echo       "UseSerial": false,
            echo       "UseTcp": true,
            echo       "TcpPort": !PORT!,
            echo       "ControlPortLocal": !LOCAL_PORT!,
            echo       "ControlPortRemote": !REMOTE_PORT!,
            echo       "LockStep": true,
            echo       "X": !X_POS!, "Y": 0, "Z": -2
            echo     },
        ) else (
            echo     "PX4_Drone%%i": {
            echo       "VehicleType": "PX4Multirotor",
            echo       "UseSerial": false,
            echo       "UseTcp": true,
            echo       "TcpPort": !PORT!,
            echo       "ControlPortLocal": !LOCAL_PORT!,
            echo       "ControlPortRemote": !REMOTE_PORT!,
            echo       "LockStep": true,
            echo       "X": !X_POS!, "Y": 0, "Z": -2
            echo     }
        )
    )
    
    echo   }
    echo }
    echo.
)

echo Management Commands:
echo   Stop all: docker-compose %PROFILES% down
echo   View logs: docker-compose logs -f
echo   Check status: docker ps
echo   Network inspect: docker network inspect airsim-shared-network
echo.

if "%STACK_TYPE%"=="ros2-vnc" or "%STACK_TYPE%"=="full-vnc" (
    echo VNC Access:
    echo   Wait 10 seconds, then open: http://localhost:6901/vnc.html
    echo   Password: airsim123
    echo.
)

echo [INFO] All services are running on the shared airsim-shared-network
echo [INFO] Your Windows Unreal host can connect via 'host.docker.internal'

pause 