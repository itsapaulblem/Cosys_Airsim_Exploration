@echo off
echo ================================================
echo    Scalable PX4 AirSim Docker Launcher
echo ================================================
echo.

REM Check if argument provided
if "%~1"=="" (
    echo Usage: %~nx0 [number_of_drones] [additional_options]
    echo.
    echo Examples:
    echo   %~nx0 1              - Single drone
    echo   %~nx0 3              - Three drones
    echo   %~nx0 5              - Five drones
    echo   %~nx0 1 ros2         - Single drone with ROS2
    echo   %~nx0 2 debug        - Two drones with debug container
    echo.
    echo Maximum: 10 drones supported
    echo.
    pause
    exit /b 1
)

set NUM_DRONES=%~1
set EXTRA_OPTIONS=%~2

REM Validate number of drones
if %NUM_DRONES% LSS 1 (
    echo [ERROR] Number of drones must be at least 1!
    pause
    exit /b 1
)

if %NUM_DRONES% GTR 10 (
    echo [ERROR] Maximum 10 drones supported!
    pause
    exit /b 1
)

echo [INFO] Creating shared data directory...
if not exist "shared_data" mkdir shared_data

echo [INFO] Building Docker image...
docker-compose -f docker-compose-scaled.yml build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker image!
    pause
    exit /b 1
)

REM Clean up any existing containers
echo [INFO] Cleaning up existing containers...
docker-compose -f docker-compose-scaled.yml down > nul 2>&1

REM Build compose command based on options
set COMPOSE_CMD=docker-compose -f docker-compose-scaled.yml

REM Add profiles if specified
if /i "%EXTRA_OPTIONS%"=="ros2" (
    set COMPOSE_CMD=%COMPOSE_CMD% --profile ros2
    echo [INFO] Including ROS2 wrapper...
)

if /i "%EXTRA_OPTIONS%"=="debug" (
    set COMPOSE_CMD=%COMPOSE_CMD% --profile debug
    echo [INFO] Including debug container...
)

REM Start the containers
echo [INFO] Starting %NUM_DRONES% PX4 drone instance(s)...
%COMPOSE_CMD% up --scale px4=%NUM_DRONES% -d

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start drone instances!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    %NUM_DRONES% PX4 Drone(s) Started Successfully!
echo ================================================
echo.

echo Port Configuration:
echo   Docker automatically assigns ports from these ranges:
echo   - AirSim TCP:      4561-%NUM_DRONES_PLUS_4560%
echo   - MAVLink Local:   14541-%NUM_DRONES_PLUS_14540%  
echo   - MAVLink Remote:  14581-%NUM_DRONES_PLUS_14580%
echo   - QGroundControl:  14550-%NUM_DRONES_PLUS_14549%
echo.

if %NUM_DRONES%==1 (
    goto :show_single_settings
) else (
    goto :show_multi_settings
)

:show_single_settings
echo AirSim Settings.json for Single Drone:
echo {
echo   "SettingsVersion": 2.0,
echo   "SimMode": "Multirotor",
echo   "ApiServerEndpoint": "0.0.0.0:41451",
echo   "Vehicles": {
echo     "PX4_Drone1": {
echo       "VehicleType": "PX4Multirotor",
echo       "UseSerial": false,
echo       "UseTcp": true,
echo       "TcpPort": 4561,
echo       "ControlPortLocal": 14541,
echo       "ControlPortRemote": 14581,
echo       "LockStep": true,
echo       "X": 0, "Y": 0, "Z": -2
echo     }
echo   }
echo }
echo.
goto :show_commands

:show_multi_settings
echo AirSim Settings.json for %NUM_DRONES% Drones:
echo {
echo   "SettingsVersion": 2.0,
echo   "SimMode": "Multirotor", 
echo   "ApiServerEndpoint": "0.0.0.0:41451",
echo   "Vehicles": {

REM Generate vehicle configurations
for /L %%i in (1,1,%NUM_DRONES%) do (
    call :generate_vehicle_config %%i
)

echo   }
echo }
echo.
goto :show_commands

:generate_vehicle_config
set /a tcp_port=4560+%1
set /a local_port=14540+%1
set /a remote_port=14580+%1
set /a x_pos=(%1-1)*5
set /a y_pos=0

echo     "PX4_Drone%1": {
echo       "VehicleType": "PX4Multirotor",
echo       "UseTcp": true, "TcpPort": %tcp_port%,
echo       "ControlPortLocal": %local_port%, "ControlPortRemote": %remote_port%,
echo       "X": %x_pos%, "Y": %y_pos%, "Z": -2

if %1==%NUM_DRONES% (
    echo     }
) else (
    echo     },
)
goto :eof

:show_commands
echo Management Commands:
echo   View logs:        docker-compose -f docker-compose-scaled.yml logs -f
echo   Stop all:         docker-compose -f docker-compose-scaled.yml down
echo   Scale up/down:    docker-compose -f docker-compose-scaled.yml up --scale px4=[N] -d
echo   Container status: docker-compose -f docker-compose-scaled.yml ps
echo.

echo Troubleshooting:
echo   List containers:  docker ps
echo   Check networks:   docker network ls
echo   GPS diagnosis:    python diagnose_gps_docker.py
echo   PX4 console:      docker exec -it [container_name] /Scripts/px4_console.sh
echo.

if /i "%EXTRA_OPTIONS%"=="debug" (
    echo Debug container access:
    echo   docker exec -it px4-debug /bin/bash
    echo.
)

if /i "%EXTRA_OPTIONS%"=="ros2" (
    echo ROS2 commands:
    echo   Check ROS2 logs: docker-compose -f docker-compose-scaled.yml logs airsim-ros2
    echo   ROS2 topics:     docker exec -it airsim-ros2 ros2 topic list
    echo.
)

echo 💡 Benefits of this approach:
echo   - Simple scaling: just change the number
echo   - Automatic port assignment
echo   - Clean container names (px4_1, px4_2, etc.)
echo   - All drones on same network
echo   - No GPS networking issues
echo.
pause 