@echo off
echo ================================================
echo    Single Drone PX4 AirSim Setup
echo ================================================
echo.

echo [INFO] Stopping any existing containers...
docker-compose down

echo [INFO] Building Docker image...
docker-compose build
if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker image!
    pause
    exit /b 1
)

echo [INFO] Starting single PX4 drone...
docker-compose up -d
if %errorlevel% neq 0 (
    echo [ERROR] Failed to start drone!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    Single Drone Started Successfully!
echo ================================================
echo.
echo Port Configuration:
echo   AirSim TCP:      4561
echo   MAVLink Local:   14541  
echo   MAVLink Remote:  14581
echo   QGroundControl:  14550
echo.
echo AirSim Settings.json:
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
echo Management Commands:
echo   View logs:        docker-compose logs -f
echo   Stop:             docker-compose down
echo   Container status: docker-compose ps
echo   PX4 console:      docker exec -it px4-single /bin/bash
echo.
pause