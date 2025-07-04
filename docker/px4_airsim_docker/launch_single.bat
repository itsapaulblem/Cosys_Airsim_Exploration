@echo off
echo ================================================
echo    PX4 Single Drone Launcher for AirSim
echo ================================================
echo.

echo [INFO] Building Docker image...
docker-compose -f docker-compose.yml build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker image!
    pause
    exit /b 1
)

echo [INFO] Starting single drone instance...
docker-compose -f docker-compose.yml --profile single up -d

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start single drone instance!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    Single Drone Started Successfully!
echo ================================================
echo.
echo Port Configuration:
echo   TCP (AirSim):     4561
echo   UDP (MAVLink):    14541/14581
echo.
echo AirSim Settings.json:
echo {
echo   "SettingsVersion": 2.0,
echo   "SimMode": "Multirotor",
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
echo [INFO] To stop: docker-compose -f docker-compose.yml --profile single down
echo [INFO] To view logs: docker-compose -f docker-compose.yml logs -f
echo.
pause 