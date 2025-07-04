@echo off
echo ================================================
echo    Fixed PX4 AirSim Docker Launcher
echo ================================================
echo.

REM Check if argument provided
if "%~1"=="" (
    echo Usage: %~nx0 [method] [number_of_drones]
    echo.
    echo Methods:
    echo   explicit [1-5]   - Use explicit service definitions ^(recommended^)
    echo   scaled [1-10]    - Use Docker Compose scaling ^(experimental^)
    echo.
    echo Examples:
    echo   %~nx0 explicit 1     - Single drone using explicit definition
    echo   %~nx0 explicit 3     - Three drones using explicit definitions  
    echo   %~nx0 scaled 2       - Two drones using scaling ^(may have issues^)
    echo.
    pause
    exit /b 1
)

set METHOD=%~1
set NUM_DRONES=%~2

REM Validate method
if /i not "%METHOD%"=="explicit" if /i not "%METHOD%"=="scaled" (
    echo [ERROR] Invalid method: %METHOD%
    echo Valid methods: explicit, scaled
    pause
    exit /b 1
)

REM Validate number of drones
if "%NUM_DRONES%"=="" set NUM_DRONES=1

if %NUM_DRONES% LSS 1 (
    echo [ERROR] Number of drones must be at least 1!
    pause
    exit /b 1
)

if /i "%METHOD%"=="explicit" (
    if %NUM_DRONES% GTR 5 (
        echo [ERROR] Explicit method supports maximum 5 drones!
        pause
        exit /b 1
    )
) else (
    if %NUM_DRONES% GTR 10 (
        echo [ERROR] Scaled method supports maximum 10 drones!
        pause
        exit /b 1
    )
)

echo [INFO] Creating shared data directory...
if not exist "shared_data" mkdir shared_data

echo [INFO] Building Docker image...
docker-compose -f docker-compose-scaled-fixed.yml build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker image!
    pause
    exit /b 1
)

REM Clean up any existing containers
echo [INFO] Cleaning up existing containers...
docker-compose -f docker-compose-scaled-fixed.yml down > nul 2>&1

if /i "%METHOD%"=="explicit" (
    echo [INFO] Starting %NUM_DRONES% drone^(s^) using explicit method...
    
    REM Build the profile list for explicit services
    set PROFILES=explicit
    set SERVICE_LIST=px4-1
    
    if %NUM_DRONES% GEQ 2 set SERVICE_LIST=%SERVICE_LIST% px4-2
    if %NUM_DRONES% GEQ 3 set SERVICE_LIST=%SERVICE_LIST% px4-3
    if %NUM_DRONES% GEQ 4 set SERVICE_LIST=%SERVICE_LIST% px4-4
    if %NUM_DRONES% GEQ 5 set SERVICE_LIST=%SERVICE_LIST% px4-5
    
    docker-compose -f docker-compose-scaled-fixed.yml --profile explicit up %SERVICE_LIST% -d
    
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to start explicit drone instances!
        pause
        exit /b 1
    )
    
    echo.
    echo ================================================
    echo    %NUM_DRONES% Drone^(s^) Started ^(Explicit Method^)
    echo ================================================
    echo.
    
    echo Port Configuration:
    for /L %%i in (1,1,%NUM_DRONES%) do (
        set /a tcp_port=4560+%%i
        set /a local_port=14540+%%i
        set /a remote_port=14580+%%i
        set /a qgc_port=14549+%%i
        call :show_drone_ports %%i
    )
    
) else (
    echo [INFO] Starting %NUM_DRONES% drone^(s^) using scaled method...
    
    docker-compose -f docker-compose-scaled-fixed.yml up --scale px4=%NUM_DRONES% -d
    
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to start scaled drone instances!
        pause
        exit /b 1
    )
    
    echo.
    echo ================================================
    echo    %NUM_DRONES% Drone^(s^) Started ^(Scaled Method^)
    echo ================================================
    echo.
    echo Note: Scaled method assigns random host ports.
    echo Use "docker ps" to see actual port mappings.
)

goto :show_settings

:show_drone_ports
set /a tcp_port=4560+%1
set /a local_port=14540+%1
set /a remote_port=14580+%1
set /a qgc_port=14549+%1
echo   PX4_Drone%1:  TCP=%tcp_port% ^| MAVLink=%local_port%/%remote_port% ^| QGC=%qgc_port%
goto :eof

:show_settings
echo.
echo AirSim Settings.json Generation:
echo   python generate_settings.py %NUM_DRONES%
echo.

if /i "%METHOD%"=="explicit" (
    echo Generated settings will use these ports:
    for /L %%i in (1,1,%NUM_DRONES%) do (
        set /a tcp_port=4560+%%i
        echo   PX4_Drone%%i: TcpPort %tcp_port%
    )
) else (
    echo For scaled method, check actual ports with: docker ps
)

echo.
echo Management Commands:
echo   View logs:        docker-compose -f docker-compose-scaled-fixed.yml logs
echo   Stop all:         docker-compose -f docker-compose-scaled-fixed.yml down
echo   Container status: docker-compose -f docker-compose-scaled-fixed.yml ps
echo.

echo Troubleshooting:
echo   List containers:  docker ps
echo   Check networks:   docker network ls
echo   GPS diagnosis:    python diagnose_gps_docker.py
echo   PX4 console:      docker exec -it px4-drone-1 /Scripts/px4_console.sh
echo.

echo 💡 Key Improvements:
echo   - Each container has unique ports ^(no conflicts^)
echo   - Explicit method: predictable port assignments
echo   - Scaled method: Docker assigns ports automatically
echo   - All containers on same network ^(no GPS issues^)
echo.
pause 