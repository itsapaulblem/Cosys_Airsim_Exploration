@echo off
echo Finding Docker Container IP Address...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo.
echo === Docker Container IPs ===
docker ps --format "table {{.Names}}\t{{.Ports}}"

echo.
echo === Network Information ===
for /f "tokens=*" %%i in ('docker ps -q') do (
    echo Container: %%i
    docker inspect %%i --format="{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}"
)

echo.
echo === Port Mappings ===
docker port px4-airsim-simple-container 2>nul
if %errorlevel% neq 0 (
    docker port px4-airsim-container 2>nul
    if %errorlevel% neq 0 (
        docker port px4-airsim-custom-container 2>nul
    )
)

echo.
echo === Docker Desktop IP (usually works) ===
echo Try connecting AirSim to: localhost:4560
echo Or try: 127.0.0.1:4560

echo.
echo Press any key to exit...
pause >nul 