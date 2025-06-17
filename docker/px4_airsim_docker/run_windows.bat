@echo off
echo Starting PX4-AirSim Docker Container on Windows...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Docker is running, proceeding with container startup...

REM Stop any existing container
docker-compose down

REM Build and start the container
echo Building and starting PX4-AirSim container...
docker-compose up --build

echo Container stopped. Press any key to exit.
pause 