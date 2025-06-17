@echo off
echo Starting PX4 Container for AirSim Connection (IP: 172.28.240.1)...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Docker is running, proceeding with container startup...

REM Stop any existing container
docker-compose -f docker-compose.airsim.yml down

REM Build and start the container
echo Building and starting PX4 container for AirSim...
echo Container will be available at 172.28.240.1:4560
docker-compose -f docker-compose.airsim.yml up --build

echo Container stopped. Press any key to exit.
pause 