@echo off
echo Starting PX4 SITL Docker Container (Simple Version) on Windows...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Docker is running, proceeding with container startup...

REM Stop any existing container
docker-compose -f docker-compose.simple.yml down

REM Build and start the container
echo Building and starting PX4 SITL container (simple version)...
docker-compose -f docker-compose.simple.yml up --build

echo Container stopped. Press any key to exit.
pause 