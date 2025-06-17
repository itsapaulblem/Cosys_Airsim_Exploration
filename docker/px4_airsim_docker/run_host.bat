@echo off
echo Starting PX4 Container with Host Networking...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Docker is running, proceeding with container startup...

REM Stop any existing container
docker-compose -f docker-compose.host.yml down

REM Clean up any existing containers to force rebuild
docker container prune -f

REM Build and start the container with force rebuild
echo Building and starting PX4 container with host networking...
echo This will make PX4 available directly on your Windows ports
docker-compose -f docker-compose.host.yml up --build --force-recreate

echo Container stopped. Press any key to exit.
pause 