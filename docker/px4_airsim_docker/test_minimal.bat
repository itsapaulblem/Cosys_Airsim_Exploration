@echo off
echo Testing PX4 Minimal Setup for AirSim...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Building minimal PX4 container...
docker build -f Dockerfile.simple -t px4-minimal-test . --no-cache

if %errorlevel% neq 0 (
    echo ERROR: Container build failed
    pause
    exit /b 1
)

echo Starting container with minimal setup...
docker run --rm -p 4560:4560 -p 14550:14550 px4-minimal-test

echo Press any key to exit.
pause 