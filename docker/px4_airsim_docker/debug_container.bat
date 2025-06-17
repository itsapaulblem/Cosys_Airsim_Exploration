@echo off
echo Debugging PX4 Container Build...

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo === Building container with debug output ===
docker build -f Dockerfile.simple -t px4-debug . --no-cache --progress=plain

if %errorlevel% neq 0 (
    echo ERROR: Container build failed
    pause
    exit /b 1
)

echo === Running container interactively ===
echo You can manually check the PX4 build inside the container
docker run -it --rm px4-debug bash

echo Press any key to exit.
pause 