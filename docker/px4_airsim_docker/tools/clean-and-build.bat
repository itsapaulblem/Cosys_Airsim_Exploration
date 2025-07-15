@echo off

echo 🧹 Cleaning up existing images and containers...

REM Stop and remove any running containers
docker-compose -f docker-compose.host-network.yml down 2>nul

REM Remove existing images with this name
docker rmi px4-ultra-swarm-host:latest 2>nul

REM Clean up build cache
docker builder prune -f

echo ✅ Cleanup complete!
echo.

REM Now build the image
call build-host-network.bat