@echo off
echo ================================================
echo    Docker Cleanup Script
echo ================================================
echo.

echo [INFO] Stopping all containers...
docker-compose -f ../single_drone/docker-compose.yml down 2>nul
docker-compose -f ../multi_drone/docker-compose.yml down 2>nul

echo [INFO] Removing any orphaned containers...
docker container prune -f

echo [INFO] Checking for remaining PX4 containers...
for /f "tokens=1" %%i in ('docker ps -a --filter "name=px4" --format "{{.Names}}" 2^>nul') do (
    echo   Removing %%i...
    docker rm -f %%i 2>nul
)

echo [INFO] Removing unused networks...
docker network prune -f

echo [INFO] Current container status:
docker ps -a --filter "name=px4" --format "table {{.Names}}\t{{.Status}}"

echo.
echo [INFO] Cleanup complete!
echo.
pause