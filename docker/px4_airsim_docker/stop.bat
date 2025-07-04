@echo off
echo ================================================
echo    Stopping All PX4 Drone Instances
echo ================================================
echo.

echo [INFO] Stopping all Docker Compose profiles...

REM Stop all possible profiles
docker-compose -f docker-compose.yml --profile single down
docker-compose -f docker-compose.yml --profile drone-2 down  
docker-compose -f docker-compose.yml --profile drone-3 down
docker-compose -f docker-compose.yml --profile drone-4 down
docker-compose -f docker-compose.yml --profile drone-5 down

echo.
echo [INFO] Cleaning up any remaining containers...
for /f "tokens=*" %%i in ('docker ps -q --filter "name=px4-"') do (
    docker stop %%i
)
for /f "tokens=*" %%i in ('docker ps -aq --filter "name=px4-"') do (
    docker rm %%i
)

echo.
echo [INFO] All PX4 instances have been stopped and cleaned up.
echo.
pause 