@echo off
REM Network cleanup script for PX4-AirSim Docker setup
REM Removes conflicting networks and containers

echo ========================================
echo   Docker Network Cleanup
echo ========================================

echo 🧹 Stopping and removing all PX4 containers...
docker-compose down --remove-orphans

echo 🗑️  Removing conflicting networks...
docker network ls | findstr airsim-network
if %ERRORLEVEL% equ 0 (
    echo Found existing airsim-network, removing...
    docker network rm airsim-network 2>nul
    if %ERRORLEVEL% equ 0 (
        echo ✅ Network removed successfully
    ) else (
        echo ⚠️  Network might be in use, forcing removal...
        docker network prune -f
    )
) else (
    echo ✅ No conflicting networks found
)

echo 🧽 Cleaning up unused Docker resources...
docker system prune -f

echo ✅ Cleanup completed!
echo.
echo 📋 Current Docker networks:
docker network ls
echo.
echo 💡 You can now run launch_with_gps_fix.bat safely

pause