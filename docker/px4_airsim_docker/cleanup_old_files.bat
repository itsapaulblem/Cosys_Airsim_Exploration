@echo off
echo ================================================
echo    PX4 AirSim Docker Cleanup Script
echo ================================================
echo.
echo This script will remove all the old redundant files
echo and keep only the clean, consolidated setup.
echo.
echo WARNING: This will permanently delete the old files!
echo Make sure you've tested the new setup first.
echo.
echo Files that will be KEPT:
echo   - docker-compose-clean.yml
echo   - Dockerfile-clean  
echo   - start_px4.sh
echo   - launch_single.bat
echo   - launch_multi.bat
echo   - stop.bat
echo   - README_CLEAN.md
echo   - cleanup_old_files.bat (this script)
echo.
echo Files that will be REMOVED:
echo   - docker-compose.yml (old version)
echo   - docker-compose.simple.yml
echo   - docker-compose.host.yml
echo   - docker-compose.airsim.yml
echo   - Dockerfile.px4-instance
echo   - Dockerfile.simple
echo   - Dockerfile (old version)
echo   - launch_px4_simple.bat
echo   - launch_px4_swarm.bat
echo   - launch_4_drone_swarm.bat
echo   - stop_px4_swarm.bat
echo   - stop_px4_swarm.sh
echo   - launch_px4_swarm.sh
echo   - start_px4_minimal.sh
echo   - start_px4_airsim.sh
echo   - test_px4_startup.sh
echo   - start_px4_simple.sh
echo   - start_px4_airsim_gcs.sh
echo   - px4_send_command.bat
echo   - px4_container_shell.bat
echo   - connect_px4_console.bat
echo   - px4_direct_console.bat
echo   - debug_container.bat
echo   - run_host.bat
echo   - run_airsim.bat
echo   - test_network.bat
echo   - run_simple.bat
echo   - run_windows.bat
echo   - check_docker_ip.bat
echo.

set /p confirm="Are you sure you want to proceed? (y/N): "
if /i not "%confirm%"=="y" (
    echo Cleanup cancelled.
    pause
    exit /b 0
)

echo.
echo [INFO] Starting cleanup...

REM Remove old Docker Compose files
echo Removing old Docker Compose files...
if exist docker-compose.yml del docker-compose.yml
if exist docker-compose.simple.yml del docker-compose.simple.yml
if exist docker-compose.host.yml del docker-compose.host.yml
if exist docker-compose.airsim.yml del docker-compose.airsim.yml

REM Remove old Dockerfiles
echo Removing old Dockerfiles...
if exist Dockerfile.px4-instance del Dockerfile.px4-instance
if exist Dockerfile.simple del Dockerfile.simple
if exist Dockerfile del Dockerfile

REM Remove old batch files
echo Removing old batch files...
if exist launch_px4_simple.bat del launch_px4_simple.bat
if exist launch_px4_swarm.bat del launch_px4_swarm.bat
if exist launch_4_drone_swarm.bat del launch_4_drone_swarm.bat
if exist stop_px4_swarm.bat del stop_px4_swarm.bat
if exist px4_send_command.bat del px4_send_command.bat
if exist px4_container_shell.bat del px4_container_shell.bat
if exist connect_px4_console.bat del connect_px4_console.bat
if exist px4_direct_console.bat del px4_direct_console.bat
if exist debug_container.bat del debug_container.bat
if exist run_host.bat del run_host.bat
if exist run_airsim.bat del run_airsim.bat
if exist test_network.bat del test_network.bat
if exist run_simple.bat del run_simple.bat
if exist run_windows.bat del run_windows.bat
if exist check_docker_ip.bat del check_docker_ip.bat

REM Remove old shell scripts
echo Removing old shell scripts...
if exist stop_px4_swarm.sh del stop_px4_swarm.sh
if exist launch_px4_swarm.sh del launch_px4_swarm.sh
if exist start_px4_minimal.sh del start_px4_minimal.sh
if exist start_px4_airsim.sh del start_px4_airsim.sh
if exist test_px4_startup.sh del test_px4_startup.sh
if exist start_px4_simple.sh del start_px4_simple.sh
if exist start_px4_airsim_gcs.sh del start_px4_airsim_gcs.sh

echo.
echo [INFO] Renaming clean files to standard names...

REM Rename the clean files to standard names
if exist docker-compose-clean.yml (
    move docker-compose-clean.yml docker-compose.yml
    echo Renamed docker-compose-clean.yml to docker-compose.yml
)

if exist Dockerfile-clean (
    move Dockerfile-clean Dockerfile
    echo Renamed Dockerfile-clean to Dockerfile
)

echo.
echo ================================================
echo    Cleanup Complete!
echo ================================================
echo.
echo Your Docker setup is now clean and simplified:
echo.
echo Core Files:
echo   - docker-compose.yml (renamed from docker-compose-clean.yml)
echo   - Dockerfile (renamed from Dockerfile-clean)
echo   - start_px4.sh
echo   - launch_single.bat
echo   - launch_multi.bat
echo   - stop.bat
echo   - README_CLEAN.md
echo.
echo Usage:
echo   Single drone:    launch_single.bat
echo   Multi-drone:     launch_multi.bat ^<number^>
echo   Stop all:        stop.bat
echo.
echo You can now delete this cleanup script if you want.
echo.
pause 