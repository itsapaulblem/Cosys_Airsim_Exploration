@echo off
REM Step-by-step PX4-AirSim connection troubleshooting

echo ========================================
echo   PX4-AirSim Connection Troubleshooting
echo ========================================

echo Step 1: Clean setup
echo 🧹 Cleaning up any existing setup...
docker-compose down --remove-orphans
docker network rm px4_airsim_docker_airsim-network 2>nul
docker network rm airsim-network 2>nul
timeout /t 3 /nobreak

echo.
echo Step 2: Starting single PX4 container
echo 🚁 Starting single PX4 drone for testing...
docker-compose --profile single up -d
timeout /t 10 /nobreak

echo.
echo Step 3: Checking container status
echo 📋 Container status:
docker-compose ps

echo.
echo Step 4: Testing TCP port connectivity
echo 🌐 Testing if PX4 TCP port 4561 is reachable...
timeout /t 5 /nobreak
powershell -Command "try { $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 4561); $client.Close(); Write-Host '✅ Port 4561 is reachable - PX4 is listening' } catch { Write-Host '❌ Port 4561 not reachable - PX4 might not be ready yet' }"

echo.
echo Step 5: Checking PX4 logs for readiness
echo 📊 Recent PX4 logs (looking for 'simulator connected'):
docker logs --tail 20 px4-single | findstr /i "simulator connected" 2>nul
if %ERRORLEVEL% equ 0 (
    echo ✅ PX4 reports simulator connected
) else (
    echo ⏳ PX4 not ready yet, showing last 10 lines:
    docker logs --tail 10 px4-single
)

echo.
echo Step 6: AirSim Configuration Check
echo ⚙️  Copy the correct settings file:
echo.
echo 1. Copy this file to your AirSim Documents folder:
echo    "%~dp0settings\airsim_px4_single_settings.json"
echo.
echo 2. Rename it to: settings.json
echo.
echo 3. Location should be:
echo    Windows: %%USERPROFILE%%\Documents\AirSim\settings.json
echo    Linux: ~/Documents/AirSim/settings.json
echo.

echo Step 7: Launch AirSim
echo 🎮 Now launch AirSim (Unreal Engine) and you should see:
echo   - PX4 connection established in AirSim console
echo   - Vehicle spawned and connected
echo.

echo Step 8: Verify connection
echo 🔗 In AirSim console, you should see messages like:
echo   - "LogMavLinkCom: MavLinkConnection connecting to TCP 127.0.0.1:4561"
echo   - "LogMavLinkCom: Connected to PX4 simulator"
echo.

echo 💡 If connection fails:
echo   1. Check PX4 logs: docker logs px4-single
echo   2. Check port binding: netstat -an | findstr 4561
echo   3. Verify settings.json is in correct location
echo   4. Restart both PX4 and AirSim
echo.

pause