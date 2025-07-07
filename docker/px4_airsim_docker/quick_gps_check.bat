@echo off
REM Quick GPS connectivity checker for PX4-AirSim Docker setup

echo ========================================
echo   Quick GPS Connectivity Check
echo ========================================

echo 📋 Checking container status...
docker-compose ps

echo.
echo 🌐 Testing port connectivity...

REM Test AirSim API port
echo Testing AirSim API (port 41451)...
powershell -Command "try { $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 41451); $client.Close(); Write-Host '✅ AirSim API reachable' } catch { Write-Host '❌ AirSim API not reachable' }"

REM Test PX4 TCP ports
echo Testing PX4 TCP port 4561...
powershell -Command "try { $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 4561); $client.Close(); Write-Host '✅ PX4 TCP 4561 reachable' } catch { Write-Host '❌ PX4 TCP 4561 not reachable' }"

echo Testing PX4 TCP port 4562...
powershell -Command "try { $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 4562); $client.Close(); Write-Host '✅ PX4 TCP 4562 reachable' } catch { Write-Host '❌ PX4 TCP 4562 not reachable' }"

echo.
echo 📊 Container logs summary...
echo --- PX4 Drone 1 Recent Logs ---
docker logs --tail 10 px4-drone-1 2>nul

echo.
echo --- PX4 Drone 2 Recent Logs ---
docker logs --tail 10 px4-drone-2 2>nul

echo.
echo 💡 Quick Solutions:
echo   1. Ensure AirSim is running with correct settings.json
echo   2. Wait 30-60 seconds for GPS initialization
echo   3. Check Docker network: docker network ls
echo   4. Run full diagnosis: python diagnose_gps_docker.py

pause