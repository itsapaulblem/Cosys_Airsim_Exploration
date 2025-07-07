@echo off
REM Test script to verify the external simulator fix

echo ========================================
echo   Testing External Simulator Fix
echo ========================================

echo 🧹 Cleaning up existing containers...
docker-compose down --remove-orphans
docker network prune -f
timeout /t 3 /nobreak

echo.
echo 🚀 Building and starting single PX4 container with fix...
docker-compose --profile single up --build -d
timeout /t 15 /nobreak

echo.
echo 📋 Container Status:
docker-compose ps

echo.
echo 🔍 Testing the fix - checking PX4 logs...
echo.
echo "Looking for 'Simulator connected' in logs:"
docker logs px4-single 2>&1 | findstr /i "simulator connected"
if %ERRORLEVEL% equ 0 (
    echo.
    echo "❌ PROBLEM: PX4 still reports 'Simulator connected' without external simulator"
    echo "   This means the auto-simulator is still running."
    echo "   The fix needs more work."
) else (
    echo "✅ GOOD: No premature 'Simulator connected' message found"
    echo "   PX4 should be waiting for external simulator (AirSim)"
)

echo.
echo "Recent PX4 log output:"
echo "====================="
docker logs --tail 15 px4-single

echo.
echo 🌐 Testing TCP port 4561 availability...
timeout /t 3 /nobreak
powershell -Command "try { $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 4561); $client.Close(); Write-Host '✅ Port 4561 is listening' } catch { Write-Host '❌ Port 4561 not available yet - PX4 might still be starting' }"

echo.
echo 💡 Next Steps:
echo   1. If the fix worked: Launch AirSim and check for connection
echo   2. If still broken: Review Docker logs and startup configuration
echo   3. Monitor: docker logs -f px4-single

pause