@echo off
REM Real-time monitoring of PX4-AirSim connection

echo ========================================
echo   PX4-AirSim Connection Monitor
echo ========================================

echo 📊 Monitoring PX4 logs for connection events...
echo 🔍 Watching for 'Simulator connected' messages...
echo 💡 Press Ctrl+C to stop monitoring
echo.

REM Start monitoring in background
start "PX4 Log Monitor" cmd /k "docker logs -f px4-single 2>&1 | findstr /i /c:\"simulator\" /c:\"connected\" /c:\"mavlink\" /c:\"tcp\""

echo ✅ Log monitor started in separate window
echo.
echo 📋 What to look for:
echo   ❌ BAD: "Simulator connected" appears immediately (auto-simulator)
echo   ✅ GOOD: "Simulator connected" appears only after launching AirSim
echo.
echo 🔧 Commands to test:
echo   1. Launch AirSim with correct settings.json
echo   2. Watch for connection message in monitor window
echo   3. Check AirSim console for PX4 connection confirmation
echo.

pause