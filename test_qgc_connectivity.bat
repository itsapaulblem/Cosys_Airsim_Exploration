@echo off
echo ================================================
echo   QGroundControl Connectivity Test
echo ================================================
echo.

echo [INFO] Testing port forwarding and PX4 connectivity...
echo.

echo [TEST 1] Checking WSL2 PX4 instances...
wsl bash -c "ps aux | grep px4 | grep -v grep"

if %errorlevel% neq 0 (
    echo [ERROR] No PX4 instances found in WSL2!
    echo [INFO] Run 'launch_px4_native.bat 2' to start PX4 instances
    goto :end
)

echo.
echo [TEST 2] Checking PX4 MAVLink ports in WSL2...
wsl bash -c "ss -tuln | grep 145"

if %errorlevel% neq 0 (
    echo [ERROR] No MAVLink ports found!
    goto :end
)

echo.
echo [TEST 3] Checking Windows port forwarding rules...
netsh interface portproxy show all | findstr 145

if %errorlevel% neq 0 (
    echo [ERROR] No port forwarding rules found!
    echo [INFO] Run 'setup_qgc_port_forwarding.bat' as Administrator
    goto :end
)

echo.
echo [TEST 4] Testing connectivity to forwarded ports...
echo [INFO] Testing connection to localhost:14541 (should reach WSL2:14581)

REM Try to connect to the forwarded port using PowerShell
powershell -Command "try { $client = New-Object System.Net.Sockets.UdpClient; $client.Connect('127.0.0.1', 14541); Write-Host '[SUCCESS] Port 14541 is reachable'; $client.Close() } catch { Write-Host '[ERROR] Cannot reach port 14541' }"

echo.
echo [INFO] Testing connection to localhost:14542 (should reach WSL2:14582)
powershell -Command "try { $client = New-Object System.Net.Sockets.UdpClient; $client.Connect('127.0.0.1', 14542); Write-Host '[SUCCESS] Port 14542 is reachable'; $client.Close() } catch { Write-Host '[ERROR] Cannot reach port 14542' }"

echo.
echo ================================================
echo   QGroundControl Setup Instructions
echo ================================================
echo.
echo If all tests passed, configure QGroundControl:
echo.
echo 1. Open QGroundControl
echo 2. Go to Settings (gear icon) ^> Comm Links
echo 3. Click "Add" to create a new connection
echo 4. Configure as follows:
echo.
echo    Connection Type: UDP
echo    Name: PX4_Drone1
echo    Listening Port: 14541
echo    Server Address: 127.0.0.1
echo.
echo 5. Click "OK" and then "Connect"
echo.
echo For multiple drones, repeat with ports 14542, 14543, etc.
echo.

:end
pause 