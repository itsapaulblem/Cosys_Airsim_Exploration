@echo off
echo Testing Docker and Network Connectivity...

echo.
echo === Docker Status ===
docker info
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running properly
    goto :end
)

echo.
echo === Testing GitHub Connectivity ===
curl -I https://github.com
if %errorlevel% neq 0 (
    echo ERROR: Cannot reach GitHub
    echo This might be a network/firewall issue
    goto :end
)

echo.
echo === Testing Docker Network ===
docker run --rm alpine:latest ping -c 3 8.8.8.8
if %errorlevel% neq 0 (
    echo ERROR: Docker containers cannot reach internet
    echo Check Docker Desktop network settings
    goto :end
)

echo.
echo === Testing Git Clone in Docker ===
docker run --rm alpine/git:latest clone --depth 1 https://github.com/PX4/PX4-Autopilot.git /tmp/test
if %errorlevel% neq 0 (
    echo ERROR: Git clone fails in Docker
    echo This could be a DNS or proxy issue
    goto :end
)

echo.
echo === All Tests Passed ===
echo Your Docker setup should work fine for building the PX4 container

:end
echo.
echo Press any key to exit...
pause >nul 