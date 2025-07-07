@echo off
echo ================================================
echo    Docker Container Monitor
echo ================================================
echo.

:loop
cls
echo [%date% %time%] Container Status:
echo.
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | findstr /V "NAMES"
echo.
echo Port Summary:
echo.
for /f "tokens=1,3 delims= " %%a in ('docker ps --format "{{.Names}} {{.Ports}}"') do (
    echo %%a: %%b
)
echo.
echo ================================================
echo Press Ctrl+C to exit, or wait 10 seconds for refresh...
echo ================================================
timeout /t 10 /nobreak > nul
goto loop