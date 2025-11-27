@echo off
REM MNSTEVV Auto-Launch Script for Windows + WSL2
REM Automatically detects vEthernet (WSL) IP and launches MNSTEVV with correct configuration

setlocal EnableDelayedExpansion

echo =========================================
echo  MNSTEVV Auto-Launch for Windows + WSL2
echo =========================================
echo.

REM Check if PowerShell is available
where powershell >nul 2>&1
if errorlevel 1 (
    echo ERROR: PowerShell not found in PATH
    echo Please ensure PowerShell is installed
    pause
    exit /b 1
)

REM Check if mnstevv is installed
where mnstevv >nul 2>&1
if errorlevel 1 (
    echo WARNING: mnstevv command not found in PATH
    echo.
    echo Trying alternative methods...
    echo.
    REM Try direct Python execution
    python mnstevv.py --version >nul 2>&1
    if errorlevel 1 (
        echo MNSTEVV not installed. Installing now...
        call install.bat
        if errorlevel 1 (
            echo ERROR: Failed to install MNSTEVV CLI
            pause
            exit /b 1
        )
    ) else (
        echo Found mnstevv.py - will use direct Python execution
        set USE_PYTHON=1
    )
) else (
    set USE_PYTHON=0
)

REM Get WSL2 vEthernet IP
echo Detecting WSL2 vEthernet adapter IP...
for /f "delims=" %%i in ('powershell -ExecutionPolicy Bypass -File "get-wsl2-ip.ps1" -Silent -JsonOutput 2^>nul') do (
    set JSON_OUTPUT=%%i
)

REM Parse JSON output to get IP (simple parsing for success and IP)
echo !JSON_OUTPUT! | findstr /C:"\"success\":true" >nul
if errorlevel 1 (
    echo ERROR: Failed to detect WSL2 IP using get-wsl2-ip.ps1
    echo.
    echo Trying comprehensive vEthernet detection...
    for /f "tokens=*" %%i in ('powershell -Command "(Get-NetIPAddress -AddressFamily IPv4| Where-Object {$_.InterfaceAlias -like \"vEthernet*\"}).IPAddress | Select-Object -First 1"') do (
        set WSL2_IP=%%i
    )
    if "!WSL2_IP!"=="" (
        echo Comprehensive detection also failed. Falling back to manual mode...
        echo Please run one of these PowerShell commands to get your WSL2 IP:
        echo   ^(Get-NetIPAddress -InterfaceAlias "vEthernet ^(WSL^)" -AddressFamily IPv4^).IPAddress
        echo   ^(Get-NetIPAddress -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -AddressFamily IPv4^).IPAddress
        echo   ^(Get-NetIPAddress -AddressFamily IPv4^| Where-Object {$_.InterfaceAlias -like "vEthernet*"}^).IPAddress
        echo.
        set /p WSL2_IP="Enter the IP address manually: "
    ) else (
        echo Comprehensive vEthernet detection succeeded!
    )
) else (
    REM Extract IP from JSON (crude but works for simple JSON)
    for /f "tokens=2 delims=:," %%a in ('echo !JSON_OUTPUT! ^| findstr /C:"\"ip\""') do (
        set TEMP_IP=%%a
        REM Remove quotes and spaces
        set WSL2_IP=!TEMP_IP:"=!
        set WSL2_IP=!WSL2_IP: =!
    )
)

if "!WSL2_IP!"=="" (
    echo ERROR: Could not determine WSL2 IP address
    pause
    exit /b 1
)

echo Detected WSL2 IP: !WSL2_IP!
echo.

REM Parse command line arguments
set NUM_DRONES=3
set PROFILE=integrated
set EXTRA_ARGS=

:parse_args
if "%~1"=="" goto :done_parsing
if /i "%~1"=="--num_drones" (
    set NUM_DRONES=%~2
    shift
    shift
    goto :parse_args
)
if /i "%~1"=="--profile" (
    set PROFILE=%~2
    shift
    shift
    goto :parse_args
)
REM Collect any other arguments
set EXTRA_ARGS=!EXTRA_ARGS! %~1
shift
goto :parse_args

:done_parsing

echo Configuration:
echo   - WSL2 IP: !WSL2_IP!
echo   - Number of drones: !NUM_DRONES!
echo   - Profile: !PROFILE!
echo   - AirSim host: !WSL2_IP!
echo   - PX4 SIM host: !WSL2_IP!
echo.

REM Launch MNSTEVV with detected IP
echo Launching MNSTEVV...
if "!USE_PYTHON!"=="1" (
    echo Command: python mnstevv.py up --num_drones !NUM_DRONES! --profile !PROFILE! --airsim-host !WSL2_IP! --px4-sim-host !WSL2_IP! !EXTRA_ARGS!
    echo.
    python mnstevv.py up --num_drones !NUM_DRONES! --profile !PROFILE! --airsim-host !WSL2_IP! --px4-sim-host !WSL2_IP! !EXTRA_ARGS!
) else (
    echo Command: mnstevv up --num_drones !NUM_DRONES! --profile !PROFILE! --airsim-host !WSL2_IP! --px4-sim-host !WSL2_IP! !EXTRA_ARGS!
    echo.
    mnstevv up --num_drones !NUM_DRONES! --profile !PROFILE! --airsim-host !WSL2_IP! --px4-sim-host !WSL2_IP! !EXTRA_ARGS!
)

if errorlevel 1 (
    echo.
    echo ERROR: MNSTEVV command failed
    pause
    exit /b 1
)

echo.
echo =========================================
echo  Services started successfully!
echo =========================================
echo.
echo Next steps:
echo   1. Ensure AirSim is running on Windows
echo   2. Check status: mnstevv status
echo   3. View logs: mnstevv logs --follow
echo   4. Stop services: mnstevv down
echo.

endlocal