@echo off
REM AirSim Settings Configuration Helper

echo ========================================
echo   AirSim Settings Configuration
echo ========================================

set AIRSIM_DIR=%USERPROFILE%\Documents\AirSim
set SETTINGS_FILE=%AIRSIM_DIR%\settings.json

REM Create AirSim directory if it doesn't exist
if not exist "%AIRSIM_DIR%" (
    echo 📁 Creating AirSim directory: %AIRSIM_DIR%
    mkdir "%AIRSIM_DIR%"
)

REM Backup existing settings if they exist
if exist "%SETTINGS_FILE%" (
    echo 💾 Backing up existing settings to settings_backup.json
    copy "%SETTINGS_FILE%" "%AIRSIM_DIR%\settings_backup.json" >nul
)

REM Determine which settings file to use
set SOURCE_FILE=
if "%1"=="single" (
    set SOURCE_FILE=%~dp0settings\airsim_px4_single_settings.json
    echo 🚁 Configuring for single drone setup
) else if "%1"=="multi" (
    set SOURCE_FILE=%~dp0settings\airsim_px4_multi_settings.json
    echo 🚁🚁🚁 Configuring for multi-drone setup
) else (
    echo ❓ Please specify configuration type:
    echo   copy_airsim_settings.bat single    (for single drone)
    echo   copy_airsim_settings.bat multi     (for multiple drones)
    echo.
    goto :end
)

REM Copy the settings file
if exist "%SOURCE_FILE%" (
    echo 📋 Copying %SOURCE_FILE%
    echo    to %SETTINGS_FILE%
    copy "%SOURCE_FILE%" "%SETTINGS_FILE%" >nul
    
    if %ERRORLEVEL% equ 0 (
        echo ✅ AirSim settings configured successfully!
        echo.
        echo 📍 Next steps:
        echo   1. Launch AirSim (Unreal Engine)
        echo   2. Start your PX4 containers if not already running
        echo   3. Check AirSim console for connection messages
        echo.
        echo 🔧 Configuration details:
        echo   - Settings file: %SETTINGS_FILE%
        if "%1"=="single" (
            echo   - Vehicle: PX4 on TCP port 4561
        ) else (
            echo   - Vehicles: Drone1-5 on TCP ports 4561-4565
        )
        echo   - GPS coordinates: 47.641468, -122.140165
    ) else (
        echo ❌ Failed to copy settings file
    )
) else (
    echo ❌ Source settings file not found: %SOURCE_FILE%
)

echo.
:end
pause