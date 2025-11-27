@echo off
REM Installation script for MNSTEVV CLI on Windows

echo.
echo ============================================
echo  MNSTEVV CLI Installation
echo ============================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+ and try again
    pause
    exit /b 1
)

echo Python detected:
python --version

REM Check if pip is available
pip --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: pip is not installed or not in PATH
    pause
    exit /b 1
)

echo.
echo Installing dependencies...
pip install -r requirements.txt

if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)

echo.
echo Installing MNSTEVV CLI...
pip install -e .

if errorlevel 1 (
    echo ERROR: Failed to install MNSTEVV CLI
    pause
    exit /b 1
)

echo.
echo ============================================
echo  Installation completed successfully!
echo ============================================
echo.
echo Checking if mnstevv command is available...
mnstevv --version >nul 2>&1
if errorlevel 1 (
    echo WARNING: mnstevv command not found in PATH
    echo.
    echo This usually means Python Scripts directory is not in PATH.
    echo.
    echo SOLUTION 1 - Add Python Scripts to PATH:
    echo   1. Find your Python installation directory
    python -c "import sys; print('Python location:', sys.executable)"
    python -c "import sys; import os; print('Scripts directory:', os.path.join(os.path.dirname(sys.executable), 'Scripts'))"
    echo   2. Add the Scripts directory shown above to your PATH environment variable
    echo   3. Restart your command prompt and try again
    echo.
    echo SOLUTION 2 - Use direct execution:
    echo   python mnstevv.py --help
    echo.
    echo SOLUTION 3 - Use the auto-launcher:
    echo   .\mnstevv-auto.bat --num_drones 3
    echo.
) else (
    echo SUCCESS: mnstevv command is available!
    mnstevv --version
)

echo.
echo Usage examples:
echo   mnstevv up --num_drones 3         # Start 3 drones
echo   mnstevv status                    # Check status
echo   mnstevv --help                    # Show all commands
echo.
echo Alternative usage if mnstevv not in PATH:
echo   python mnstevv.py up --num_drones 3
echo   .\mnstevv-auto.bat --num_drones 3  # Auto-detect WSL2 IP
echo.
pause