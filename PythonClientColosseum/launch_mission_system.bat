@echo off
echo ===============================================================
echo   Mission Control System - Quick Start
echo ===============================================================
echo.

echo Starting API Server...
start "API Server" cmd /c "call "airsim_env_20250611_100857\Scripts\activate.bat" & python mission_api_server.py & pause"

echo Waiting for API server to start...
timeout /t 5 >nul

echo Starting Streamlit UI...
start "Streamlit UI" cmd /c "call "streamlit_env_20250611_101013\Scripts\activate.bat" & streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A & pause"

echo.
echo ===============================================================
echo   Mission Control System Started!
echo   API Server: http://localhost:8000
echo   Streamlit UI: http://localhost:8501
echo ===============================================================
pause
