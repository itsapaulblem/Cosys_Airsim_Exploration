@echo off
echo ===============================================================
echo   Complete Mission Control System
echo ===============================================================
echo.
echo Starting both API Server and Streamlit UI...
echo.

REM Start API Server in separate window
start "Mission API Server" start_api_server.bat

REM Wait a moment
timeout /t 5 >nul

REM Start Streamlit UI in separate window  
start "Streamlit UI" start_streamlit_ui.bat

echo.
echo ===============================================================
echo   System Started!
echo   API Server: http://localhost:8000
echo   Streamlit UI: http://localhost:8501
echo   
echo   Close both windows to stop the system
echo ===============================================================
echo.
pause
