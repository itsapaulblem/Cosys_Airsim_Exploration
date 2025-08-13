@echo off
echo Starting Mission API Server...
cd /d "%~dp0"
call "airsim_env\Scripts\activate.bat"
python mission_api_server.py
pause
