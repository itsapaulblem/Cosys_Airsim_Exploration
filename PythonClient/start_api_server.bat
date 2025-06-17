@echo off
echo Starting Mission API Server...
cd /d "%~dp0"
call "cosys_mission_env\Scripts\activate.bat"
python mission_api_server.py
pause
