@echo off
echo Starting Streamlit Mission Control...
cd /d "%~dp0"
call "streamlit_env\Scripts\activate.bat"
streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A
pause
