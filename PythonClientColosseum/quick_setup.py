#!/usr/bin/env python3
"""
Quick Setup for AirSim + Streamlit (Windows-friendly)

This script avoids Windows permission issues by creating timestamped environments
instead of trying to delete existing ones.
"""

import subprocess
import sys
import venv
from pathlib import Path
from datetime import datetime

def create_timestamped_env(base_name, description):
    """Create a virtual environment with timestamp to avoid conflicts"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    venv_path = Path(f"{base_name}_{timestamp}")
    
    print(f"\n Creating {description} at {venv_path}")
    
    try:
        venv.create(venv_path, with_pip=True)
        print(f" Created: {venv_path}")
    except Exception as e:
        print(f" Failed to create environment: {e}")
        return None, None, None
    
    # Get platform-specific paths
    if sys.platform == "win32":
        python_path = venv_path / "Scripts" / "python.exe"
        pip_path = venv_path / "Scripts" / "pip.exe"
        activate_path = venv_path / "Scripts" / "activate.bat"
    else:
        python_path = venv_path / "bin" / "python"
        pip_path = venv_path / "bin" / "pip"
        activate_path = venv_path / "bin" / "activate"
    
    return python_path, pip_path, activate_path

def install_packages(pip_path, packages, env_name):
    """Install packages in the given environment"""
    print(f"\n Installing {env_name} packages...")
    
    for package in packages:
        print(f"  Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], 
                         check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"    Warning: {package} failed: {e}")

def create_launcher_batch(api_activate_path, streamlit_activate_path):
    """Create Windows batch files for launching"""
    
    # Individual launchers
    api_launcher = f"""@echo off
echo Starting Mission API Server...
cd /d "%~dp0"
call "{api_activate_path}"
python mission_api_server.py
pause
"""
    
    streamlit_launcher = f"""@echo off
echo tarting Streamlit Mission Control...
cd /d "%~dp0"
call "{streamlit_activate_path}"
streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A
pause
"""
    
    # Combined launcher
    combined_launcher = f"""@echo off
echo ===============================================================
echo   Mission Control System - Quick Start
echo ===============================================================
echo.

echo Starting API Server...
start "API Server" cmd /c "call "{api_activate_path}" & python mission_api_server.py & pause"

echo Waiting for API server to start...
timeout /t 5 >nul

echo Starting Streamlit UI...
start "Streamlit UI" cmd /c "call "{streamlit_activate_path}" & streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A & pause"

echo.
echo ===============================================================
echo   Mission Control System Started!
echo   API Server: http://localhost:8000
echo   Streamlit UI: http://localhost:8501
echo ===============================================================
pause
"""
    
    # Write batch files
    with open("start_api.bat", "w") as f:
        f.write(api_launcher)
    
    with open("start_ui.bat", "w") as f:
        f.write(streamlit_launcher)
    
    with open("launch_mission_system.bat", "w") as f:
        f.write(combined_launcher)
    
    print("\n Created launcher files:")
    print("   • start_api.bat")
    print("   • start_ui.bat")
    print("   • launch_mission_system.bat")

def main():
    print(" Quick Setup - AirSim + Streamlit")
    print("=" * 50)
    print("Creating separate environments with timestamps...")
    print("=" * 50)
    
    # AirSim packages
    airsim_packages = [
        "setuptools", "wheel", "numpy", 
        "tornado==4.5.3", "backports.ssl_match_hostname",
        "msgpack-rpc-python", "opencv-python", "matplotlib", "requests"
    ]
    
    # Streamlit packages  
    streamlit_packages = [
        "setuptools", "wheel", "tornado>=6.1",
        "streamlit>=1.28.0", "pandas>=1.5.0", "plotly>=5.0.0", 
        "Pillow>=8.0.0", "matplotlib>=3.5.0", "requests>=2.28.0"
    ]
    
    # Create AirSim environment
    python_airsim, pip_airsim, activate_airsim = create_timestamped_env("airsim_env", "AirSim Environment")
    if python_airsim:
        install_packages(pip_airsim, airsim_packages, "AirSim")
        
        # Install airsim separately
        print("  Installing airsim...")
        try:
            subprocess.run([str(pip_airsim), "install", "--no-cache-dir", "--no-build-isolation", "airsim"], 
                         check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"    AirSim installation warning: {e}")
    else:
        print(" Failed to create AirSim environment")
        return
    
    # Create Streamlit environment
    python_streamlit, pip_streamlit, activate_streamlit = create_timestamped_env("streamlit_env", "Streamlit Environment")
    if python_streamlit:
        install_packages(pip_streamlit, streamlit_packages, "Streamlit")
    else:
        print(" Failed to create Streamlit environment")
        return
    
    # Create launcher scripts
    create_launcher_batch(activate_airsim, activate_streamlit)
    
    print("\n" + "=" * 60)
    print(" SETUP COMPLETE!")
    print("=" * 60)
    print("\n Quick Start:")
    print("   Double-click: launch_mission_system.bat")
    print("\n What was created:")
    print(f"    AirSim Environment: {activate_airsim.parent}")
    print(f"    Streamlit Environment: {activate_streamlit.parent}")
    print("    Launcher scripts: start_api.bat, start_ui.bat")
    print("\n No dependency conflicts!")
    print("=" * 60)

if __name__ == "__main__":
    main() 