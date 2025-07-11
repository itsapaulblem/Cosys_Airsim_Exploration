#!/usr/bin/env python3
"""
Setup Unified Environment for Cosys AirSim and Streamlit

This script creates a single virtual environment that contains both
Cosys AirSim and Streamlit, since cosys airsim is compatible with tornado 6.1+.

Environment: cosys_mission_env (tornado 6.1+)
"""

import os
import subprocess
import sys
import venv
from pathlib import Path
import shutil

def create_venv(venv_path, description):
    """Create a virtual environment"""
    print(f"\n📦 Creating {description} at {venv_path}")
    
    if venv_path.exists():
        print(f"Environment already exists. Attempting to remove...")
        try:
            # Try to remove the existing environment
            shutil.rmtree(venv_path)
            print("✅ Successfully removed existing environment")
        except PermissionError as e:
            print(f"⚠️  Cannot remove existing environment: {e}")
            print("This usually happens if the environment is currently active.")
            print("\n🔧 Please do one of the following:")
            print(f"   1. Close any terminals with '{venv_path.name}' activated")
            print(f"   2. Manually delete the '{venv_path}' folder")
            print(f"   3. Rename the folder and continue")
            
            response = input(f"Continue anyway and create '{venv_path}_new'? (y/n): ")
            if response.lower() == 'y':
                venv_path = venv_path.parent / f"{venv_path.name}_new"
                print(f"Creating environment at: {venv_path}")
            else:
                print("Setup cancelled. Please resolve the permission issue and try again.")
                return None, None
        except Exception as e:
            print(f"❌ Unexpected error removing environment: {e}")
            return None, None
    
    try:
        venv.create(venv_path, with_pip=True)
        print(f"✅ Created virtual environment: {venv_path}")
    except Exception as e:
        print(f"❌ Failed to create virtual environment: {e}")
        return None, None
    
    # Get platform-specific paths
    if sys.platform == "win32":
        python_path = venv_path / "Scripts" / "python.exe"
        pip_path = venv_path / "Scripts" / "pip.exe"
    else:
        python_path = venv_path / "bin" / "python"
        pip_path = venv_path / "bin" / "pip"
    
    return python_path, pip_path

def install_unified_packages(pip_path):
    """Install both Cosys AirSim and Streamlit packages in one environment"""
    print("\n🚁🌐 Installing Unified Mission Control Packages...")
    
    # Install build dependencies first
    print("Installing build dependencies...")
    build_packages = ["setuptools", "wheel", "numpy"]
    for package in build_packages:
        print(f"Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Warning: Failed to install {package}: {e}")
    
    # Install tornado 6.1+ (compatible with both cosys airsim and streamlit)
    print("Installing modern tornado (6.1+)...")
    try:
        subprocess.run([str(pip_path), "install", "--no-cache-dir", "tornado>=6.1"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"⚠️  Warning: Failed to install tornado: {e}")
    
    # Install Cosys AirSim dependencies
    print("\n🚁 Installing Cosys AirSim packages...")
    airsim_packages = [
        "rpc-msgpack",
        "opencv-python",
        "matplotlib",
        "requests"
    ]
    
    for package in airsim_packages:
        print(f"Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Warning: Failed to install {package}: {e}")
    
    # Install airsim with special handling
    # print("Installing airsim...")
    # try:
    #     subprocess.run([str(pip_path), "install", "--no-cache-dir", "--no-build-isolation", "airsim"], check=True)
    # except subprocess.CalledProcessError as e:
    #     print(f"⚠️  Warning: AirSim installation issue: {e}")
    
    # Install Streamlit and UI dependencies
    print("\n🌐 Installing Streamlit Mission Control packages...")
    streamlit_packages = [
        "streamlit>=1.28.0",
        "pandas>=1.5.0",
        "plotly>=5.0.0",
        "Pillow>=8.0.0"
    ]
    
    for package in streamlit_packages:
        print(f"Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Warning: Failed to install {package}: {e}")

def create_launcher_scripts():
    """Create launcher scripts for the unified environment"""
    print("\n📝 Creating launcher scripts...")
    
    # API Server launcher (no emojis for Windows compatibility)
    api_launcher = """@echo off
echo Starting Mission API Server...
cd /d "%~dp0"
cosys_mission_env\\Scripts\\activate.bat
python mission_api_server.py
pause
"""
    
    with open("start_api_server.bat", "w", encoding='utf-8') as f:
        f.write(api_launcher)
    
    # Streamlit launcher  
    streamlit_launcher = """@echo off
echo Starting Streamlit Mission Control...
cd /d "%~dp0"
cosys_mission_env\\Scripts\\activate.bat
streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A
pause
"""
    
    with open("start_streamlit_ui.bat", "w", encoding='utf-8') as f:
        f.write(streamlit_launcher)
    
    # Combined launcher
    combined_launcher = """@echo off
echo ===============================================================
echo   Cosys AirSim Mission Control System
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
"""
    
    with open("launch_mission_control.bat", "w", encoding='utf-8') as f:
        f.write(combined_launcher)
    
    # Environment activation helper
    activate_helper = """@echo off
echo ===============================================================
echo   Activating Cosys Mission Control Environment
echo ===============================================================
echo.
cosys_mission_env\\Scripts\\activate.bat
"""
    
    with open("activate_env.bat", "w", encoding='utf-8') as f:
        f.write(activate_helper)
    
    print("✅ Created launcher scripts:")
    print("   • start_api_server.bat")
    print("   • start_streamlit_ui.bat") 
    print("   • launch_mission_control.bat")
    print("   • activate_env.bat")

def main():
    print("🚁 Cosys AirSim + Streamlit Unified Environment Setup")
    print("=" * 65)
    print("Creating a single environment with both Cosys AirSim and Streamlit")
    print("since cosys airsim is compatible with tornado 6.1+")
    print("=" * 65)
    
    # Create unified environment
    env_path = Path("cosys_mission_env")
    python_path, pip_path = create_venv(env_path, "Cosys Mission Control Environment")
    
    if python_path is None:
        print("❌ Failed to create environment. Exiting.")
        return
    
    install_unified_packages(pip_path)
    
    # Create launcher scripts
    create_launcher_scripts()
    
    print("\n" + "=" * 70)
    print("🎉 SETUP COMPLETE! Unified Environment Created")
    print("=" * 70)
    print("\n🚀 Quick Start:")
    print("   Double-click: launch_mission_control.bat")
    print("\n🔧 Manual Start:")
    print("   1. Double-click: start_api_server.bat")
    print("   2. Double-click: start_streamlit_ui.bat")
    print("\n⚡ Environment Management:")
    print("   • Activate environment: activate_env.bat")
    print("\n📍 Access URLs:")
    print("   📡 API Server: http://localhost:8000")
    print("   🌐 Mission Control: http://localhost:8501")
    print("\n✅ Single environment - no dependency conflicts!")
    print("   Environment: cosys_mission_env")
    print("   Packages: Cosys AirSim + Streamlit + tornado 6.1+")
    print("=" * 70)

if __name__ == "__main__":
    main() 