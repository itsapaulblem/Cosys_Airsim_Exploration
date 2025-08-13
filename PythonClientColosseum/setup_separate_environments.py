#!/usr/bin/env python3
"""
Setup Separate Environments for AirSim and Streamlit

This script creates two separate virtual environments to avoid 
the tornado dependency conflict between msgpack-rpc-python and streamlit.

Environment 1: airsim_env (tornado 4.5.3)
Environment 2: streamlit_env (tornado 6.1+)
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

def install_airsim_packages(pip_path):
    """Install AirSim packages with tornado 4.5.3"""
    print("\n🚁 Installing AirSim Environment Packages...")
    
    packages = [
        # Core dependencies
        "setuptools", "wheel", "numpy",
        
        # Specific tornado version for msgpack-rpc-python
        "tornado==4.5.3",
        "backports.ssl_match_hostname",
        
        # AirSim dependencies
        "msgpack-rpc-python",
        "opencv-python",
        "matplotlib",
        
        # HTTP requests (for API communication)
        "requests"
    ]
    
    for package in packages:
        print(f"Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Warning: Failed to install {package}: {e}")
    
    # Install airsim last with special handling
    print("Installing airsim...")
    try:
        subprocess.run([str(pip_path), "install", "--no-cache-dir", "--no-build-isolation", "airsim"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"⚠️  Warning: AirSim installation issue: {e}")

def install_streamlit_packages(pip_path):
    """Install Streamlit packages with modern tornado"""
    print("\n🌐 Installing Streamlit Environment Packages...")
    
    packages = [
        # Core dependencies
        "setuptools", "wheel",
        
        # Modern tornado for Streamlit
        "tornado>=6.1",
        
        # Streamlit and UI dependencies
        "streamlit>=1.28.0",
        "pandas>=1.5.0",
        "plotly>=5.0.0",
        "Pillow>=8.0.0",
        "matplotlib>=3.5.0",
        
        # HTTP requests (for API communication)
        "requests>=2.28.0"
    ]
    
    for package in packages:
        print(f"Installing {package}...")
        try:
            subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Warning: Failed to install {package}: {e}")

def create_launcher_scripts():
    """Create launcher scripts for both environments"""
    print("\n📝 Creating launcher scripts...")
    
    # API Server launcher (no emojis for Windows compatibility)
    api_launcher = """@echo off
echo Starting Mission API Server...
cd /d "%~dp0"
airsim_env\\Scripts\\activate.bat
python mission_api_server.py
pause
"""
    
    with open("start_api_server.bat", "w", encoding='utf-8') as f:
        f.write(api_launcher)
    
    # Streamlit launcher  
    streamlit_launcher = """@echo off
echo Starting Streamlit Mission Control...
cd /d "%~dp0"
streamlit_env\\Scripts\\activate.bat
streamlit run streamlit_mission_control.py --theme.base dark --theme.primaryColor #4AFF4A
pause
"""
    
    with open("start_streamlit_ui.bat", "w", encoding='utf-8') as f:
        f.write(streamlit_launcher)
    
    # Combined launcher
    combined_launcher = """@echo off
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
"""
    
    with open("launch_mission_control.bat", "w", encoding='utf-8') as f:
        f.write(combined_launcher)
    
    print("✅ Created launcher scripts:")
    print("   • start_api_server.bat")
    print("   • start_streamlit_ui.bat") 
    print("   • launch_mission_control.bat")

def main():
    print("🚁 AirSim + Streamlit Separate Environment Setup")
    print("=" * 60)
    print("This creates two separate environments to avoid tornado conflicts:")
    print("  • airsim_env: AirSim with tornado 4.5.3")
    print("  • streamlit_env: Streamlit with tornado 6.1+")
    print("=" * 60)
    
    # Create AirSim environment
    airsim_env_path = Path("airsim_env")
    python_airsim, pip_airsim = create_venv(airsim_env_path, "AirSim Environment")
    
    if python_airsim is None:
        print("❌ Failed to create AirSim environment. Exiting.")
        return
    
    install_airsim_packages(pip_airsim)
    
    # Create Streamlit environment  
    streamlit_env_path = Path("streamlit_env")
    python_streamlit, pip_streamlit = create_venv(streamlit_env_path, "Streamlit Environment")
    
    if python_streamlit is None:
        print("❌ Failed to create Streamlit environment. Exiting.")
        return
        
    install_streamlit_packages(pip_streamlit)
    
    # Create launcher scripts
    create_launcher_scripts()
    
    print("\n" + "=" * 70)
    print("🎉 SETUP COMPLETE! Two Separate Environments Created")
    print("=" * 70)
    print("\n🚀 Quick Start:")
    print("   Double-click: launch_mission_control.bat")
    print("\n🔧 Manual Start:")
    print("   1. Double-click: start_api_server.bat")
    print("   2. Double-click: start_streamlit_ui.bat")
    print("\n📍 Access URLs:")
    print("   📡 API Server: http://localhost:8000")
    print("   🌐 Mission Control: http://localhost:8501")
    print("\n✅ No more dependency conflicts!")
    print("=" * 70)

if __name__ == "__main__":
    main() 