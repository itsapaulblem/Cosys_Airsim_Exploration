#!/usr/bin/env python3
"""
Quick demo launcher for Streamlit Mission Control Interface

This script provides a simple way to launch the Streamlit mission control
interface with proper environment setup.
"""

import subprocess
import sys
import os
from pathlib import Path

def check_streamlit_installed():
    """Check if Streamlit is installed"""
    try:
        import streamlit
        return True
    except ImportError:
        return False

def install_streamlit_deps():
    """Install Streamlit and dependencies"""
    print("Installing Streamlit dependencies...")
    subprocess.check_call([
        sys.executable, "-m", "pip", "install",
        "streamlit>=1.28.0",
        "pandas>=1.5.0", 
        "plotly>=5.0.0",
        "matplotlib>=3.5.0"
    ])

def launch_streamlit():
    """Launch the Streamlit mission control interface"""
    script_path = Path(__file__).parent / "streamlit_mission_control.py"
    
    if not script_path.exists():
        print(f"Error: {script_path} not found!")
        return False
    
    print(" Launching Drone Mission Control Center...")
    print(" Interface will be available at: http://localhost:8501")
    print(" Use Ctrl+C to stop the server")
    print("-" * 60)
    
    try:
        subprocess.run([
            sys.executable, "-m", "streamlit", "run", 
            str(script_path),
            "--theme.base", "dark",
            "--theme.primaryColor", "#4AFF4A"
        ])
    except KeyboardInterrupt:
        print("\n Mission Control Center stopped.")
        return True
    except Exception as e:
        print(f" Error launching Streamlit: {e}")
        return False

def main():
    print(" Drone Mission Control Center - Demo Launcher")
    print("=" * 60)
    
    # Check if Streamlit is installed
    if not check_streamlit_installed():
        print("  Streamlit not found. Installing dependencies...")
        try:
            install_streamlit_deps()
            print(" Dependencies installed successfully!")
        except Exception as e:
            print(f" Failed to install dependencies: {e}")
            print(" Please run manually: pip install streamlit pandas plotly matplotlib")
            return
    
    # Launch Streamlit
    print(" Starting Mission Control Interface...")
    launch_streamlit()

if __name__ == "__main__":
    main() 