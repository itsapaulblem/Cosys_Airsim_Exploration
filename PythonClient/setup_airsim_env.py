import os
import subprocess
import sys
import venv
from pathlib import Path

def create_venv(venv_path):
    print(f"Creating virtual environment at {venv_path}")
    venv.create(venv_path, with_pip=True)
    
    # Get the path to the Python executable in the virtual environment
    if sys.platform == "win32":
        python_path = venv_path / "Scripts" / "python.exe"
        pip_path = venv_path / "Scripts" / "pip.exe"
    else:
        python_path = venv_path / "bin" / "python"
        pip_path = venv_path / "bin" / "pip"

    return python_path, pip_path

def install_packages(pip_path):
    print("Installing required packages...")
    
    # Install build dependencies first
    print("Installing build dependencies...")
    subprocess.run([str(pip_path), "install", "--no-cache-dir", "setuptools", "wheel"], check=True)
    
    # Install numpy first (required by airsim build process)
    print("Installing numpy first...")
    subprocess.run([str(pip_path), "install", "--no-cache-dir", "numpy"], check=True)
    
    # Install tornado with compatible version for both AirSim and Streamlit
    print("Installing compatible tornado version...")
    subprocess.run([str(pip_path), "install", "--no-cache-dir", "tornado>=6.2,<7.0"], check=True)
    
    # Install AirSim core packages with compatible versions
    airsim_packages = [
        "msgpack-rpc-python",
        "opencv-python",
        "matplotlib"
    ]
    
    for package in airsim_packages:
        print(f"Installing {package}...")
        subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)
    
    # Install airsim with --no-build-isolation so it can access numpy
    print("Installing airsim...")
    subprocess.run([str(pip_path), "install", "--no-cache-dir", "--no-build-isolation", "airsim"], check=True)
    
    # Install Streamlit Mission Control dependencies
    print("\n Installing Mission Control Interface dependencies...")
    streamlit_packages = [
        "streamlit>=1.28.0",
        "pandas>=1.5.0",
        "plotly>=5.0.0",
        "Pillow>=8.0.0"
    ]
    
    for package in streamlit_packages:
        print(f"Installing {package}...")
        subprocess.run([str(pip_path), "install", "--no-cache-dir", package], check=True)

def main():
    # Create a directory for the virtual environment
    venv_path = Path("airsim_env")
    
    if venv_path.exists():
        print(f"Virtual environment already exists at {venv_path}")
        response = input("Do you want to recreate it? (y/n): ")
        if response.lower() == 'y':
            import shutil
            shutil.rmtree(venv_path)
        else:
            print("Using existing virtual environment")
            return

    python_path, pip_path = create_venv(venv_path)
    install_packages(pip_path)
    
    print("\n" + "="*70)
    print(" SETUP COMPLETE! AirSim + Mission Control Interface Ready")
    print("="*70)
    print("\n To get started:")
    print("\n1. Activate the virtual environment:")
    if sys.platform == "win32":
        print(f"   {venv_path}\\Scripts\\activate")
    else:
        print(f"   source {venv_path}/bin/activate")
    
    print("\n2. Test AirSim connection:")
    print("   python multirotor/hello_drone.py")
    
    print("\n3. Launch Mission Control Interface:")
    print("   streamlit run streamlit_mission_control.py")
    print("   (or double-click launch_mission_control.bat on Windows)")
    
    print("\n4. Access the web interface:")
    print("    http://localhost:8501")
    
    print("\n Available mission types:")
    print("   • Box Orbital Mission")
    print("   • Spiral Search Mission") 
    print("   • Grid Survey Mission")
    print("   • Figure-8 Mission")
    print("   • Star Pattern Mission")
    print("   • Wave Mission")
    
    print("\n Happy flying! ")
    print("="*70)

if __name__ == "__main__":
    main() 