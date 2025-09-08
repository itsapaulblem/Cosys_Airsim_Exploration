# WSL2 + ROS2 + AirSim Troubleshooting Guide

## Quick Diagnostic Commands

Run these commands to quickly identify common issues:

```bash
# Check WSL version
wsl --version

# Check Ubuntu version
lsb_release -a

# Check ROS2 installation
ros2 --version

# Test AirSim connection
python3 ~/test_airsim_connection.py

# Check ROS2 environment
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
echo $DISPLAY

# List available ROS2 topics
ros2 topic list

# Check if GUI forwarding works
xclock
```

## Common Issues and Solutions

### 1. WSL2 Installation Issues

#### Problem: "WSL 2 requires an update to its kernel component"
```bash
# Solution: Download and install WSL2 kernel update
# Windows PowerShell (as Administrator):
# Download from: https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi
# Then run: wsl --set-default-version 2
```

#### Problem: "Please enable the Virtual Machine Platform Windows feature"
```powershell
# Windows PowerShell (as Administrator):
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
# Restart computer
```

#### Problem: WSL2 consuming too much memory
```bash
# Create/edit %USERPROFILE%\.wslconfig on Windows:
[wsl2]
memory=8GB
processors=4
swap=2GB
localhostForwarding=true
```

### 2. ROS2 Installation Issues

#### Problem: "Unable to locate package ros-humble-desktop"
```bash
# Solution: Fix repository setup
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Re-add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

#### Problem: "ROS_DISTRO was set to '...', but ROS2 is not running"
```bash
# Solution: Source ROS2 setup
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Problem: colcon build fails with "Permission denied"
```bash
# Solution: Fix workspace permissions
cd ~/airsim_ws
sudo chown -R $USER:$USER ~/airsim_ws
chmod -R 755 ~/airsim_ws
```

### 3. AirSim Connection Issues

#### Problem: "Connection refused" or "No connection could be made"
```bash
# Check 1: Test Windows host connectivity
ping $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')

# Check 2: Test AirSim port
nc -zv $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}') 41451

# Check 3: Windows Firewall
# Add Windows Firewall exception for port 41451
# Or temporarily disable firewall for testing
```

#### Problem: "ImportError: No module named 'airsim'"
```bash
# Solution: Install AirSim Python package
pip3 install airsim

# Alternative: Install from source
cd /mnt/l/Cosys-AirSim/PythonClient
pip3 install -e .
```

#### Problem: AirSim connects but no ROS2 topics appear
```bash
# Check if airsim_node is running
ros2 node list

# Launch with verbose output
ros2 launch airsim_ros_pkgs airsim_node.launch.py --ros-args --log-level debug

# Check for Python path issues
python3 -c "import airsim; print('AirSim import successful')"
```

### 4. GUI and RViz2 Issues

#### Problem: "cannot connect to X server"
```bash
# Solution 1: Check DISPLAY variable
echo $DISPLAY

# Solution 2: Set DISPLAY manually
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0

# Solution 3: Install and start VcXsrv on Windows
# Download from: https://sourceforge.net/projects/vcxsrv/
# Run XLaunch with default settings
```

#### Problem: RViz2 launches but shows blank screen
```bash
# Solution 1: Check OpenGL support
glxinfo | grep "direct rendering"

# Solution 2: Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
rviz2

# Solution 3: Install mesa-utils
sudo apt install mesa-utils

# Solution 4: Try different display settings
export LIBGL_ALWAYS_INDIRECT=1
rviz2
```

#### Problem: "Qt platform plugin could not be initialized"
```bash
# Solution: Install Qt dependencies
sudo apt install qt5-default qtbase5-dev

# Alternative: Use different Qt platform
export QT_QPA_PLATFORM=xcb
rviz2
```

### 5. Network and Performance Issues

#### Problem: Slow network performance between WSL2 and Windows
```bash
# Solution: Optimize network settings
# Add to /etc/sysctl.conf:
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

#### Problem: High CPU usage in WSL2
```bash
# Solution 1: Limit WSL2 resources in .wslconfig
# [wsl2]
# processors=4
# memory=8GB

# Solution 2: Check for runaway processes
htop

# Solution 3: Restart WSL2
# (In Windows PowerShell): wsl --shutdown && wsl
```

#### Problem: "DDS discovery issues" or topics not visible
```bash
# Solution: Configure DDS settings
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create FastDDS profile for better discovery
cat > ~/fastdds_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_participant">
        <rtps>
            <builtin>
                <discovery_config>
                    <simple_discovery_config>
                        <initial_announcements>
                            <count>10</count>
                            <period>100</period>
                        </initial_announcements>
                    </simple_discovery_config>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_profile.xml
```

### 6. Build and Compilation Issues

#### Problem: "CMake Error: Could not find a package configuration file"
```bash
# Solution: Install missing dependencies
cd ~/airsim_ws
rosdep install --from-paths src --ignore-src -r -y

# For specific packages:
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

#### Problem: "fatal error: 'airsim/...' file not found"
```bash
# Solution: Ensure AirSim headers are available
# Check if source is copied correctly
ls ~/airsim_ws/src/airsim_ros_pkgs/include/

# Rebuild from clean state
cd ~/airsim_ws
rm -rf build install log
colcon build
```

#### Problem: Python import errors during build
```bash
# Solution: Check Python environment
python3 --version
pip3 list | grep airsim

# Ensure correct Python path
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
```

## Diagnostic Scripts

### System Check Script
```bash
#!/bin/bash
# Create ~/diagnose_system.sh

echo "=== WSL2 + ROS2 + AirSim System Diagnosis ==="

echo "--- System Info ---"
echo "WSL Version: $(wsl.exe --version 2>/dev/null || echo 'WSL not accessible from Linux')"
echo "Ubuntu: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"

echo "--- ROS2 Info ---"
echo "ROS2 Version: $(ros2 --version 2>/dev/null || echo 'ROS2 not installed')"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

echo "--- Network Info ---"
echo "WSL IP: $(ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)"
echo "Windows Host IP: $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')"

echo "--- Python Environment ---"
echo "Python: $(python3 --version)"
echo "AirSim: $(python3 -c 'import airsim; print("Installed")' 2>/dev/null || echo 'Not installed')"

echo "--- GUI Support ---"
echo "DISPLAY: $DISPLAY"
echo "X11 Test: $(timeout 5 xclock 2>/dev/null && echo 'Working' || echo 'Failed')"

echo "--- AirSim Connection ---"
if python3 ~/test_airsim_connection.py >/dev/null 2>&1; then
    echo "AirSim: Connected"
else
    echo "AirSim: Not connected"
fi

echo "--- ROS2 Workspace ---"
if [ -d ~/airsim_ws ]; then
    echo "Workspace: Exists"
    if [ -f ~/airsim_ws/install/setup.bash ]; then
        echo "Build: Complete"
    else
        echo "Build: Not built"
    fi
else
    echo "Workspace: Not created"
fi
```

### Performance Monitor Script
```bash
#!/bin/bash
# Create ~/monitor_performance.sh

echo "=== Performance Monitoring ==="
echo "Press Ctrl+C to stop"

while true; do
    clear
    echo "=== $(date) ==="
    
    echo "--- Memory Usage ---"
    free -h
    
    echo "--- CPU Usage ---"
    top -bn1 | head -5
    
    echo "--- WSL2 Resource Usage ---"
    echo "WSL Memory: $(ps aux | awk '{sum+=$6} END {print sum/1024 " MB"}')"
    
    echo "--- Network ---"
    echo "Connections to AirSim:"
    netstat -an | grep 41451 || echo "No connections"
    
    echo "--- ROS2 Nodes ---"
    ros2 node list 2>/dev/null || echo "No ROS2 nodes"
    
    sleep 5
done
```

## Environment Reset Commands

If you need to completely reset your environment:

### Reset ROS2 Environment
```bash
# Remove ROS2 packages
sudo apt remove ros-humble-* -y
sudo apt autoremove -y

# Remove workspace
rm -rf ~/airsim_ws

# Clean environment variables from ~/.bashrc
sed -i '/ROS/d' ~/.bashrc
sed -i '/ros/d' ~/.bashrc
sed -i '/colcon/d' ~/.bashrc

# Re-run setup script
chmod +x /mnt/l/Cosys-AirSim/ros2/setup_wsl_ros2.sh
/mnt/l/Cosys-AirSim/ros2/setup_wsl_ros2.sh
```

### Reset WSL2 (Nuclear Option)
```powershell
# In Windows PowerShell (as Administrator):
wsl --shutdown
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```

## Getting Help

If you're still experiencing issues:

1. **Check logs**: Most errors provide detailed logs
2. **Search online**: ROS2 and WSL2 have active communities
3. **GitHub Issues**: Check AirSim and ROS2 repositories
4. **ROS Answers**: https://answers.ros.org/
5. **WSL GitHub**: https://github.com/microsoft/wsl

## Quick Recovery Commands

```bash
# Quick environment fix
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/airsim_ws/install/setup.bash

# Quick workspace rebuild
cd ~/airsim_ws && colcon build

# Quick AirSim test
python3 ~/test_airsim_connection.py

# Quick ROS2 launch
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

Remember: Most issues are environment-related. When in doubt, restart WSL2 with `wsl --shutdown` from Windows PowerShell, then restart WSL2.