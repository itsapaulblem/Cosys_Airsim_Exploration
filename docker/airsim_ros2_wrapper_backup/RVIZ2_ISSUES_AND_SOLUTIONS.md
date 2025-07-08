# RViz2 Issues and Solutions in Docker VNC Environment

## 🚨 Current Issue
RViz2 fails to start in the VNC environment due to **Ogre 3D rendering engine** incompatibility with the virtual display setup. The specific error is:

```
RenderingAPIException: Invalid parentWindowHandle (wrong server or screen) in GLXWindow::create
Unable to create glx visual
```

## 🔍 Root Cause Analysis

### 1. **OpenGL/GLX Visual Issues**
- VNC server (TightVNC) has limited GLX extension support
- Ogre 3D engine (used by RViz2) requires specific GLX visuals that aren't available
- Software rendering is working, but GLX visual creation fails

### 2. **ROS2 Launch Syntax**
- Fixed argument passing from `--ros-args -p use_sim_time:=false` to `use_sim_time:=false`

### 3. **Display Server Limitations**
- TightVNC doesn't support all X11 extensions needed for complex 3D rendering
- Virtual displays have different capabilities than physical displays

## ✅ Implemented Solutions

### 1. **Enhanced Fallback Launcher** (`launch_rviz_fallback.sh`)
The launcher now tries **5 different approaches** in sequence:

1. **Standard ROS2 Launch** - Fixed syntax issues
2. **Direct Binary Launch** - Bypass ROS2 launch system  
3. **Software-Only Rendering** - Force Mesa software rendering with custom Ogre config
4. **Minimal Configuration** - Simplified RViz2 config with basic panels
5. **Alternative Display** - Xvfb virtual display with enhanced GLX

### 2. **Alternative RQT Tools** (`launch_rqt_alternative.sh`)
When RViz2 fails, automatically starts working ROS2 GUI tools:

- **RQT Graph** - Visualize ROS2 node connections
- **RQT Topic** - Monitor topic data and types  
- **RQT Plot** - Real-time data plotting
- **RQT Console** - View log messages

### 3. **Comprehensive Graphics Fixes**
Applied multiple layers of graphics compatibility:

```bash
# Software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=1
export GALLIUM_DRIVER=llvmpipe

# Ogre-specific fixes
export OGRE_RTT_MODE=Copy
export OGRE_CONFIG_DIALOG=false
export __GLX_VENDOR_LIBRARY_NAME=mesa

# Qt/X11 compatibility
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
```

## 🎯 Current Working Solution

### **Use RQT Tools Instead of RViz2**

**Windows Command:**
```cmd
docker\airsim_ros2_wrapper\run_rqt_alternative.bat
```

**Container Command:**
```bash
/launch_rqt_alternative.sh
```

### **Access via VNC Web Browser:**
- URL: `http://localhost:6901/vnc.html`
- Password: `airsim123`

## 📊 What You Can Do with RQT Tools

### 1. **Monitor AirSim Data in Real-Time**
```bash
# In RQT Plot, add these topics:
/airsim_node/drone_1/odom/pose/pose/position/x
/airsim_node/drone_1/odom/pose/pose/position/y
/airsim_node/drone_1/odom/pose/pose/position/z
```

### 2. **View Network Topology**
- RQT Graph shows all ROS2 nodes and their connections
- See data flow from AirSim → ROS2 wrapper → your applications

### 3. **Terminal Monitoring**
```bash
# View drone position
ros2 topic echo /airsim_node/drone_1/odom

# View IMU data  
ros2 topic echo /airsim_node/drone_1/imu

# List all available topics
ros2 topic list | grep airsim_node
```

## 🔄 Alternative Approaches (Future)

### 1. **X11 Forwarding** (Linux Host)
If running on Linux, can use native X11 forwarding instead of VNC:
```bash
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
```

### 2. **NVIDIA Container Runtime** (GPU Host)
For hardware-accelerated rendering:
```bash
docker run --gpus all --runtime=nvidia ...
```

### 3. **NoMachine or X2Go** (Alternative Remote Desktop)
Better 3D acceleration support than VNC.

## 🎮 Quick Start Guide

### **Method 1: Use Alternative RQT Tools (Recommended)**
```cmd
# Windows
docker\airsim_ros2_wrapper\run_rqt_alternative.bat

# Access GUI: http://localhost:6901/vnc.html
```

### **Method 2: Try RViz2 Fallback**
```cmd
# Windows  
docker\airsim_ros2_wrapper\run_vnc.bat

# Inside VNC, run in terminal:
/launch_rviz_fallback.sh
```

### **Method 3: Terminal-Only Monitoring**
```cmd
# Windows
docker\airsim_ros2_wrapper\run_headless.bat

# Then use:
ros2 topic echo /airsim_node/drone_1/odom
ros2 topic list
ros2 node list
```

## 🏆 Performance Comparison

| Tool | 3D Visualization | Real-time Data | Network View | Startup Time | VNC Compatible |
|------|------------------|----------------|--------------|--------------|----------------|
| RViz2 | ✅ Excellent | ✅ | ❌ | Slow | ❌ Fails |
| RQT Graph | ❌ | ❌ | ✅ Excellent | Fast | ✅ Works |
| RQT Plot | ❌ | ✅ Excellent | ❌ | Fast | ✅ Works |
| RQT Topic | ❌ | ✅ Good | ❌ | Fast | ✅ Works |
| Terminal | ❌ | ✅ Good | ❌ | Instant | ✅ Works |

## 🔧 Troubleshooting

### **If RQT Tools Don't Start:**
```bash
# Check X11 connection
xset q

# Check display variable
echo $DISPLAY

# Manual start
export DISPLAY=:1
rqt_graph &
```

### **If VNC Doesn't Work:**
```bash
# Check VNC server
vncserver -list

# Restart VNC
vncserver -kill :1
vncserver :1 -geometry 1920x1080
```

### **If AirSim Connection Fails:**
```bash
# Test Python connection
python3 -c "import airsim; client = airsim.MultirotorClient(); print('Connected:', client.ping())"

# Check ROS2 node
ros2 node list | grep airsim
```

## 📚 Summary

The **RQT Tools approach** provides a **working, practical solution** for monitoring AirSim data in a Docker VNC environment. While it doesn't have the 3D visualization of RViz2, it offers:

- ✅ **Reliable startup** in VNC environments
- ✅ **Real-time data monitoring** and plotting  
- ✅ **Network topology visualization**
- ✅ **Fast performance** with low overhead
- ✅ **Full AirSim integration** with all topics/services

This solution lets you **immediately start working** with AirSim ROS2 data while the complex 3D rendering issues are addressed in future updates. 