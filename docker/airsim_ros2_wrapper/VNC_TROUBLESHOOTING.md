# VNC Troubleshooting Guide

## 🟡 Normal Warnings (Ignore These!)

### X11 Security Extension Warning
```
xauth: (argv):1:  couldn't query Security extension on display ":1"
```
**Status: ✅ NORMAL** - This is expected in VNC environments. TightVNC doesn't support all X11 extensions, but this doesn't affect functionality.

### Keyboard Extension Warning  
```
xkeyboard extension is not present
```
**Status: ✅ NORMAL** - VNC handles keyboard input directly. Basic keyboard functionality works fine without the XKB extension.

### Font Path Warning
```
Please set correct fontPath in the vncserver script.
Couldn't start Xtightvnc; trying default font path.
```
**Status: ✅ NORMAL** - VNC automatically falls back to default fonts. This warning can be ignored.

---

## 🐛 Fixed Issues: VNC Server Startup Failures

The following issues have been identified and resolved in the updated containers:

### ❌ **Problem 1: Unsupported VNC Extensions**
```
Unrecognized option: +extension
```

**Cause**: TightVNC doesn't support the `+extension GLX` flags
**Solution**: ✅ **FIXED** - Removed unsupported extension flags from startup script

### ❌ **Problem 2: Missing .Xauthority File**
```
xauth: file /home/vncuser/.Xauthority does not exist
```

**Cause**: X11 authorization file not created
**Solution**: ✅ **FIXED** - Added creation of .Xauthority file in startup script

### ❌ **Problem 3: Font Path Issues**
```
Please set correct fontPath in the vncserver script.
```

**Cause**: TightVNC can't find system fonts
**Solution**: ✅ **FIXED** - Using default font path (removed custom font settings)

## 🔧 **Quick Fixes**

### **Option 1: Use the Fixed Container (Recommended)**

1. **Rebuild the container with fixes**:
   ```bash
   ./build_vnc.bat
   ```

2. **Test with minimal VNC setup**:
   ```bash
   ./run_vnc_minimal.bat
   ```

3. **Access VNC**:
   - **Web Browser**: `http://localhost:6901/vnc.html`
   - **VNC Client**: `localhost:5901`
   - **Password**: `airsim123`

### **Option 2: Manual VNC Testing**

Use the troubleshooting script to test step-by-step:
```bash
./troubleshoot_vnc.bat
```

This will:
- Build the updated container
- Test VNC components individually
- Show detailed error messages
- Keep container running for manual testing

### **Option 3: Alternative VNC Server**

If TightVNC continues to have issues, we can switch to X11VNC or TigerVNC:

```dockerfile
# Replace tightvncserver with x11vnc
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*
```

## 🎯 **Immediate Workarounds**

### **Workaround 1: Use Simple Container + External VNC**

If VNC continues to fail, you can use the simple container and connect external tools:

```bash
# Start simple container
./run_simple.bat

# Use external tools for visualization:
# - QGroundControl for mission planning
# - RViz2 on host machine (if ROS2 installed)
# - Web-based monitoring tools
```

### **Workaround 2: Use Development Container**

Connect directly to the container for development:

```bash
# Start container
./run_simple.bat

# In another terminal, connect for development
docker exec -it airsim_ros2_container bash

# Now you have full access to ROS2 environment
ros2 topic list
ros2 launch airsim_ros_pkgs rviz.launch.py
```

## 📊 **VNC Status Check Commands**

Once VNC is running, use these commands to verify:

```bash
# Check VNC processes
ps aux | grep vnc

# Check display
echo $DISPLAY
xdpyinfo -display :1

# Test X11 forwarding
xclock -display :1

# Check VNC log
cat /home/vncuser/.vnc/*.log
```

## 🚀 **Expected Success Output**

When VNC works correctly, you should see:
```
VNC Server started on :1 (port 5901)
Password: airsim123
noVNC started on port 6901
Web Access: http://localhost:6901/vnc.html
AirSim ROS2 Node: Running
VNC Desktop: Running on :1
```

## 🔄 **Recovery Steps**

If VNC gets stuck:

1. **Kill VNC processes**:
   ```bash
   docker exec -it container_name bash -c "pkill -f vnc"
   ```

2. **Clean VNC files**:
   ```bash
   docker exec -it container_name bash -c "rm -rf /home/vncuser/.vnc/* /tmp/.X*"
   ```

3. **Restart container**:
   ```bash
   docker restart container_name
   ```

## 📞 **Support Commands**

Use these for additional debugging:

```bash
# Check container logs
docker logs airsim-ros2-vnc

# Interactive troubleshooting
docker exec -it airsim-ros2-vnc bash

# Port checking
netstat -tulpn | grep -E "(5901|6901)"
```

---

**The VNC issues have been identified and fixed in the updated Dockerfile. Use the new scripts to test the corrected version!** 🎉 