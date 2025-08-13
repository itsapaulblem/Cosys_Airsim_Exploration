# Windows Setup Guide for AirSim + PX4 Docker Solution

This guide shows how to set up the complete AirSim + PX4 Docker solution on Windows with X11 forwarding for GUI display.

## 🎯 Overview

Since Docker containers on Windows don't have direct access to the Windows display, we need an X11 server to forward the GUI applications from the containers to your Windows desktop.

## 📋 Prerequisites

1. **Docker Desktop** installed and running
2. **X11 Server** (VcXsrv recommended)
3. **GitHub Container Registry Access**

## 🚀 Step-by-Step Setup

### Step 1: Install Docker Desktop

1. Download [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Install and start Docker Desktop
3. Ensure "Use Linux containers" is selected
4. Test: `docker run hello-world`

### Step 2: Install VcXsrv (X11 Server)

**Option A: Direct Download**
1. Download [VcXsrv](https://sourceforge.net/projects/vcxsrv/) from SourceForge
2. Install VcXsrv with default settings

**Option B: Using Chocolatey**
```powershell
# Install Chocolatey if not already installed
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# Install VcXsrv
choco install vcxsrv
```

**Option C: Using winget**
```powershell
winget install --id=marha.VcXsrv -e
```

### Step 3: Configure VcXsrv

**IMPORTANT**: VcXsrv must be configured correctly for Docker containers to connect.

1. **Start VcXsrv** from Start Menu
2. **XLaunch Configuration**:
   - **Display settings**: Multiple windows, Display number: 0
   - **Client startup**: Start no client
   - **Extra settings**: 
     - ✅ **Disable access control** (CRITICAL!)
     - ✅ **Clipboard** (optional)
     - ❌ **Native opengl** (uncheck this)
     - ❌ **Primary Selection** (uncheck this)

3. **Save configuration**: Save as `AirSim.xlaunch` for future use

4. **Verify VcXsrv is running**:
   ```powershell
   netstat -an | findstr ":6000"
   # Should show: TCP 0.0.0.0:6000 LISTENING
   ```

### Step 4: Authenticate with GitHub Container Registry

```powershell
docker login ghcr.io
# Enter GitHub username and Personal Access Token
```

> **Note**: You need a GitHub Personal Access Token with `read:packages` permission.

### Step 5: Download and Setup

1. **Navigate to the docker directory**:
   ```powershell
   cd L:\Cosys-AirSim\docker
   ```

2. **Run the Windows setup script**:
   ```powershell
   .\run_full_stack_windows.ps1
   ```

## 🖥️ Alternative X11 Servers

### X410 (Microsoft Store)
- **Pros**: Easy setup, good performance
- **Cons**: Paid software (~$10)
- **Setup**: Install from Microsoft Store, enable "Allow connections from any computer"

### MobaXterm
- **Pros**: Free, includes many tools
- **Cons**: Heavier installation
- **Setup**: Download MobaXterm, start local terminal (X11 server auto-starts)

### WSL2 with WSLg (Windows 11)
- **Pros**: Built into Windows 11
- **Cons**: Only on Windows 11, more complex setup
- **Setup**: Enable WSLg, may conflict with VcXsrv

## 🔧 Manual Setup (Alternative)

If the PowerShell script doesn't work, here's the manual approach:

### 1. Set Environment Variables
```powershell
# Get your Windows IP
$windowsIP = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.InterfaceAlias -notlike "*Loopback*"} | Select-Object -First 1).IPAddress
$env:DISPLAY = "${windowsIP}:0.0"
```

### 2. Download Blocks Environment (if WSL available)
```powershell
wsl bash ./download_blocks_env_binary.sh
```

### 3. Build and Start
```powershell
docker-compose -f docker-compose-full-stack.yml build
docker-compose -f docker-compose-full-stack.yml --profile full-stack up -d
```

### 4. Start AirSim
```powershell
docker exec -d airsim-blocks /home/airsim_user/blocks_env/LinuxBlocks/Linux/Blocks.sh -windowed -ResX=1280 -ResY=720
```

## 🐛 Troubleshooting

### VcXsrv Issues

**Problem**: "Cannot connect to display"
- **Solution**: Ensure VcXsrv is running and "Disable access control" is checked

**Problem**: X11 server not listening on port 6000
- **Solution**: Restart VcXsrv, check Windows Firewall settings

**Problem**: Black screen or no window
- **Solution**: Try unchecking "Native opengl" in VcXsrv settings

### Docker Issues

**Problem**: "Access denied" from GitHub Container Registry
```powershell
docker login ghcr.io
# Use GitHub username and Personal Access Token (not password)
```

**Problem**: Docker not running
- **Solution**: Start Docker Desktop, ensure Linux containers are enabled

**Problem**: Long build times
- **Solution**: Ensure good internet connection, GitHub Container Registry access

### Network Issues

**Problem**: Containers can't reach Windows host
```powershell
# Test from container
docker run --rm alpine ping host.docker.internal
```

**Problem**: Wrong IP address detection
```powershell
# Manually set the IP
$env:DISPLAY = "YOUR_WINDOWS_IP:0.0"
```

## 📱 Usage Commands

```powershell
# Start the full stack
.\run_full_stack_windows.ps1 start

# View logs
.\run_full_stack_windows.ps1 logs

# Check status
.\run_full_stack_windows.ps1 status

# Stop everything
.\run_full_stack_windows.ps1 stop
```

## 🎮 Testing Your Setup

1. **Start VcXsrv** with correct settings
2. **Run the PowerShell script**: `.\run_full_stack_windows.ps1`
3. **Look for AirSim window** in VcXsrv
4. **Test GPS functionality**:
   ```powershell
   cd ..\PythonClient\multirotor
   python -c "import setup_path; import cosysairsim as airsim; import time; client = airsim.MultirotorClient(); client.confirmConnection(); time.sleep(5); home = client.getHomeGeoPoint(); print(f'GPS Home: ({home.latitude}, {home.longitude})')"
   ```

## 🎯 Expected Results

When everything is working correctly:

1. **VcXsrv shows AirSim Blocks environment**
2. **PX4 console is accessible**: `docker exec -it px4-full-stack /Scripts/px4_shell.sh`
3. **GPS home location is set** within 10-30 seconds
4. **Drone can be armed** via Python API

## 🌟 Performance Tips

1. **Allocate more resources to Docker Desktop**:
   - Settings → Resources → Advanced
   - Increase Memory to 8GB+
   - Increase CPUs to 4+

2. **Use SSD storage** for Docker containers

3. **Close other heavy applications** during testing

4. **Use windowed mode** for better performance:
   ```bash
   -windowed -ResX=1280 -ResY=720
   ```

## 🔄 Starting/Stopping

### Quick Start (after initial setup)
```powershell
# Start VcXsrv (if not already running)
# Then:
.\run_full_stack_windows.ps1 start
```

### Clean Shutdown
```powershell
.\run_full_stack_windows.ps1 stop
# Close VcXsrv if desired
```

---

## 📞 Getting Help

If you encounter issues:

1. **Check the troubleshooting section above**
2. **Verify VcXsrv configuration** (most common issue)
3. **Check Docker logs**: `docker-compose logs`
4. **Test X11 forwarding**: Try running a simple GUI app like `xeyes`

For persistent issues, provide:
- Windows version
- Docker Desktop version  
- VcXsrv version
- Complete error messages
- Output of `docker-compose ps` 