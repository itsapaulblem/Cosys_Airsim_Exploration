# MNSTEVV PATH Troubleshooting Guide

If you get the error **"mnstevv.exe is not on Path"** or **"mnstevv command not found"**, here are the solutions:

## Understanding the Problem

When you run `pip install -e .` or the `install.bat` script, it installs the `mnstevv` command to Python's Scripts directory. However, this directory might not be in your system's PATH environment variable, making the command unavailable.

## Solution 1: Add Python Scripts to PATH (Recommended)

### Step 1: Find Your Python Scripts Directory
```cmd
# In Command Prompt, run these commands to find your Python paths:
where mnstevv
```

```powershell
# In Windows Powershell, run these commands to find your Python paths:
Get-Command mnstevv
```

Example output:
```
Executable directory: C:\Users\YourName\AppData\Local\Programs\Python\Python313\Scripts\mnstevv.exe
Script directory: C:\Users\YourName\AppData\Local\Programs\Python\Python313\Scripts\
```

### Step 2: Add Scripts Directory to PATH
1. Copy the Scripts directory path from Step 1
2. Open Windows Settings → Search for "Environment Variables"
3. Click "Edit the system environment variables" 
4. Click "Environment Variables..." button
5. In "User variables" section, find and select "Path", then click "Edit..."
6. Click "New" and paste your Scripts directory path
7. Click "OK" on all dialogs
8. **Restart your Command Prompt** (important!)

### Step 3: Test Installation
```cmd
# In a new Command Prompt window:
mnstevv --version
```

## Solution 2: Use Direct Python Execution (Quick Fix)

If you don't want to modify PATH, you can run MNSTEVV directly:

```cmd
# From the docker/cli directory:
python mnstevv.py --help
python mnstevv.py up --num_drones 3
```

## Solution 3: Use the Auto-Launcher Scripts (Easiest)

The auto-launcher scripts handle this automatically:

```cmd
# From docker/cli directory:
.\mnstevv-auto.bat --num_drones 3
```

This script will:
1. Check if `mnstevv` command is available
2. If not, fall back to `python mnstevv.py`
3. Automatically detect WSL2 IP and launch with correct settings

## Solution 4: Virtual Environment (Advanced)

If you're using a virtual environment, make sure it's activated:

```cmd
# Create and activate virtual environment
python -m venv venv
venv\Scripts\activate

# Install MNSTEVV in the virtual environment
pip install -e .

# Now mnstevv should be available (while venv is active)
mnstevv --version
```

## Verification Commands

After applying any solution, verify it works:

```cmd
# Test basic command availability
where mnstevv
mnstevv --version
mnstevv --help

# Test with WSL2 IP detection
cd docker\cli
.\mnstevv-auto.bat --dry-run --num_drones 2
```

## Common Issues and Fixes

### Issue: "Python is not recognized"
**Fix**: Install Python from python.org and ensure "Add to PATH" is checked during installation.

### Issue: "pip is not recognized"
**Fix**: Reinstall Python or run: `python -m pip install -e .`

### Issue: PATH changes don't take effect
**Fix**: Restart Command Prompt or run `refreshenv` (if you have Chocolatey).

### Issue: Multiple Python versions
**Fix**: Use the specific Python version: `python3.11 -m pip install -e .`

## Alternative: Use PowerShell Instead of CMD

PowerShell often has better PATH resolution:

```powershell
# In PowerShell:
pip install -e .
mnstevv --version

# Or use the auto-launcher:
.\mnstevv-auto.bat --num_drones 3
```

## Quick Test Script

Create a test file `test-mnstevv.bat`:

```cmd
@echo off
echo Testing MNSTEVV installation...
echo.

where mnstevv >nul 2>&1
if errorlevel 1 (
    echo ❌ mnstevv not found in PATH
    echo.
    python mnstevv.py --version >nul 2>&1
    if errorlevel 1 (
        echo ❌ mnstevv.py not found - run install.bat first
    ) else (
        echo ✅ mnstevv.py found - use: python mnstevv.py
    )
) else (
    echo ✅ mnstevv command found in PATH
    mnstevv --version
)
pause
```

This will diagnose your installation and provide specific guidance.