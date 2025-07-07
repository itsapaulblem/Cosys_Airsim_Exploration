# PX4-AirSim Docker Quick Start Guide

## Issues Fixed
1. **Multi-drone spawning**: Now all 5 drones start with "multi" profile
2. **AirSim connection**: Provided proper settings files and troubleshooting tools
3. **🔧 NEW: Auto-simulator binding**: Fixed PX4 connecting to itself instead of waiting for AirSim

## The "Simulator Connected" Problem - SOLVED!

**Problem**: PX4 reported "Simulator connected on TCP port 4561" even when AirSim wasn't running.

**Root Cause**: PX4 was auto-starting internal `simulator_mavlink` module which bound to port 4561 immediately.

**Solution**: 
- Set `PX4_SIMULATOR=none` to disable auto-start
- Set `PX4_SIM_MODEL=none_iris` for proper external simulator mode
- Modified startup script to properly wait for external connections

## Quick Start Options

### Option 1: Single Drone (Recommended for testing)
```powershell
# 1. Configure AirSim settings
.\copy_airsim_settings.bat single

# 2. Start single PX4 container
.\launch_with_gps_fix.bat single

# 3. Launch AirSim (Unreal Engine)
# 4. Check connection in AirSim console
```

### Option 2: Multiple Drones
```powershell
# 1. Configure AirSim settings
.\copy_airsim_settings.bat multi

# 2. Start all PX4 containers
.\launch_with_gps_fix.bat multi

# 3. Launch AirSim (Unreal Engine)
# 4. Check connections for all 5 drones
```

## Testing the Fix

### Verify External Simulator Mode Works:
```powershell
# Test that PX4 waits for external simulator
.\test_external_simulator_fix.bat

# Monitor connection in real-time
.\monitor_connection.bat
```

**Expected Behavior**:
- ✅ **BEFORE AirSim starts**: No "Simulator connected" message
- ✅ **AFTER AirSim starts**: "Simulator connected" appears in logs
- ❌ **If broken**: "Simulator connected" appears immediately

## Troubleshooting

### If only one container starts:
- Check: `docker-compose ps`
- Should show 5 containers for "multi" profile
- Fixed: All drones now have "multi" profile

### If AirSim shows "not connected":
```powershell
# Run step-by-step diagnostics
.\test_connection_step_by_step.bat

# Quick connectivity check
.\quick_gps_check.bat

# Check PX4 logs
docker logs px4-single          # for single drone
docker logs px4-drone-1         # for multi drone
```

### Common Connection Issues:
1. **Wrong settings.json**: Use `copy_airsim_settings.bat` to ensure correct config
2. **PX4 not ready**: Wait 10-15 seconds after container start
3. **Port conflicts**: Run `cleanup_networks.bat` first
4. **Network issues**: Check `docker network ls` for conflicts

## Expected Behavior

### Single Drone:
- 1 container: `px4-single`
- AirSim vehicle: `PX4` on port 4561
- QGC port: 14550

### Multi Drone:
- 5 containers: `px4-drone-1` through `px4-drone-5`
- AirSim vehicles: `Drone1` through `Drone5` on ports 4561-4565
- QGC ports: 14550-14554

## Files Overview
- `launch_with_gps_fix.bat` - Enhanced launcher with network cleanup
- `copy_airsim_settings.bat` - Automatic settings configuration
- `test_connection_step_by_step.bat` - Guided troubleshooting
- `quick_gps_check.bat` - Fast connectivity test
- `cleanup_networks.bat` - Network conflict resolution
- `settings/` - Proper AirSim configuration files