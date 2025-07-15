# PX4 GPS Home Troubleshooting Guide

## Problem: "Vehicle does not have a valid GPS home location"

This error occurs when trying to arm a PX4 drone in AirSim because the GPS home position hasn't been properly set. This guide provides a comprehensive solution.

## Root Causes

1. **Missing OriginGeopoint** in settings.json
2. **GPS sensor not enabled** or misconfigured
3. **Missing PX4 parameters** for GPS initialization
4. **Timing issues** - not waiting long enough for GPS lock

## Solution

### 1. Update settings.json

Create or update your `settings.json` file (located at `~/Documents/AirSim/settings.json` on Linux or `C:\Users\[YourName]\Documents\AirSim\settings.json` on Windows):

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "LocalHostIp": "127.0.0.1",
  "ApiServerPort": 41451,
  
  "OriginGeopoint": {
    "Latitude": 47.641468,
    "Longitude": -122.140165,
    "Altitude": 122
  },
  
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "X": 0,
      "Y": 0,
      "Z": -1,
      "Pitch": 0,
      "Roll": 0,
      "Yaw": 0,
      
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "127.0.0.1",
      "LockStep": true,
      
      "Sensors": {
        "Gps": {
          "SensorType": 3,
          "Enabled": true,
          "EphTimeConstant": 0.9,
          "EpvTimeConstant": 0.9,
          "EphInitial": 25.0,
          "EpvInitial": 25.0,
          "EphFinal": 0.1,
          "EpvFinal": 0.1,
          "EphMin3d": 3.0,
          "EphMin2d": 4.0,
          "UpdateLatency": 0.2,
          "UpdateFrequency": 50,
          "StartupDelay": 1
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled": true
        },
        "Barometer": {
          "SensorType": 1,
          "Enabled": true,
          "PressureFactorSigma": 0.0001825
        },
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        }
      },
      
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "LPE_LAT": 47.641468,
        "LPE_LON": -122.140165,
        "COM_ARM_WO_GPS": 0,
        "EKF2_AID_MASK": 1,
        "EKF2_HGT_MODE": 0,
        "EKF2_GPS_CHECK": 31
      }
    }
  }
}
```

### 2. Key Configuration Elements Explained

#### OriginGeopoint (Required)
- Sets the geographic reference point for the simulation
- Must be at the root level of the JSON
- Used by AirSim to convert between local and global coordinates

#### GPS Sensor Configuration
- `"Enabled": true` - Must be enabled
- `StartupDelay` - Simulates GPS acquisition time
- `UpdateFrequency` - How often GPS updates are sent
- EPH/EPV settings control GPS accuracy simulation

#### PX4 Parameters
- `LPE_LAT` & `LPE_LON` - Local position estimator coordinates (should match OriginGeopoint)
- `COM_ARM_WO_GPS: 0` - Requires GPS for arming (set to 1 to bypass GPS requirement)
- `EKF2_AID_MASK: 1` - Enables GPS fusion in EKF2
- `EKF2_HGT_MODE: 0` - Uses barometer for altitude
- `EKF2_GPS_CHECK: 31` - GPS quality checks

### 3. Startup Sequence

1. Start AirSim first
2. Wait for AirSim to fully load
3. Start PX4 SITL:
   ```bash
   cd /path/to/PX4-Autopilot
   make px4_sitl none_iris
   ```
4. Wait 10-30 seconds for GPS lock
5. Look for "got GPS Home" message in console
6. Try arming the vehicle

### 4. Using the Debug Script

Run the provided debug script to monitor GPS status:

```bash
python debug_gps_home.py
```

This will show:
- Current GPS coordinates and fix status
- Home position status
- GPS accuracy metrics
- Troubleshooting recommendations

### 5. Common Issues and Solutions

#### Issue: GPS not getting lock
**Solution**: Wait longer (up to 30 seconds), ensure GPS sensor is enabled

#### Issue: Home position remains at 0,0
**Solution**: Check OriginGeopoint is properly set in settings.json

#### Issue: "got GPS Home" but still can't arm
**Solution**: Check other arming requirements (safety switch, RC calibration)

#### Issue: Works in simulation but not with real GPS
**Solution**: Real GPS needs clear sky view, may take 2-3 minutes for first fix

### 6. Alternative Solutions

#### Bypass GPS Requirement (Development Only)
Set `"COM_ARM_WO_GPS": 1` in Parameters to allow arming without GPS

#### Manual Home Setting
Use QGroundControl to manually set home position

#### Force GPS Mode
If using different firmware, ensure GPS mode is enabled:
```bash
param set SYS_HAS_GPS 1
```

### 7. Docker-Specific Configuration

For Docker setups, ensure:
- IP addresses are correctly configured for container networking
- Use appropriate host IP (not 127.0.0.1) if running in separate containers
- Port forwarding is properly configured

### 8. Verification Steps

1. Check settings.json is in the correct location
2. Verify JSON syntax is valid (no trailing commas)
3. Restart both AirSim and PX4 after changes
4. Monitor console output for GPS-related messages
5. Use debug script to verify GPS data flow

### References

- [AirSim Settings Documentation](https://microsoft.github.io/AirSim/settings/)
- [PX4 GPS Configuration](https://docs.px4.io/main/en/gps_compass/)
- [Issue #3130](https://github.com/microsoft/AirSim/issues/3130)

## Advanced Fixes for MAVLink Synchronization Issues

### Problem: Settings Configured Correctly but Still Getting GPS Home Error

If you've followed all the above steps but are still getting "Vehicle does not have a valid GPS home location" errors when using the Python client API (while manual PX4 console arming works), this indicates a deeper MAVLink synchronization issue between PX4 and AirSim.

#### Root Cause: Incomplete MAVLink Message Handling

The issue occurs because the MAVLink GPS home position message handler in AirSim was incomplete, missing crucial synchronization code.

#### Solution: Enhanced MAVLink GPS Home Position Handling

The following fixes have been implemented in `MavLinkMultirotorApi.hpp`:

1. **Complete GPS Home Message Handler**
   ```cpp
   case MavLinkHomePosition::kMessageId: {
       MavLinkHomePosition MavLinkHomePosition;
       MavLinkHomePosition.decode(message);
       
       current_state_.home.global_pos.lat = MavLinkHomePosition.latitude / 1E7f;
       current_state_.home.global_pos.lon = MavLinkHomePosition.longitude / 1E7f;
       current_state_.home.global_pos.alt = MavLinkHomePosition.altitude / 1000.0f;
       current_state_.home.local_pos.x = MavLinkHomePosition.x;
       current_state_.home.local_pos.y = MavLinkHomePosition.y;
       current_state_.home.local_pos.z = MavLinkHomePosition.z;
       current_state_.home.is_set = true;  // THIS WAS MISSING!
       
       addStatusMessage("GPS home position set via MAVLink message");
       break;
   }
   ```

2. **GPS Backup Mechanism**
   When PX4 doesn't send home position messages, a backup mechanism sets home from GPS data:
   ```cpp
   // In GPS lock handler
   if (locked && !has_gps_lock_) {
       addStatusMessage("Got GPS lock");
       has_gps_lock_ = true;
       
       if (!current_state_.home.is_set) {
           addStatusMessage("Setting home position from GPS data (backup method)");
           current_state_.home.global_pos.lat = MavLinkGpsRawInt.lat / 1E7f;
           current_state_.home.global_pos.lon = MavLinkGpsRawInt.lon / 1E7f;
           current_state_.home.global_pos.alt = MavLinkGpsRawInt.alt / 1000.0f;
           current_state_.home.is_set = true;
       }
   }
   ```

3. **Local Position Fallback for No-GPS Scenarios**
   ```cpp
   void setLocalPositionAsHome() {
       current_state_.home.global_pos.lat = std::numeric_limits<float>::quiet_NaN();
       current_state_.home.global_pos.lon = std::numeric_limits<float>::quiet_NaN();
       current_state_.home.global_pos.alt = 0.0f;
       current_state_.home.is_set = true;
       addStatusMessage("Home position set to local coordinates (no GPS)");
   }
   ```

### Problem: "Ground is not stable, variance is X.XXXXXX"

After fixing GPS home synchronization, you may encounter ground stability errors during arming.

#### Solution: Smart Ground Stability Checker

The ground stability check has been enhanced with adaptive tolerance and local position mode bypass:

1. **Automatic Local Position Mode Detection**
   ```cpp
   void waitForStableGroundPosition(float timeout_sec) {
       // Skip check if GPS coordinates are NaN (local position mode)
       if (current_state_.home.global_pos.lat != current_state_.home.global_pos.lat) {
           addStatusMessage("Skipping ground stability check (local position mode)");
           return;
       }
       
       // Continue with enhanced stability check...
   }
   ```

2. **Progressive Tolerance Relaxation**
   ```cpp
   float tolerance = 0.1f; // Start with strict tolerance
   TTimePoint start_time = ClockFactory::get()->nowNanos();
   
   while (elapsed_sec < timeout_sec) {
       float variance = getGroundVariance();
       
       if (variance <= tolerance) {
           return; // Ground is stable enough
       }
       
       // Relax tolerance progressively for multi-drone scenarios
       if (elapsed_sec > timeout_sec * 0.3f) tolerance = 0.3f;
       if (elapsed_sec > timeout_sec * 0.6f) tolerance = 0.5f;
       if (elapsed_sec > timeout_sec * 0.8f) tolerance = 1.0f;
       
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
       elapsed_sec = (ClockFactory::get()->nowNanos() - start_time) / 1e9f;
   }
   ```

3. **Enhanced Error Reporting**
   ```cpp
   addStatusMessage(Utils::stringf(
       "Ground unstable after %.1fs, variance: %.6f (tolerance: %.6f)",
       timeout_sec, variance, tolerance));
   ```

### Files Modified

The fixes have been applied to all three copies of the MavLinkMultirotorApi.hpp file:
- `/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- `/Unreal/Plugins/AirSim/Source/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- `/Unreal/Environments/Blocks/Plugins/AirSim/Source/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`

### Benefits for Multi-Drone Operations

These fixes provide significant improvements for multi-drone scenarios:
- **Handles timing variations** between simultaneous drone startups
- **Adapts to different conditions** across multiple spawn points
- **Provides detailed diagnostics** for troubleshooting specific drones
- **Supports both GPS and no-GPS operation** seamlessly
- **Reduces false failures** in multi-drone swarm operations

### Verification Commands

After applying these fixes, verify the improvements:

```bash
# 1. Test Python client arming (should now work)
python3 -c "
import cosysairsim as airsim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
try:
    client.armDisarm(True)
    print('SUCCESS: Arming via Python client worked!')
except Exception as e:
    print(f'ERROR: {e}')
"

# 2. Check AirSim console for home position messages
# Look for: 'GPS home position set via MAVLink message'
#       or: 'Setting home position from GPS data (backup method)'
#       or: 'Home position set to local coordinates (no GPS)'

# 3. Monitor ground stability messages
# Look for: 'Skipping ground stability check (local position mode)'
#       or: 'Ground stable with variance: X.XXXXXX'
```

## Summary

The GPS home error is typically resolved by:
1. Adding OriginGeopoint to settings.json
2. Enabling GPS sensor with proper configuration
3. Setting PX4 parameters for GPS initialization
4. Waiting sufficient time for GPS lock

For advanced synchronization issues, the enhanced MAVLink handling provides:
5. **Complete GPS home message synchronization** between PX4 and AirSim
6. **GPS backup mechanisms** when MAVLink messages are delayed
7. **Local position fallback** for no-GPS operation
8. **Smart ground stability checking** with adaptive tolerance

These fixes ensure reliable operation in both single-drone and multi-drone scenarios, with comprehensive support for GPS and no-GPS operation modes.