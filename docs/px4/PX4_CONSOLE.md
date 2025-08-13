# PX4 Console Commands Reference

This document provides a comprehensive list of PX4 console commands, with special focus on GPS configuration and diagnostics.

## Table of Contents
- [GPS Status and Information Commands](#gps-status-and-information-commands)
- [GPS Parameter Configuration Commands](#gps-parameter-configuration-commands)
- [GPS Driver Commands](#gps-driver-commands)
- [RTK GPS Commands](#rtk-gps-commands)
- [Diagnostic and Testing Commands](#diagnostic-and-testing-commands)
- [Navigation and EKF Commands](#navigation-and-ekf-commands)
- [Save and Load Configuration](#save-and-load-configuration)
- [Common GPS Troubleshooting Workflow](#common-gps-troubleshooting-workflow)

## GPS Status and Information Commands

### Basic Status Commands
```bash
gps status                    # Display GPS module status, satellite count, fix type
listener sensor_gps           # Listen to raw GPS sensor data
listener vehicle_gps_position # Listen to processed GPS position data
listener sensor_gnss_relative # Listen to relative GNSS data (for RTK)
```

### Monitoring Commands
```bash
listener sensor_gps -n 10             # Listen to 10 GPS messages
listener vehicle_gps_position -n 5    # Listen to 5 position messages
listener sensor_gps -r 1              # Listen at 1 Hz rate
```

## GPS Parameter Configuration Commands

### View Parameters
```bash
param show GPS*               # Show all GPS-related parameters
param show SER_GPS*           # Show GPS serial port parameters
param show EKF2_GPS*          # Show EKF2 GPS parameters
param show GNSS*              # Show GNSS-related parameters
param show RTK*               # Show RTK-related parameters
```

### Primary GPS Configuration
```bash
param set GPS_1_CONFIG <port>     # Set GPS1 port (e.g., 102 for TELEM2)
param set GPS_1_PROTOCOL <type>   # Set protocol (1=u-blox, 2=MTK, 3=Ashtech, 4=Emlid)
param set SER_GPS1_BAUD <rate>    # Set baud rate (0=Auto, 115200, 230400, etc.)
```

**GPS Protocol Values:**
- `1` = u-blox (default)
- `2` = MTK
- `3` = Ashtech
- `4` = Emlid

### Secondary GPS Configuration
```bash
param set GPS_2_CONFIG <port>     # Set GPS2 port
param set GPS_2_PROTOCOL <type>   # Set GPS2 protocol
param set SER_GPS2_BAUD <rate>    # Set GPS2 baud rate
```

### Advanced GPS Settings
```bash
param set GPS_YAW_OFFSET <degrees>      # GPS antenna yaw offset
param set GPS_1_GNSS <bitmask>          # Enable GNSS systems (GPS, GLONASS, Galileo, BeiDou)
param set GPS_2_GNSS <bitmask>          # Secondary GPS GNSS systems
param set GNSS_MAX_MAIN_SAT <count>     # Maximum satellites to use
param set EKF2_GPS_CTRL <bitmask>       # EKF2 GPS control settings
```

**GNSS System Bitmask:**
- `1` = GPS
- `2` = GLONASS
- `4` = Galileo
- `8` = BeiDou
- `16` = QZSS
- `32` = SBAS

## GPS Driver Commands

### Driver Control
```bash
gps start                     # Start GPS driver
gps stop                      # Stop GPS driver
gps restart                   # Restart GPS driver
gps info                      # Show GPS driver info
```

### Advanced Driver Commands
```bash
gps start -d <device>         # Start GPS on specific device
gps start -b <baudrate>       # Start GPS with specific baud rate
gps start -p <protocol>       # Start GPS with specific protocol
```

## RTK GPS Commands

### RTK Configuration
```bash
param set GPS_UBX_DYNMODEL <model>      # Dynamic model (0=Automotive, 2=Airborne)
param set RTK_AUTO_SETUP <enable>       # Enable RTK auto-setup
param set RTK_RATE <rate>               # RTK correction rate
param set GPS_UBX_CFG_INTF <enable>     # Enable interface configuration
```

**Dynamic Model Values:**
- `0` = Automotive
- `1` = Sea
- `2` = Airborne (recommended for drones)

### RTK Monitoring
```bash
listener vehicle_gps_position           # Check RTK fix status
listener sensor_gnss_relative          # Monitor RTK relative positioning
listener rtcm_message                  # Monitor RTCM correction messages
```

## Diagnostic and Testing Commands

### GPS Diagnostics
```bash
listener sensor_gps -n 10             # Listen to 10 GPS messages
listener vehicle_gps_position -n 5    # Listen to 5 position messages
perf                                   # Check performance counters
top                                    # Monitor system load
hist                                   # Show message history
```

### Hardware Diagnostics
```bash
i2cdetect -b 1                        # Scan I2C bus for devices
i2cdetect -b 2                        # Scan I2C bus 2
i2cdetect                             # Scan default I2C bus
```

### GPS Failure Simulation
```bash
failure gps off                       # Simulate GPS failure
failure gps ok                        # Restore GPS functionality
failure gps intermittent              # Simulate intermittent GPS issues
```

## Navigation and EKF Commands

### EKF2 GPS Configuration
```bash
param set EKF2_GPS_DELAY <ms>         # GPS measurement delay (default: 110ms)
param set EKF2_GPS_POS_X <m>          # GPS antenna X position
param set EKF2_GPS_POS_Y <m>          # GPS antenna Y position
param set EKF2_GPS_POS_Z <m>          # GPS antenna Z position
param set EKF2_GPS_V_NOISE <m/s>      # GPS velocity noise (default: 0.5)
param set EKF2_GPS_P_NOISE <m>        # GPS position noise (default: 0.5)
```

### EKF2 GPS Control
```bash
param set EKF2_GPS_CTRL <bitmask>     # EKF2 GPS control settings
```

**EKF2_GPS_CTRL Bitmask:**
- `1` = Horizontal position fusion
- `2` = Vertical position fusion
- `4` = Velocity fusion
- `8` = Dual antenna heading fusion

### Monitor Navigation
```bash
listener estimator_status             # Monitor EKF2 estimator
listener vehicle_attitude             # Check attitude estimation
listener vehicle_local_position      # Check local position
listener vehicle_global_position     # Check global position
```

## Save and Load Configuration

### Parameter Management
```bash
param save                            # Save parameters to storage
param reset                           # Reset all parameters to defaults
param reset_nostart                   # Reset parameters without restarting
param load <filename>                 # Load parameters from file
param export <filename>               # Export parameters to file
```

### Backup and Restore
```bash
param export /fs/microsd/my_params.txt    # Export to SD card
param load /fs/microsd/my_params.txt      # Load from SD card
```

## Common GPS Troubleshooting Workflow

### Step-by-Step Troubleshooting
```bash
# 1. Check GPS status
gps status

# 2. Check if GPS data is being received
listener sensor_gps -n 5

# 3. Verify GPS parameters
param show GPS_1_CONFIG
param show GPS_1_PROTOCOL
param show SER_GPS1_BAUD

# 4. Check EKF2 GPS usage
listener estimator_status

# 5. Monitor GPS position
listener vehicle_gps_position -n 10

# 6. Check satellite count and fix type
listener sensor_gps -n 1

# 7. Verify GPS hardware connection
i2cdetect -b 1

# 8. Test GPS restart
gps restart
```

### GPS Fix Type Interpretation
- `0` = No fix
- `1` = Dead reckoning only
- `2` = 2D fix
- `3` = 3D fix
- `4` = GNSS + dead reckoning
- `5` = Time-only fix
- `6` = RTK Float
- `7` = RTK Fixed

### Common Issues and Solutions

#### No GPS Data
```bash
# Check if GPS driver is running
gps status

# Restart GPS driver
gps restart

# Check serial port configuration
param show SER_GPS1_BAUD
param show GPS_1_CONFIG
```

#### Poor GPS Accuracy
```bash
# Check satellite count
listener sensor_gps -n 1

# Verify GNSS systems enabled
param show GPS_1_GNSS

# Check EKF2 GPS noise settings
param show EKF2_GPS_P_NOISE
param show EKF2_GPS_V_NOISE
```

#### RTK Issues
```bash
# Check RTK configuration
param show RTK_AUTO_SETUP
param show GPS_UBX_DYNMODEL

# Monitor RTK corrections
listener rtcm_message

# Check relative positioning
listener sensor_gnss_relative
```

## Additional Utilities

### System Information
```bash
ver                                   # Show PX4 version
uname                                 # Show system information
dmesg                                 # Show kernel messages
```

### File System
```bash
ls /fs/microsd                        # List SD card contents
cat /fs/microsd/params                # Show saved parameters
```

### Network (for MAVLink)
```bash
mavlink status                        # Show MAVLink status
mavlink stream                        # Show MAVLink streams
```

---

**Note:** This reference covers the most commonly used PX4 console commands for GPS configuration and diagnostics. For the latest command syntax and options, always refer to the official PX4 documentation or use `help <command>` in the PX4 console.