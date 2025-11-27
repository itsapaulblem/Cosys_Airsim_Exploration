# AirSim ROS2 Velocity Command Examples

This guide provides working examples for sending velocity commands to AirSim drones through ROS2.

## Quick Fix for Your Issue

**IMPORTANT**: If you're using **PX4** drones, the issue is NOT the message structure - it's the **PX4-specific requirements**:

1. **Must takeoff first**: `ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'`
2. **Must use continuous commands**: `--rate 10` flag (commands only last 0.05 seconds otherwise)
3. **Must use RPC Dynamic Launch**: Ensure you're using the correct topic names

**For PX4 users**: See `PX4_VELOCITY_TROUBLESHOOTING.md` for the complete solution.

**For SimpleFlight users**: Your command was missing the required message structure. Here's the correct format:

### ✅ Correct Command (Full Structure)
```bash
ros2 topic pub /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 1.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  },
  drivetrain: 0,
  yaw_mode: {
    is_rate: true,
    yaw_or_rate: 0.0
  }
}' --rate 10
```

### ❌ Incorrect Command (Your Original - Missing Fields)
```bash
# This causes the drone to fall because fields are missing
ros2 topic pub /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd
```

## Using the Helper Script

We've created a Python helper script that makes sending velocity commands much easier:

### Setup
```bash
cd ~/Cosys-AirSim/ros2
source install/setup.bash
cd python_examples
```

### Basic Usage Examples

#### 1. Move Forward
```bash
# Move forward at 1 m/s
python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 1.0

# Move forward continuously (press Ctrl+C to stop)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 1.0 --continuous
```

#### 2. Move in Multiple Directions
```bash
# Move diagonally forward-right and up
python3 velocity_command_helper.py --vehicle PX4_Drone1 --x 1.0 --y 0.5 --z -0.5
```

#### 3. Hover/Stop
```bash
# Stop all movement and hover
python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover
```

#### 4. Move Up/Down
```bash
# Move up at 0.5 m/s (negative z in NED coordinates)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --up 0.5

# Move down at 0.5 m/s
python3 velocity_command_helper.py --vehicle PX4_Drone1 --down 0.5
```

#### 5. Rotate
```bash
# Rotate clockwise while hovering
python3 velocity_command_helper.py --vehicle PX4_Drone1 --yaw 0.5 --continuous
```

#### 6. World Frame Movement
```bash
# Move north (world frame) regardless of drone orientation
python3 velocity_command_helper.py --vehicle PX4_Drone1 --world-frame --x 1.0
```

## Direct ROS2 Command Examples

### Body Frame Commands (Relative to Drone's Heading)

#### Move Forward
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

#### Move Right
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

#### Move Up (Negative Z)
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

#### Diagonal Movement (Forward + Right + Up)
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.5, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

#### Rotate While Moving Forward
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.5}
}'
```

### World Frame Commands (Global Coordinates)

#### Move North
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

#### Move East
```bash
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'
```

## Understanding the Coordinate System

### NED (North-East-Down) Coordinates
AirSim uses the NED coordinate system:
- **X-axis**: North/Forward (positive = north/forward)
- **Y-axis**: East/Right (positive = east/right)  
- **Z-axis**: Down (positive = down, **negative = up**)

### Body Frame vs World Frame
- **Body Frame** (`vel_cmd_body_frame`): Movement relative to drone's current orientation
  - X = forward/backward relative to drone's nose
  - Y = left/right relative to drone's orientation
  - Z = up/down

- **World Frame** (`vel_cmd_world_frame`): Movement in global coordinates
  - X = north/south regardless of drone orientation
  - Y = east/west regardless of drone orientation
  - Z = up/down

## Message Structure Explained

```yaml
VelCmd:
  twist:                    # Velocity components
    linear:
      x: 1.0               # Forward velocity (m/s)
      y: 0.0               # Sideways velocity (m/s)
      z: 0.0               # Vertical velocity (m/s)
    angular:
      x: 0.0               # Roll rate (rad/s) - usually unused
      y: 0.0               # Pitch rate (rad/s) - usually unused
      z: 0.0               # Yaw rate (rad/s)
  drivetrain: 0            # 0 = MaxDegreeOfFreedom, 1 = ForwardOnly
  yaw_mode:
    is_rate: true          # true = use yaw rate, false = use absolute yaw
    yaw_or_rate: 0.0       # Yaw rate (if is_rate=true) or angle (if false)
```

## Common Patterns

### Square Pattern
```bash
# Move in a square pattern
# Forward
python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 1.0 --continuous &
sleep 5 && kill $!

# Right
python3 velocity_command_helper.py --vehicle PX4_Drone1 --right 1.0 --continuous &
sleep 5 && kill $!

# Backward
python3 velocity_command_helper.py --vehicle PX4_Drone1 --backward 1.0 --continuous &
sleep 5 && kill $!

# Left
python3 velocity_command_helper.py --vehicle PX4_Drone1 --left 1.0 --continuous &
sleep 5 && kill $!
```

### Circle Pattern
```bash
# Move in a circle (forward + rotation)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --x 1.0 --yaw 0.3 --continuous
```

### Emergency Stop
```bash
# Immediately stop all movement
python3 velocity_command_helper.py --vehicle PX4_Drone1 --stop
```

## Troubleshooting

### Issue: Drone Falls When Sending Commands
**Cause**: Incomplete message structure (missing twist, drivetrain, or yaw_mode fields)
**Solution**: Use the complete message format shown above or use the helper script

### Issue: Drone Doesn't Respond
**Check**:
1. Verify the vehicle name matches your settings.json
2. Ensure ROS2 node is running (`ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py`)
3. Check topic exists: `ros2 topic list | grep vel_cmd`
4. Verify message is being received: `ros2 topic echo /PX4_Drone1/vel_cmd_body_frame`

### Issue: Drone Moves in Wrong Direction
**Cause**: Confusion between body frame and world frame
**Solution**: 
- Use `vel_cmd_body_frame` for movement relative to drone's heading
- Use `vel_cmd_world_frame` for movement in global coordinates

### Issue: Drone Drifts After Command
**Cause**: Velocity commands are sent once instead of continuously
**Solution**: Use `--rate 10` flag with ros2 topic pub or `--continuous` with the helper script

## Testing Sequence

Here's a safe testing sequence to verify everything works:

```bash
# 1. First, make sure drone is flying
ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'

# 2. Test hover (should maintain position)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover --continuous &
sleep 3 && kill $!

# 3. Test gentle forward movement
python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 0.5 --continuous &
sleep 3 && kill $!

# 4. Stop
python3 velocity_command_helper.py --vehicle PX4_Drone1 --stop

# 5. Test upward movement
python3 velocity_command_helper.py --vehicle PX4_Drone1 --up 0.5 --continuous &
sleep 3 && kill $!

# 6. Return to hover
python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover
```

## Additional Resources

- [AirSim ROS2 Documentation](../README_MULTIROTOR_ARCHITECTURE.md)
- [ROS2 Usage Guide](../ROS2_USAGE_GUIDE.md)
- [AirSim API Documentation](https://microsoft.github.io/AirSim/apis/)

## Notes

1. Always include the `--rate` flag when using `ros2 topic pub` for continuous commands
2. The drone needs to be armed and flying (takeoff) before velocity commands work properly
3. Use lower velocities (0.5-1.0 m/s) for testing to avoid crashes
4. Remember that positive Z means down in NED coordinates
5. The helper script makes it much easier to send properly formatted commands