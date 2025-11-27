# PX4 AirSim Velocity Command Troubleshooting Guide

## The Problem: "My PX4 drone doesn't move with velocity commands"

If your velocity commands have correct structure but the drone still doesn't move, you're likely hitting **PX4-specific requirements** that differ from SimpleFlight drones.

## Root Causes Identified

### 🚨 Critical Issue #1: Command Duration = 0.05 Seconds
**Problem**: Each velocity command only lasts 0.05 seconds in the ROS2 wrapper code.
```cpp
// In multirotor_node.cpp line 210
multirotor_client->moveByVelocityAsync(
    current_vel_cmd_.x, current_vel_cmd_.y, current_vel_cmd_.z,
    0.05,  // ← Only 0.05 seconds!
    ...
);
```
**Result**: Single commands are imperceptible - drone moves briefly then stops.

### 🚨 Critical Issue #2: Drone Must Be Airborne
**Problem**: PX4 velocity commands only work when the drone is flying.
**Result**: Commands are ignored if drone is on the ground.

### 🚨 Critical Issue #3: PX4 Requires Continuous Commands  
**Problem**: PX4 has a timeout mechanism - without regular commands, it reverts to its previous flight mode.
**Result**: Single commands don't sustain movement.

## Complete Working Solution

### Step 1: Verify Your Setup
```bash
# Check RPC dynamic launch is running
ros2 node list | grep -E "(PX4_Drone1|airsim_coordination)"

# Verify topics exist  
ros2 topic list | grep PX4_Drone1

# Expected topics for PX4_Drone1:
# /PX4_Drone1/vel_cmd_body_frame
# /PX4_Drone1/vel_cmd_world_frame  
# /PX4_Drone1/takeoff
# /PX4_Drone1/land
```

### Step 2: Takeoff First (REQUIRED)
```bash
# PX4 velocity commands only work when airborne
ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'

# Wait for takeoff to complete (3-5 seconds)
# You should see the drone rise in the simulation
```

### Step 3: Send Continuous Velocity Commands
```bash
# ✅ CORRECT: Continuous commands with --rate flag
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 1.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  },
  drivetrain: 0,
  yaw_mode: {
    is_rate: true,
    yaw_or_rate: 0.0
  }
}'

# ❌ WRONG: Single command (will not work with PX4)
ros2 topic pub /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '...'
```

### Step 4: Stop Movement
```bash
# Press Ctrl+C to stop the velocity command
# Then send hover command to maintain position
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 0.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  },
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}' &

# Or use the helper script
python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover --continuous
```

## Complete Working Example Sequence

```bash
# Terminal 1: Start ROS2 (if not already running)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Terminal 2: Execute flight sequence
cd ~/Cosys-AirSim/ros2/python_examples

# 1. Takeoff
echo "Taking off..."
ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'
sleep 5

# 2. Move forward for 5 seconds
echo "Moving forward..."
timeout 5 ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'

# 3. Move right for 3 seconds  
echo "Moving right..."
timeout 3 ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'

# 4. Hover for 2 seconds
echo "Hovering..."
timeout 2 ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}'

# 5. Land
echo "Landing..."
ros2 service call /PX4_Drone1/land airsim_interfaces/srv/Land '{}'
```

## Using the Helper Script (Easier Method)

```bash
cd ~/Cosys-AirSim/ros2/python_examples

# 1. Takeoff
ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'
sleep 5

# 2. Move forward continuously (Ctrl+C to stop)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 1.0 --continuous

# 3. In another terminal, you can change direction
python3 velocity_command_helper.py --vehicle PX4_Drone1 --right 0.5 --continuous

# 4. Stop and hover
python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover --continuous

# 5. Land when done
ros2 service call /PX4_Drone1/land airsim_interfaces/srv/Land '{}'
```

## Key Differences: PX4 vs SimpleFlight

| Aspect | PX4 | SimpleFlight |
|--------|-----|--------------|
| **Takeoff Required** | ✅ Must takeoff first | ❌ Can move on ground |
| **Continuous Commands** | ✅ Required (timeout) | ❌ Single commands work |
| **Command Duration** | 0.05s (need --rate) | Varies |
| **Flight Modes** | OFFBOARD mode required | Direct control |
| **Complexity** | Higher (realistic) | Lower (arcade-like) |

## Diagnostic Commands

### Check if RPC Dynamic is Working
```bash
# Should show individual vehicle nodes
ros2 node list
# Expected: /PX4_Drone1, /airsim_coordination_node

# Should NOT show: /airsim_node (that's legacy mode)
```

### Monitor Command Reception
```bash
# Watch if commands are being received
ros2 topic echo /PX4_Drone1/vel_cmd_body_frame
```

### Check Vehicle State
```bash
# Monitor drone position/velocity
ros2 topic echo /PX4_Drone1/odom_local_ned --once
```

### Verify Services Available
```bash
# Should show takeoff/land services
ros2 service list | grep PX4_Drone1
```

## Common Mistakes and Fixes

### ❌ Mistake: "I sent one command and nothing happened"
```bash
# Wrong: Single command
ros2 topic pub /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '...'
```
✅ **Fix**: Use continuous commands
```bash
# Correct: Continuous commands  
ros2 topic pub --rate 10 /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '...'
```

### ❌ Mistake: "Commands work but drone doesn't move"
**Cause**: Drone not airborne
✅ **Fix**: Always takeoff first
```bash
ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'
```

### ❌ Mistake: "Drone moves briefly then stops"  
**Cause**: 0.05 second duration + no continuous commands
✅ **Fix**: Use `--rate 10` for sustained movement

### ❌ Mistake: "Using legacy topic names"
```bash
# Wrong: Legacy topic (airsim_node prefix)
/airsim_node/PX4_Drone1/vel_cmd_body_frame
```
✅ **Fix**: RPC dynamic uses direct vehicle names
```bash
# Correct: Direct vehicle topic
/PX4_Drone1/vel_cmd_body_frame
```

## Emergency Procedures

### Force Stop All Movement
```bash
# Emergency hover (stops all movement)
python3 velocity_command_helper.py --vehicle PX4_Drone1 --stop

# Emergency land (if drone is misbehaving)
ros2 service call /PX4_Drone1/land airsim_interfaces/srv/Land '{}'
```

### Reset If Drone Gets Stuck
```bash
# Reset simulation
ros2 service call /airsim_coordination_node/reset airsim_interfaces/srv/Reset '{}'
```

## Summary for Your Specific Issue

Your command structure was **100% correct**:
```bash
ros2 topic pub /PX4_Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}},
  drivetrain: 0,
  yaw_mode: {is_rate: true, yaw_or_rate: 0.0}
}' --rate 10
```

The missing pieces were:
1. **Takeoff first**: `ros2 service call /PX4_Drone1/takeoff airsim_interfaces/srv/Takeoff '{}'`
2. **Continuous commands**: `--rate 10` flag  
3. **Understanding the 0.05s duration**: Single commands are too brief to see

## Next Steps

1. Try the complete working sequence above
2. Use the helper script for easier control
3. Always remember: **Takeoff → Continuous Commands → Land**
4. For complex missions, consider using the ROS2 action servers instead of raw velocity commands

This should resolve your velocity command issues completely!