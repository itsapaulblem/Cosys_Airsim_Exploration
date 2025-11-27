# REP 105 Frame Authorities Compliance Testing Guide

## 🎯 Overview
This guide validates that the AirSim ROS2 integration now fully complies with REP 105 Frame Authorities specification, fixing the previous static transform violations.

## ✅ REP 105 Compliance Achievements

### 1. **Fixed Frame Authority Violations**
- ❌ **REMOVED**: Static `map→odom` transform (violated REP 105)
- ✅ **ADDED**: Dynamic `map→odom` transform via localization component
- ✅ **MAINTAINED**: Proper `odom→base_link` authority by odometry sources

### 2. **Implemented Proper Architecture**
- **Odometry Source**: `vehicle_node_base.cpp` publishes `odom→base_link` ✅
- **Localization Component**: `localization_node.cpp` publishes `map→odom` ✅  
- **Static Transforms**: Only sensor transforms (REP 105 compliant) ✅

## 🧪 Testing Procedures

### Prerequisites
```bash
# 1. Build the updated ROS2 workspace
cd ros2
colcon build --packages-select airsim_ros_pkgs

# 2. Source the workspace  
source install/setup.bash

# 3. Start AirSim with a vehicle (e.g., Blocks environment with drone_1)
```

### Test 1: REP 105 Frame Authorities Validation

**Launch the REP 105 compliant system (UPDATED - localization now integrated!):**
```bash
# Single command launches EVERYTHING with REP 105 compliance:
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py enable_localization:=true

# Or disable localization to see the broken chain:
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py enable_localization:=false

# Verify the complete transform chain
ros2 run tf2_ros tf2_echo map drone_1/base_link1
```

**Expected Result**: ✅ Complete transform chain working:
```
map → drone_1/odom → drone_1/base_link1 → drone_1/lidarSensor1_link
```

### Test 2: Frame Authority Compliance Check

**Verify proper frame authorities:**
```bash
# Check who publishes each transform
ros2 run tf2_tools view_frames

# Verify dynamic map→odom (should update at ~20Hz)
ros2 topic echo /tf --field transforms[0].header.stamp

# Verify odometry authority (should match odom→base_link)
ros2 topic echo /drone_1/odom --field header.frame_id
```

**Expected Results**:
- ✅ `map→odom`: Published by `localization_drone_1` node
- ✅ `odom→base_link1`: Published by vehicle odometry source
- ✅ `base_link1→sensors`: Static transforms by vehicle node

### Test 3: REP 105 Behavior Validation

**Test proper localization authority pattern:**
```bash
# 1. Move the drone in AirSim
# 2. Observe that map→odom updates dynamically (localization)
# 3. Observe that odom→base_link updates continuously (odometry)

# Monitor transform updates
ros2 run tf2_ros tf2_monitor map drone_1/odom drone_1/base_link1
```

**Expected Behavior**:
- ✅ **map→odom**: Updates based on "localization corrections" from AirSim ground truth
- ✅ **odom→base_link**: Updates continuously from vehicle odometry
- ✅ **Complete chain**: Always connected without missing transforms

## 🔍 Debugging REP 105 Issues

### Common Issues and Solutions

**Issue**: `tf2_echo` shows "frame does not exist"
```bash
# Solution: Check that localization node is running
ros2 node list | grep localization
ros2 topic list | grep tf
```

**Issue**: Transforms update too slowly
```bash
# Check localization update rate (should be ~20Hz)
ros2 topic hz /tf --filter="transforms[0].child_frame_id=='drone_1/odom'"
```

**Issue**: Frame authorities violated
```bash
# Verify no static map→odom transforms exist
ros2 topic echo /tf_static --once
# Should NOT contain map→odom transforms
```

## 📊 REP 105 Compliance Checklist

### ✅ Frame Authorities (REP 105 §Frame Authorities)
- [ ] **Odometry sources** publish `odom→base_link` transforms ✅
- [ ] **Localization components** receive `odom→base_link` and publish `map→odom` ✅  
- [ ] **NO direct** `map→base_link` transforms published ✅
- [ ] **Dynamic computation** of localization correction ✅

### ✅ Transform Chain Structure (REP 105 §Coordinate Frames)
- [ ] `map` frame as world-fixed global reference ✅
- [ ] `odom` frame as continuous local reference ✅
- [ ] `base_link` frame attached to robot base ✅
- [ ] Sensor frames properly connected to `base_link` ✅

### ✅ Naming Conventions (REP 105 §Specification)
- [ ] Namespace-style frame naming (`drone_1/odom`, `drone_1/base_link1`) ✅
- [ ] Standard frame names (`map`, `odom`, `base_link`) ✅
- [ ] Sensor frame naming (`drone_1/lidarSensor1_link`) ✅

## 🎉 Expected Final Result

**Perfect REP 105 Compliant Transform Tree:**
```
map
└── drone_1/odom              [Published by: localization_drone_1]
    └── drone_1/base_link1    [Published by: vehicle odometry]
        ├── drone_1/camera0_body
        ├── drone_1/camera0_optical  
        ├── drone_1/lidarSensor1_link
        └── [other sensor frames...]
```

**Frame Authority Compliance:**
- 🌐 **Localization Component**: Owns `map→odom` (dynamic, 20Hz)
- 🚗 **Odometry Source**: Owns `odom→base_link` (continuous updates)
- 📡 **Static Publishers**: Own sensor transforms only

This implementation now fully complies with ROS REP 105 Frame Authorities specification!