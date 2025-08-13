# 📁 Multirotor Directory Organization

This document outlines the organized structure of the multirotor directory.

## 🗂️ **Current Structure**

```
PythonClient/multirotor/
├── mission_planning/           # 🚁 MISSION PLANNING FRAMEWORK
│   ├── README.md              # Mission planning documentation
│   ├── generic_mission.py     # Abstract base class for all missions
│   ├── unreal_mission_interface.py # Unreal Engine integration
│   ├── box_mission.py         # Rectangular box patterns
│   ├── spiral_search_mission.py # Spiral search patterns
│   ├── grid_survey_mission.py # Professional mapping surveys
│   ├── figure8_mission.py     # Figure-8 surveillance patterns
│   ├── wave_mission.py        # Sinusoidal wave patterns
│   ├── star_pattern_mission.py # Radiating spoke patterns
│   ├── star_mission.py        # Alternative star pattern
│   └── example_custom_mission.py # Template for new missions
├── photos/                    # 📸 MISSION PHOTOS
│   ├── box_mission_photos/    # Box mission photo storage
│   ├── spiral_search_photos/  # Spiral search photo storage
│   ├── grid_survey_photos/    # Grid survey photo storage
│   ├── figure8_mission_photos/ # Figure-8 photo storage
│   ├── wave_mission_photos/   # Wave mission photo storage
│   └── star_mission_photos/   # Star pattern photo storage
├── examples/                  # 📚 EXAMPLE SCRIPTS
│   ├── hello_drone.py         # Basic drone control example
│   ├── takeoff.py             # Simple takeoff example
│   ├── navigate.py            # Navigation examples
│   ├── orbit.py               # Orbital movement example
│   └── survey.py              # Survey pattern example
├── utils/                     # 🔧 UTILITY SCRIPTS
│   ├── setup_path.py          # Path setup utilities
│   ├── state.py               # State monitoring utilities
│   ├── teleport.py            # Teleportation utilities
│   └── reset_test_drone.py    # Drone reset utilities
├── tests/                     # 🧪 TESTING SCRIPTS
│   ├── test_connection.py     # Connection testing
│   ├── test_collision.py      # Collision testing
│   ├── stability_test.py      # Stability testing
│   └── drone_stress_test.py   # Stress testing
├── sensors/                   # 📡 SENSOR SCRIPTS
│   ├── drone_lidar.py         # LiDAR functionality
│   ├── high_res_camera.py     # High resolution camera
│   ├── point_cloud.py         # Point cloud processing
│   └── kinect_publisher.py    # Kinect sensor integration
├── advanced/                  # 🚀 ADVANCED FEATURES
│   ├── multi_agent_drone.py   # Multi-drone coordination
│   ├── external_physics_engine.py # Custom physics
│   ├── gimbal.py              # Gimbal control
│   └── set_wind.py            # Weather simulation
└── README.md                  # Main documentation
```

## 🎯 **Quick Navigation**

### **For Mission Development:**
```bash
cd PythonClient/multirotor/mission_planning
```

### **For Testing:**
```bash
cd PythonClient/multirotor/tests
python test_connection.py
```

### **For Examples:**
```bash
cd PythonClient/multirotor/examples
python hello_drone.py
```

## 📋 **TODO: Complete Organization**

The following files still need to be organized:

**Move to examples/**:
- `hello_drone.py`
- `takeoff.py` 
- `navigate.py`
- `orbit.py`
- `survey.py`
- `manual_mode_demo.py`
- `box.py`
- `path.py`
- `land.py`
- `arm.py`
- `disarm.py`

**Move to utils/**:
- `setup_path.py`
- `state.py`
- `teleport.py`
- `reset_test_drone.py`
- `set_trace_line.py`
- `opencv_show.py`
- `wav_reader.py`
- `speaker.py`
- `pause_continue_drone.py`
- `line_of_sight.py`
- `clock_speed.py`
- `add_drone.py`

**Move to tests/**:
- `test_connection.py` (already exists)
- `test_collision.py`
- `stability_test.py`
- `drone_stress_test.py`

**Move to sensors/**:
- `drone_lidar.py`
- `vehicleframe_lidar_pointcloud.py`
- `sensorframe_lidar_pointcloud.py`
- `high_res_camera.py`
- `point_cloud.py`
- `kinect_publisher.py`

**Move to advanced/**:
- `multi_agent_drone.py`
- `external_physics_engine.py`
- `gimbal.py`
- `set_wind.py`
- `set_fog.py`

**Keep in root:**
- `README.md`
- `ORGANIZATION.md`
- `params.txt`
- `Error.wav`
- `__pycache__/` (auto-generated)

## 🔄 **Migration Notes**

- All mission scripts are now in `mission_planning/`
- Photo directories remain in their current structure
- Import statements may need updating after complete reorganization
- The main README has been updated to reflect the new structure 