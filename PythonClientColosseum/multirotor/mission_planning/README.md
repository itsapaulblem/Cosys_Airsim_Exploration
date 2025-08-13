# 🚁 Mission Planning Framework

This directory contains the complete autonomous drone mission planning framework.

## 📁 **Contents**

### **Core Framework**
- `generic_mission.py` - Abstract base class for all missions
- `unreal_mission_interface.py` - Interface for Unreal Engine integration

### **Mission Types**
- `box_mission.py` - Rectangular box patterns with orbital photography
- `spiral_search_mission.py` - Archimedes spiral search patterns
- `grid_survey_mission.py` - Professional mapping and survey patterns
- `figure8_mission.py` - Figure-8 surveillance patterns
- `wave_mission.py` - Sinusoidal wave coverage patterns
- `star_pattern_mission.py` - Radiating spoke patterns
- `example_custom_mission.py` - Template for creating custom missions

## 🚀 **Quick Start**

```bash
# Always activate the virtual environment first
# Windows: airsim_env\Scripts\activate
# Linux/macOS: source airsim_env/bin/activate

# Navigate to mission planning
cd PythonClient/multirotor/mission_planning

# Preview missions (safe dry-run)
python box_mission.py --preview
python spiral_search_mission.py --preview --pattern outward
python grid_survey_mission.py --preview --pattern boustrophedon

# Execute missions
python box_mission.py --box_size 300 --altitude 25
python spiral_search_mission.py --max_radius 400
python grid_survey_mission.py --width 600 --height 400
```

## 🏗️ **Creating Custom Missions**

1. Copy `example_custom_mission.py` as a template
2. Inherit from `GenericMission`
3. Implement the required abstract methods:
   - `print_mission_parameters()`
   - `execute_mission_logic(target_z)`
   - `get_planned_distance()`
   - `get_mission_specific_metrics()`

## 📊 **Mission Analytics**

All missions automatically generate:
- Real-time progress tracking
- Post-mission performance reports
- Quality scoring (A+ to D grades)
- Error logging and recommendations
- Photo organization and metadata

## 🛡️ **Safety Features**

- Emergency landing procedures
- Collision detection and avoidance
- Progressive error recovery
- Position verification
- Graceful mission abort handling

For complete documentation, see the main README in the parent directory. 