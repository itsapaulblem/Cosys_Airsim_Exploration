# 🚁 Autonomous Drone Mission Framework

A comprehensive Python framework for autonomous drone missions using Microsoft AirSim. This system provides professional-grade mission planning, execution, and analytics for various drone operations including search & rescue, mapping, and surveillance.

## 🌟 Features

- **🎯 Multiple Mission Types**: Box patterns, spiral search, grid surveys, figure-8 patterns, wave patterns, and star formations with orbital photography
- **📊 Real-time Analytics**: Comprehensive mission tracking and post-flight analysis
- **🛡️ Safety Systems**: Emergency landing procedures and progressive error recovery
- **🎥 Smooth Flight**: Velocity-based control for cinematography-quality motion
- **📸 Automated Photography**: Time-based and position-based photo capture
- **📈 Performance Metrics**: Flight efficiency, speed analysis, and quality scoring
- **🏗️ Extensible Architecture**: Template Method pattern for easy mission customization

## 📋 Prerequisites

### Software Requirements
- **Python 3.7+** 
- **Microsoft AirSim** (installed and configured)
- **Git** (for cloning the repository)

### Repository Setup
```bash
# Clone this repository
git clone <your-repository-url>
cd <repository-name>/PythonClient

# Set up the Python environment using the automated setup script
python setup_airsim_env.py

# Activate the virtual environment
# On Windows:
airsim_env\Scripts\activate

# On macOS/Linux:
source airsim_env/bin/activate
```

### AirSim Setup
1. Install and configure Microsoft AirSim
2. Ensure AirSim is configured for multirotor operation
3. Start AirSim/Unreal Engine before running missions

### Installation
The `setup_airsim_env.py` script automatically:
- ✅ Creates a dedicated virtual environment (`airsim_env`)
- ✅ Installs all required Python packages:
  - `airsim` (with proper build dependencies)
  - `numpy`
  - `opencv-python`
  - `matplotlib`
  - `msgpack-rpc-python`
  - `tornado`
- ✅ Handles complex dependency installation order
- ✅ Sets up everything needed for drone missions

**After setup, always activate the virtual environment before running missions:**
```bash
# Windows
airsim_env\Scripts\activate

# macOS/Linux  
source airsim_env/bin/activate

# Test your connection
cd multirotor
python test_connection.py

# Navigate to mission planning for missions
cd mission_planning
```

## 🚀 Quick Start

### 1. Environment Setup (Required Every Session)
**Important**: Always activate the virtual environment before running any missions:

```bash
# Navigate to your project directory
cd <repository-name>/PythonClient

# Activate the virtual environment
# Windows:
airsim_env\Scripts\activate

# macOS/Linux:
source airsim_env/bin/activate

# Verify activation (you should see (airsim_env) in your prompt)
# Then navigate to the mission scripts
cd multirotor/mission_planning
```

### 2. Test Your Connection
```bash
python test_connection.py
```

### 3. Preview Missions (Safe Dry-Run)
```bash
# Preview box mission
python box_mission.py --preview

# Preview spiral search
python spiral_search_mission.py --preview --pattern inward

# Preview grid survey
python grid_survey_mission.py --preview --pattern boustrophedon

# Preview figure-8 pattern
python figure8_mission.py --preview

# Preview wave pattern
python wave_mission.py --preview
```

### 4. Execute Real Missions
```bash
# Basic box mission
python box_mission.py --box_size 300 --altitude 25

# Search and rescue spiral
python spiral_search_mission.py --max_radius 400 --pattern outward

# Mapping survey
python grid_survey_mission.py --width 600 --height 400 --grid_spacing 30

# Figure-8 surveillance
python figure8_mission.py --loop_width 400 --loop_height 250 --loops_count 3

# Wave pattern coverage
python wave_mission.py --wave_amplitude 40 --wave_frequency 4
```

## 📖 Mission Types

### 🔲 Box Mission (`box_mission.py`)

Flies a rectangular box pattern with orbital photography at each vertex, perfect for area inspection and perimeter surveillance.

**Key Features:**
- Configurable box size and altitude
- Orbital photography at each vertex
- Smooth velocity-based movement
- Complete perimeter coverage

**Usage Examples:**
```bash
# Standard box mission
python box_mission.py --box_size 500 --altitude 30

# Detailed orbit photography
python box_mission.py --orbit_radius 75 --photos_per_orbit 12

# Preview without flying
python box_mission.py --preview --box_size 800
```

**Parameters:**
- `--box_size`: Box size in meters (default: 400)
- `--altitude`: Flight altitude in meters (default: 20)
- `--speed`: Flight speed in m/s (default: 5)
- `--orbit_radius`: Orbit radius for photography (default: 50)
- `--orbit_speed`: Orbital speed in m/s (default: 3)
- `--photos_per_orbit`: Photos per vertex orbit (default: 8)

**Applications:**
- Perimeter security surveillance
- Property boundary inspection
- Wildlife area monitoring
- Construction site assessment

---

### 🌀 Spiral Search Mission (`spiral_search_mission.py`)

Implements Archimedes spiral patterns for systematic area search operations, ideal for search & rescue and lost object recovery.

**Key Features:**
- Outward (center to edge) and inward (edge to center) patterns
- Configurable spiral spacing and coverage area
- Time-based photo intervals for complete coverage
- Mathematical precision using Archimedes spiral formula

**Usage Examples:**
```bash
# Outward search spiral
python spiral_search_mission.py --max_radius 600 --pattern outward

# Inward search with tight spacing
python spiral_search_mission.py --pattern inward --spiral_spacing 15

# High-altitude wide area search
python spiral_search_mission.py --altitude 50 --max_radius 1000
```

**Parameters:**
- `--max_radius`: Maximum spiral radius in meters (default: 500)
- `--spiral_spacing`: Distance between spiral arms (default: 25)
- `--pattern`: `outward` or `inward` spiral direction
- `--photo_interval`: Time between photos in seconds (default: 2.0)
- `--altitude`: Flight altitude in meters (default: 20)
- `--speed`: Flight speed in m/s (default: 5)

**Applications:**
- Search and rescue operations
- Lost person/object recovery
- Wildlife population surveys
- Area reconnaissance missions

---

### 🗺️ Grid Survey Mission (`grid_survey_mission.py`)

Professional photogrammetry and mapping surveys using boustrophedon or parallel flight line patterns.

**Key Features:**
- Boustrophedon (back-and-forth) and parallel flight patterns
- Configurable photo overlap for photogrammetry
- Grid rotation for optimal coverage
- Professional survey-grade accuracy

**Usage Examples:**
```bash
# Standard mapping survey
python grid_survey_mission.py --width 800 --height 600

# High-overlap photogrammetry
python grid_survey_mission.py --photo_overlap 70 --grid_spacing 25

# Rotated grid for terrain alignment
python grid_survey_mission.py --grid_rotation 45 --pattern parallel
```

**Parameters:**
- `--width`: Survey area width in meters (default: 800)
- `--height`: Survey area height in meters (default: 600)
- `--grid_spacing`: Distance between flight lines (default: 50)
- `--photo_overlap`: Photo overlap percentage (default: 30)
- `--pattern`: `boustrophedon` or `parallel` flight lines
- `--grid_rotation`: Grid rotation in degrees (default: 0)

**Applications:**
- Agricultural field mapping
- Construction site surveys
- Environmental monitoring
- 3D terrain reconstruction

---

### 🎪 Figure-8 Mission (`figure8_mission.py`)

Flies continuous figure-8 (infinity symbol) patterns for surveillance and tracking with smooth parametric motion.

**Key Features:**
- Smooth figure-8 patterns using parametric equations
- Configurable loop dimensions and crossing angles
- Photo capture at pattern crossings or time intervals
- Ultra-smooth velocity-based control with 100Hz update rate
- Automatic crossing detection

**Usage Examples:**
```bash
# Standard figure-8 surveillance
python figure8_mission.py --loop_width 400 --loop_height 250 --loops_count 3

# Photo at every crossing
python figure8_mission.py --photo_at_crossings --crossing_angle 60

# Smooth cinematic pattern
python figure8_mission.py --loop_width 300 --loop_height 200 --speed 2
```

**Parameters:**
- `--loop_width`: Width of each loop in meters (default: 300)
- `--loop_height`: Height of each loop in meters (default: 200)
- `--crossing_angle`: Angle between loops in degrees (default: 45)
- `--loops_count`: Number of complete figure-8 patterns (default: 3)
- `--photo_at_crossings`: Take photos at pattern crossings
- `--photo_interval`: Time interval between photos in seconds (default: 5.0)

**Applications:**
- Traffic monitoring at intersections
- Wildlife behavior observation
- Security patrol with overlapping coverage
- Demonstration of smooth curved flight paths

---

### 🌊 Wave Pattern Mission (`wave_mission.py`)

Flies sinusoidal wave patterns for smooth area coverage with configurable wave parameters and direction.

**Key Features:**
- Beautiful sinusoidal wave patterns across defined areas
- Horizontal or vertical wave directions
- Configurable amplitude and frequency
- Return passes (there and back)
- Coverage efficiency calculations
- 50Hz control for ultra-smooth curves

**Usage Examples:**
```bash
# Gentle coastal-style waves
python wave_mission.py --wave_amplitude 30 --wave_frequency 4 --area_width 600

# Vertical waves with return pass
python wave_mission.py --wave_direction vertical --return_passes 2

# High-frequency small waves
python wave_mission.py --wave_amplitude 15 --wave_frequency 6
```

**Parameters:**
- `--area_width`: Total area width in meters (default: 800)
- `--area_height`: Total area height in meters (default: 400)
- `--wave_amplitude`: Height of wave peaks in meters (default: 50)
- `--wave_frequency`: Number of complete waves across area (default: 3)
- `--wave_direction`: Wave orientation - `horizontal` or `vertical` (default: horizontal)
- `--return_passes`: Number of passes (1=single, 2=there and back) (default: 1)
- `--photo_interval`: Time interval between photos in seconds (default: 3.0)

**Applications:**
- Coastal erosion monitoring
- River/shoreline surveys
- Agricultural field inspection following contours
- Demonstration of smooth curved autonomous flight

---

### ⭐ Star Pattern Mission (`star_pattern_mission.py`)

Flies radiating spoke patterns from a central point for systematic area coverage with multiple flight pattern options.

**Key Features:**
- Radiating spokes from central point (like compass directions)
- Multiple pattern modes: sequential, random, alternating, crisscross
- Configurable return-to-center behavior
- Hover and photography at endpoints
- Comprehensive coverage with compass direction tracking

**Usage Examples:**
```bash
# 8-spoke sequential pattern
python star_pattern_mission.py --num_spokes 8 --spoke_length 400

# Random pattern with endpoint hovering
python star_pattern_mission.py --spoke_pattern random --hover_at_endpoints --return_to_center

# 12-spoke crisscross pattern
python star_pattern_mission.py --num_spokes 12 --spoke_pattern crisscross
```

**Parameters:**
- `--num_spokes`: Number of radiating spokes (default: 8)
- `--spoke_length`: Length of each spoke in meters (default: 400)
- `--spoke_pattern`: Pattern order - `sequential`, `random`, `alternating`, `crisscross` (default: sequential)
- `--return_to_center`: Return to center between each spoke
- `--hover_at_endpoints`: Hover at the end of each spoke
- `--hover_time`: Time to hover at endpoints in seconds (default: 3.0)
- `--photo_interval`: Time interval between photos in seconds (default: 2.0)

**Applications:**
- Emergency response (radiating search from last known location)
- Wildlife tracking from den/nest locations
- Inspection of features radiating from central point
- Testing precise return-to-center navigation

## 🏗️ Architecture

### GenericMission Base Class

The framework uses the Template Method design pattern with `GenericMission` as the abstract base class:

```python
class GenericMission(ABC):
    def run_mission(self):
        # Template method defining mission workflow
        self._initialize_mission()
        self.print_mission_parameters()
        self._connect_and_setup()
        self._execute_mission_workflow()
        self._generate_mission_report()
    
    @abstractmethod
    def execute_mission_logic(self, target_z):
        # Implemented by subclasses
        pass
```

### Mission Workflow

1. **Initialization**: Setup tracking variables and timers
2. **Parameter Display**: Show mission-specific parameters
3. **AirSim Connection**: Connect and enable API control
4. **Mission Execution**: Takeoff → Climb → Execute → Return → Land
5. **Analytics Report**: Generate comprehensive post-mission analysis

### Custom Mission Development

To create a new mission type:

```python
from generic_mission import GenericMission

class MyCustomMission(GenericMission):
    def print_mission_parameters(self):
        # Display mission parameters
        pass
    
    def execute_mission_logic(self, target_z):
        # Implement your mission pattern
        pass
    
    def get_planned_distance(self):
        # Return expected distance for efficiency calculation
        pass
    
    def get_mission_specific_metrics(self):
        # Return mission-specific metrics dictionary
        pass
```

## 📊 Mission Analytics

### Real-time Tracking
- ✅ 3D distance calculation
- ⏱️ Phase-by-phase timing
- 📸 Photo capture tracking
- ❌ Error logging and recovery

### Post-Mission Report
Every mission generates a comprehensive analysis including:

- **Mission Overview**: Status, duration, mission type
- **Performance Metrics**: Planned vs actual distance, efficiency, speed
- **Phase Breakdown**: Detailed timing for each mission segment
- **Quality Assessment**: 100-point scoring system with letter grades
- **Error Report**: Complete error log with timestamps
- **Recommendations**: Actionable feedback for improvement

### Quality Scoring System
- **Mission Success**: 40 points
- **Flight Efficiency**: 30 points (based on planned vs actual distance)
- **Error Penalty**: -10 points per error
- **Safe Landing**: 30 points

**Grade Scale**: A+ (95+), A (90+), B+ (80+), B (70+), C (60+), D (<60)

## 🛡️ Safety Features

### Emergency Procedures
- **Keyboard Interrupt Handling**: Ctrl+C triggers emergency landing
- **Progressive Error Recovery**: Continue mission despite component failures
- **Multiple Landing Fallbacks**: Normal → Hover → Manual descent
- **Position Verification**: Continuous accuracy monitoring

### Error Handling
- **Non-blocking Errors**: Photography failures don't abort missions
- **Comprehensive Logging**: All errors recorded with context
- **Graceful Degradation**: Mission continues with reduced functionality

## 📸 Photography System

### Automatic Photo Capture
- **Time-based Intervals**: Configurable photo timing
- **Position-based Triggers**: Photos at specific locations
- **Organized Storage**: Mission-specific directory structure
- **Error Recovery**: Continues mission if photo capture fails

### Photo Organization
```
photos/
├── box_mission_photos/
│   ├── vertex_1_photo_1_20240101_120000_123.png
│   └── vertex_2_photo_1_20240101_120030_456.png
├── spiral_search_photos/
│   ├── spiral_outward_20240101_130000_789.png
│   └── spiral_outward_20240101_130002_012.png
├── grid_survey_photos/
│   ├── survey_boustrophedon_20240101_140000_345.png
│   └── survey_boustrophedon_20240101_140003_678.png
├── figure8_mission_photos/
│   ├── loop_1_crossing_1_20240101_150000_901.png
│   └── loop_2_interval_20240101_150035_234.png
├── wave_mission_photos/
│   ├── pass_1_forward_20240101_160000_567.png
│   └── pass_2_reverse_20240101_160045_890.png
└── star_mission_photos/
    ├── spoke_1_endpoint_20240101_170000_123.png
    └── spoke_2_outward_20240101_170020_456.png
```

## 🔧 Configuration

### Default Parameters
```python
# Common parameters across all missions
altitude = 20        # Flight altitude in meters
speed = 5           # Flight speed in m/s
preview = False     # Preview mode (no actual flight)

# Mission-specific parameters vary by mission type
```

### Command Line Examples
```bash
# High-altitude long-range mission
python spiral_search_mission.py --altitude 50 --max_radius 1000 --speed 8

# Detailed inspection mission
python box_mission.py --altitude 10 --box_size 200 --orbit_radius 25 --photos_per_orbit 16

# Professional mapping survey
python grid_survey_mission.py --photo_overlap 80 --grid_spacing 20 --grid_rotation 30

# Smooth surveillance pattern
python figure8_mission.py --loop_width 500 --loop_height 300 --loops_count 5 --photo_at_crossings

# Cinematic wave coverage
python wave_mission.py --wave_amplitude 40 --wave_frequency 5 --return_passes 2 --speed 3

# Emergency response search
python star_pattern_mission.py --num_spokes 12 --spoke_length 600 --spoke_pattern alternating --return_to_center
```

## 🐛 Troubleshooting

### Common Issues

**"Connection failed"**
- Ensure AirSim/Unreal Engine is running
- Check that multirotor is selected in AirSim settings
- Verify no firewall blocking connection
- Try running `python test_connection.py`

**"Jerky movement"**
- Reduce speed parameter: `--speed 3`
- Increase photo intervals: `--photo_interval 3.0`
- Check system performance (CPU/GPU load)

**"Mission not completing"**
- Check error log in mission report
- Verify sufficient battery/fuel in simulation
- Ensure adequate flight area size

**"Photos not saving"**
- Check disk space availability
- Verify write permissions in current directory
- Ensure AirSim camera configuration is correct

### Debug Mode
Add debugging to any mission:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 👥 Collaboration Guide

### For Team Members
1. **Clone the repository** and run the setup script (see installation above)
2. **Always activate the virtual environment** before working:
   ```bash
   # Windows: airsim_env\Scripts\activate
   # macOS/Linux: source airsim_env/bin/activate
   ```
3. **Test your setup** with `python test_connection.py` before making changes
4. **Create feature branches** for new mission types or improvements
5. **Share mission results** by including the generated analytics reports

### Environment Notes
- The `airsim_env` folder is **excluded from Git** (in .gitignore)
- Each collaborator needs to run `setup_airsim_env.py` independently
- If you get import errors, ensure the virtual environment is activated
- The setup script handles all complex dependencies automatically

### Development Workflow
```bash
# 1. Activate environment
source airsim_env/bin/activate  # or airsim_env\Scripts\activate on Windows

# 2. Navigate to mission scripts
cd multirotor/mission_planning

# 3. Test existing missions
python box_mission.py --preview

# 4. Develop your changes
# ... your work here ...

# 5. Test thoroughly before committing
python test_connection.py
```

## 🤝 Contributing

### Adding New Mission Types
1. Inherit from `GenericMission`
2. Implement the four abstract methods
3. Add command-line argument parsing
4. Include comprehensive documentation

### Code Style
- Follow PEP 8 Python style guidelines
- Include docstrings for all methods
- Use meaningful variable names
- Add comprehensive error handling

## 📄 License

This project is part of Microsoft AirSim and follows the same MIT license terms.

## 🆘 Support

- **AirSim Documentation**: https://github.com/microsoft/AirSim
- **Issues**: Report bugs via GitHub issues
- **Discussions**: Join AirSim community discussions

## 🎯 Future Enhancements

- **Multi-drone Coordination**: Swarm mission capabilities
- **Advanced Path Planning**: Obstacle avoidance and optimization
- **Real-time Mission Modification**: Dynamic parameter adjustment
- **Machine Learning Integration**: Intelligent mission planning
- **Advanced Analytics**: Predictive performance modeling
- **Additional Pattern Types**: Vortex 3D spirals, multi-point inspection, path following
- **Waypoint Import**: GPS track and KML file support
- **Formation Flight**: Coordinated multi-drone patterns

---

*Happy Flying! 🚁✈️* 