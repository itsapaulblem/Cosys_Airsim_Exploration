# Paul's 6-Months Internship Project, PaulSys: Enhancing the CoSys-AirSim Simulator with AI-Driven Autonomy & Physics-Based Effects

![Project Status](https://img.shields.io/badge/Internship-6_Months_Complete-success) ![ROS2](https://img.shields.io/badge/ROS2-Humble-green) ![AI](https://img.shields.io/badge/AI-YOLO_Vision-orange) ![Architecture](https://img.shields.io/badge/Architecture-Modular-blue)

**📊 Full Presentation Slides**: A [slide deck](https://docs.google.com/presentation/d/1QnXTPNWawfn-A6RgIhcmu6294ree_dILD_hp1z6UNZY/edit?usp=sharing) delivered to the M&S internal team, covering the system architecture, implementation and key results

---

## 📋 Table of Contents

1. [Internship Project Overview](#-internship-project-overview)
2. [The Complete Project Plan](#-the-complete-project-plan)
3. [Internship Project Achievements](#-internship-project-achievements)
4. [Modular ROS2 Node Architecture](#-modular-ros2-node-architecture)
5. [AI Vision Systems Implementation](#-ai-vision-systems-implementation)
6. [Complete File Structure Guide](#-complete-file-structure-guide)
7. [Physics Engine Enhancements](#-physics-engine-enhancements)
8. [Technical Implementation Deep Dive](#-technical-implementation-deep-dive)
9. [Installation & Deployment](#️-installation--deployment)
10. [Startup Instructions (WSL 2.5.10.0 & Windows 10)](#startup-instructions-wsl-2510--windows-10)
11. [Architecture Overview](#architecture-overview)
12. [ROS Topics and Services](#ros-topics-and-services)
13. [Detailed Comparisons: Old vs New Architecture](#detailed-comparisons-old-vs-new-architecture)
14. [Design Decisions](#design-decisions)
15. [Troubleshooting & FAQ](#troubleshooting--faq)
16. [Internship Project Conclusion](#-internship-project-conclusion)
17. [Contact & Acknowledgments](#-contact--acknowledgments)
18. [License & Citation](#-license--citation)
19. [References](#references)

---

## 🎓 Internship Project Overview

This repository documents a **6-month internship project** that transformed Cosys-AirSim from a monolithic single-vehicle system into a **scalable, modular multi-vehicle simulation platform** with **YOLOv7 + DeepSORT AI vision capabilities** and **enhanced physics simulation**. 

### **Original Cosys-AirSim Foundation**

Cosys-AirSim is a simulator for drones, cars and more, with extensive API support, built on [Unreal Engine](https://www.unrealengine.com/). It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for visually realistic simulations.

This fork is based on the last public AirSim release from Microsoft's GitHub. Paulsys-Lab made extensive modifications to the CoSys-AirSim platform to support multiple projects and research goals. The [original AirSim MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) applies to all native AirSim source files.

---
### **🌍 Platform Comparison: Why Cosys-AirSim over Alternatives?**

**AirSim vs CoSys-AirSim:**

| Category                    | AirSim (Microsoft)                                         | CoSys-AirSim (CoSys-Lab)                                                    |
|-----------------------------|------------------------------------------------------------|-----------------------------------------------------------------------------|
| **Base Engine Support**     | Primarily supports Unreal Engine 4.27                      | Supports Unreal Engine 5                                                    |
| **Sensors Modalities**      | Standard sensors: RGB, Depth segmentation, LiDAR, IMU, GPS | Standard sensors + thermal, stereo, multi-camera and expanded LiDAR options |
| **ROS / ROS 2 Integration** | ROS 1 officially supported                                 | Includes ROS 2 workspace support                                            |

**Unity vs Unreal Engine:**

| Criteria                  | Unity Robotics Hub & ROS-TCP Connector                                  | Cosys-AirSim                                                                        |
|---------------------------|-------------------------------------------------------------------------|-------------------------------------------------------------------------------------|
| **ROS Compatibility**     | Primarily supports ROS 1; Limited ROS 2 support                         | Full ROS 2 Support                                                                  |
| **Drone Control Loop**    | Physics simulated but loop latency via TCP limits true in-loop feedback | Real-time physics-in-the-loop (SITL/HITL) with PX4 for accurate drone dynamics      |
| **Maintenance & Updates** | Deprecated (last updated Feb 2022)                                      | Maintained and regularly updated                                                    |
| **Community Support**     | Minimal support; Inactive forum                                         | Strong community and active development                                             |

### **Internship Project Transformation**

The original Cosys-AirSim ROS 2 implementation was a monolithic system that:
- ❌ **Non-scalable architecture**: Single [`airsim_ros_wrapper.cpp`](ros2/src/airsim_ros_pkgs/src/airsim_ros_wrapper.cpp) handled all vehicles in one process, violating ROS 2 best practices for distributed systems
- ❌ **Critical single points of failure**: One vehicle crash or RPC timeout would terminate the entire fleet operation
- ❌ **Lack of coordinated fleet management**: No centralized command structure for synchronized multi-vehicle operations 
- ❌ **Limited vehicle type extensibility**: The monolithic design made adding new drones (/drone1, /drone2...) extremely difficult and non-scalable, often requiring changes across 2,000+ lines of code.
- ❌ **Complete absence of AI vision capabilities**: No object detection, tracking, or autonomous behavior systems
- ❌ **Non-functional physics effects**: F10 weather menu was purely cosmetic - rain, snow, and wind had zero impact on actual drone flight dynamics

### **6-Month Solution Delivered**

A complete architectural transformation resulting in:
- ✅ **Modular Multi-Vehicle Architecture**: Independent [`multirotor_node.cpp`](ros2/src/airsim_ros_pkgs/src/multirotor_node.cpp) instances with [`VehicleNodeBase`](ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/VehicleNodeBase.hpp) inheritance hierarchy, enabling vehicle scalability
- ✅ **Fleet Coordination Platform**: Centralized [`coordination_node.cpp`](ros2/src/airsim_ros_pkgs/src/coordination_node.cpp) providing synchronized operations (`/takeoff_all`, `/land_all`, `/search_target_all`) for drone swarm management
- ✅ **Production-Ready Reliability**: Per-vehicle fault isolation with independent RPC connections, preventing cascading failures across the fleet
- ✅ **AI-Powered Vision Systems**: YOLOv7 + DeepSORT integration via [`motion_detection_node.py`](ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py) and [`generalised_object_tracking_node.py`](ros2/src/airsim_ros_pkgs/scripts/generalised_object_tracking_node.py) for real-time object detection and autonomous tracking
- ✅ **Functional Physics Integration**: F10 weather menu now drives real flight dynamics through [`WeatherPhysicsBridge.cpp`](Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.cpp) and [`FastPhysicsEngine.hpp`](AirLib/include/physics/FastPhysicsEngine.hpp) integration

---

### **🔤 Basic Technologies Overview**

**🤖 ROS 2 (Robot Operating System 2)** - Distributed robotics middleware enabling communication between software components

**👁️ YOLO (You Only Look Once) v7** - Real-time object detection AI that identifies and locates multiple objects in a single image pass

**🎯 DeepSORT** - Multi-object tracking system maintaining consistent object identities across video frames

**🛩️ Flight Controller Integration**

**PX4 Autopilot**
- **Flight Control Core** aka the "brain" of the drone
- **Open source flight control software** for drones and other unmanned vehicles
- **Integrates with MAVLink** as its communication interface and with ground control stations for mission planning

**Micro Air Vehicle Link Protocol (MAVLink)**
- **Lightweight, open source communication protocol** for drones
- **Supports both streaming data (telemetry) and direct commands** (mission updates)
- **Mostly used between QGroundControl and PX4**

**🌍 Cosys-AirSim Platform** - Enhanced simulation environment with Unreal Engine 5 support and expanded sensor capabilities

*Detailed explanations of each technology are provided in their respective sections below.*

---

## 🎯 The Complete Project Plan

**Tasks Completed During 6-Month Internship:**

1. **🌍 Virtual Environment Setup**
   - Initial exploration of Cesium for real-world 3D environments
   - Resolution of segmentation camera limitations with Cesium 3D Tiles format
   - Migration to prebuilt Unreal environments (RuralAustralia) with EO/RGB cameras

2. **🚁 Drone Configuration & Multi-Vehicle Architecture**
   - Transformation from monolithic to modular ROS2 architecture
   - Implementation of VehicleNodeBase inheritance hierarchy
   - Development of coordination_node.cpp for fleet management
   - Individual multirotor_node.cpp instances with fault isolation

3. **🤖 ROS 2 Integration & Refactoring**
   - Migration from shared RPC pool to dedicated RPC clients
   - Implementation of parallel timer loops for simultaneous processing
   - Dedicated namespaces (/drone0, /drone1, etc.)
   - Service, topic, and parameter architecture design

4. **🔧 Required Tools & Methods Implementation**
   - YOLOv7 + DeepSORT integration for object detection and tracking
   - Multi-camera fusion system (4 cameras per drone)
   - Confidence scoring algorithms for target selection
   - Real-time image processing pipeline

5. **🎮 Behaviour & Logic Loop Design**
   - 4-state autonomous behavior system (Idle → Taking Off → Hovering → Following)
   - Multi-directional movement control based on camera detection
   - Distance maintenance and target handoff between cameras

6. **🌪️ F10 Weather Physics Integration**
   - Transformation of purely cosmetic F10 weather menu into functional physics effects
   - WeatherPhysicsBridge.cpp implementation connecting Unreal weather sliders to AirLib
   - FastPhysicsEngine.hpp integration for real-time weather force calculations
   - Dynamic weather effects: rain downward pressure, snow turbulence, dust drag increases
   

## 🏆 Internship Project Achievements

### **🏗️ Architecture Redesign: Monolithic → Modular**

| **Component**        | **Before (Monolithic)**         | **After (Modular)**                             | **Impact**                             |
|----------------------|---------------------------------|-------------------------------------------------|----------------------------------------|
| **ROS 2 Structure**  | Single `airsim_ros_wrapper.cpp` | `coordination_node.cpp` + `multirotor_node.cpp` | Scalable architecture                  |
| **Vehicle Control**  | Shared processing               | Independent nodes per vehicle                   | Fault isolation                        |
| **Fleet Management** | Individual commands             | Coordinated operations                          | Synchronized control                   |
| **AI Capabilities**  | None                            | YOLOv7 + object tracking                        | Autonomous search and track operations |
| **Physics Effects**  | F10 menu decorative only        | Real weather dynamics                           | Realistic simulation                   |

## 🚁 Modular ROS2 Node Architecture

### **Coordination Node**
**📁 File Location**: `ros2/src/airsim_ros_pkgs/src/coordination_node.cpp`  
**📁 Header File**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/coordination_node.hpp`

**Global Fleet Command & Control Hub**

```cpp
class CoordinationNode : public Node {
    // Synchronized fleet operations
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_all_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_all_service_;  
    rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_all_service_;
    
    // System monitoring
    rclcpp::Service<airsim_interfaces::srv::ListSceneObjectTags>::SharedPtr health_check_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
};
```

**Global Services Provided**:
- 🚀 `/takeoff_all` - Synchronized fleet takeoff with physics validation
- 🛬 `/land_all` - Coordinated fleet landing sequence
- 🔄 `/reset_all` - Global system reset across all vehicles
- ⏸️ `/pause_simulation` - Physics simulation pause/resume control
- 🏥 `/armed_check` - Fleet-wide readiness monitoring

### **Multirotor Node**
**📁 File Location**: `ros2/src/airsim_ros_pkgs/src/multirotor_node.cpp`  
**📁 Header File**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/multirotor_node.hpp`  
**📁 Base Class**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/VehicleNodeBase.hpp`

**Individual Vehicle Control**

```cpp
class MultirotorNode : public VehicleNodeBase {
    // Advanced flight services
    rclcpp::Service<airsim_interfaces::srv::SearchTarget>::SharedPtr search_target_srv_;
    rclcpp::Service<airsim_interfaces::srv::TrackTarget>::SharedPtr track_target_srv_;
    rclcpp::Service<airsim_interfaces::srv::GpsWaypoint>::SharedPtr gps_waypoint_srv_;
    
    // Motion target tracking with weather compensation
    MotionTarget current_motion_target_;
    std::mutex motion_target_mutex_;
};
```

**Individual Vehicle Services**:
- 🛫 `/droneX/takeoff` - Weather-aware takeoff with safety checks
- 🛬 `/droneX/land` - Smart landing with ground effect compensation  

## 🤖 AI Vision Systems Implementation

### **👁️ YOLO (You Only Look Once) v7 - Object Detection AI**

YOLO is a **real-time object detection neural network** that can identify and locate multiple objects in images incredibly fast.

**What is YOLOv7?**
- Stands for "**You Only Look Once**" - Processes the whole image in one pass to detect all objects
- **Improvement over older models** that scan multiple times
- **Used for Object Detection** of 80+ COCO dataset classes

**How YOLO Works:**
1. **Single Pass Detection**: Unlike traditional methods that scan images multiple times, YOLO analyzes the entire image once
2. **Grid Division**: Divides image into a grid (e.g., 13x13 cells)
3. **Simultaneous Prediction**: Each grid cell predicts objects and their confidence scores
4. **Bounding Box Regression**: Precisely locates objects with rectangular boxes

**YOLOv7 Decision Making - Confidence Scoring Formula:**

**📁 Implementation**: `ros2/src/airsim_ros_pkgs/scripts/generalised_object_tracking_node.py` 

Function: `calculate_target_confidence()`

```
Confidence level = detection_confidence × size_factor × center_factor
                  × consistency_factor × camera_priority_factor
```

**AI Deciding Factors:**
- **detection_confidence**: YOLOv7 AI confidence (0.08-1.0)

**Developer Deciding Factors:**
- **size_factor**: Larger bounding box indicates closer person
- **center_factor**: Prioritizes targets near the image center
- **consistency_factor**: Rewards existing track IDs
- **camera_priority**: Prefers front camera (×1.2)

**In Our Project:**
```python
# YOLO detects these objects in real-time:
- Person (class 0)    → Person following drone
- Car (class 2)       → Vehicle tracking  
- Airplane (class 4)  → Drone-to-drone tracking
- 80+ COCO classes    → Any object you want to track
```

---

### **🎯 DeepSORT - Multi-Object Tracking System**

DeepSORT **tracks multiple objects across video frames**, maintaining consistent identities even when objects move, overlap, or temporarily disappear.

**DeepSORT Algorithm: Object Tracking with IDs**
- "**Deep Simple Online and Realtime Tracking**"
- Uses **object appearance (via a CNN) and motion (Kalman filter)** to track objects across frames
- **Gives each object a unique ID** to maintain identity even during movement
- **Keeps consistent tracking** of the detected target over time

**How DeepSORT Works:**
1. **Object Association**: Links detected objects across frames using appearance features
2. **Kalman Filtering**: Predicts where objects will move next
3. **Deep Learning Features**: Uses CNN to distinguish between different objects
4. **Track Management**: Maintains object histories and handles occlusions

**Tracking Pipeline:**
```python
Frame 1: Person detected → Track ID #1 assigned
Frame 2: Person moves → Track ID #1 updated with new position  
Frame 3: Person hidden → Track ID #1 maintained (predicted position)
Frame 4: Person reappears → Track ID #1 reconfirmed (same person!)
```

**Key Benefits:**
- **Identity Persistence**: Same object keeps same ID across frames
- **Occlusion Handling**: Tracks objects even when temporarily hidden
- **Multi-Target**: Simultaneously tracks multiple people/objects
- **Robust Association**: Distinguishes between similar-looking objects

---

### **🔄 Complete AI Integration Pipeline**

**YOLOv7 Detection Pipeline:**
```
📷 INPUT: 4 Camera Streams (1280 x 720, 20Hz)
    ↓
🎯 PER CAMERA FEED:
1. Image Conversion (RGB → BGR)
2. High-quality resize with interpolation
3. YOLO detects based on 80 COCO classes
4. DeepSORT assigns persistent IDs to entities
5. Filters for Person (Class ID = 0)
    ↓
🔄 MULTI CAMERA FUSION:
1. Transform Camera Coordinates to World Coordinates
2. Merge Detections from 4 Cameras
3. Select target with highest confidence above threshold
4. Send Velocity command via ROS2
5. Track handoff between cameras
    ↓
📤 OUTPUT: Target detection with world coordinates
```

**YOLOv7 DeepSORT - ROS 2 Integration:**
- Uses the refactored **MultirotorNode** from the multi-node ROS 2 architecture
- Includes a new **MotionDetectionNode** for image-based movement analysis

**Sensor-Instruction Feedback Loop:**
- **MultirotorNode**: Publishes 4 camera topics per drone
- **MotionDetectionNode**: Subscribes to camera images and publishes velocity commands
- **MultirotorNode**: Subscribes to these velocity commands for motion control

**4-State Behavior System:**

```
🔄 STATE 1: IDLE
Condition: Beginning state, no person detected
Behaviour: 
- Drone on ground
- All 4 Cameras Scanning
- YOLOv7 running continuously
        ↓ [Person Found]
🚀 STATE 2: Taking Off  
Condition: Person Detected, Takeoff Initiated
Behaviour:
- Call /drone1/takeoff service
- Wait for takeoff completion
- Continue person tracking
        ↓ [Airborne]
🚁 STATE 3: Hovering
Condition: Airborne but no current target lock
Behaviour:
- Phase 1 (0-20s): Move in last direction
- Phase 2 (20s+): Stay stationary
        ↓ [Person Found] / ↑ [Person Lost]
🎯 STATE 4: Following
Condition: Person actively tracked by cameras
Behaviour:
- Multi-directional movement
- Distance maintenance (follow_distance)
- Camera based directional control
```

---

### **🌪️ Working Weather System**

**Problem Solved**: The F10 weather menu in original Cosys-AirSim was purely visual - rain, snow, and wind effects had **zero impact** on drone flight dynamics.

**Weather Effects Integration**  
**📁 Core Physics**: `AirLib/include/physics/WeatherPhysics.hpp` + `AirLib/src/physics/WeatherPhysics.cpp`  
**📁 Unreal Bridge**: `Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.h` + `WeatherPhysicsBridge.cpp`  
**📁 Engine Integration**: `AirLib/include/physics/FastPhysicsEngine.hpp` + `AirLib/src/physics/FastPhysicsEngine.cpp`

**FastPhysicsEngine Enhancements**  
**📁 Header File**: `AirLib/include/physics/FastPhysicsEngine.hpp`  
**📁 Implementation**: `AirLib/src/physics/FastPhysicsEngine.cpp`  
**📁 Base Class**: `AirLib/include/physics/PhysicsEngineBase.hpp`
- Weather force integration in real-time flight dynamics
- Enhanced drag calculations with atmospheric effects
- Ground-lock improvements for stable landing
- Collision response optimization

**Solution Implemented**:
```cpp
// WeatherPhysicsBridge.cpp - Unreal to AirLib integration
FWeatherForces UWeatherPhysicsBridge::GetWeatherForces(UWorld* World, FVector Position, FVector Velocity, float DeltaTime) {
    // Get real F10 slider values
    float rain_intensity = GetWeatherSliderValue(World, WEATHER_PARAM_SCALAR_RAIN);
    float snow_intensity = GetWeatherSliderValue(World, WEATHER_PARAM_SCALAR_SNOW); 
    float dust_intensity = GetWeatherSliderValue(World, WEATHER_PARAM_SCALAR_DUST);
    
    // Calculate real physics forces
    if (rain_intensity > 0.0f) {
        forces.TurbulenceForce.Z += rain_intensity * 250.0f; // Downward pressure
        forces.DragMultiplier = 1.0f + (rain_intensity * 0.3f); // Increased drag
    }
    
    return forces; // Applied to actual flight dynamics
}
```

**FastPhysicsEngine Integration** (`FastPhysicsEngine.hpp`):
```cpp
// Real-time weather effects in physics calculations  
static void getNextKinematicsNoCollision(TTimeDelta dt, PhysicsBody& body, /* ... */) {
    // Add weather turbulence and wind effects
    Wrench weather_wrench = Wrench::zero();
    if (current_weather_forces_.has_effects) {
        weather_wrench.force = current_weather_forces_.turbulence_force + current_weather_forces_.wind_gust;
    }
    
    next_wrench = body_wrench + drag_wrench + ext_force_wrench + weather_wrench;
    // Weather now affects actual drone movement!
}
```

**Real Weather Effects**:
- 🌧️ **Rain**: Downward force + increased drag (drones struggle to stay airborne)
- ❄️ **Snow**: Swirling turbulence + reduced visibility effects  
- 💨 **Dust**: Strong chaotic turbulence + significant drag increase
- 🌫️ **Fog**: Subtle atmospheric effects + air density changes
- 🍂 **Falling Leaves**: Gentle environmental turbulence

---

## 📂 Complete File Structure Guide

### **Implementation Files Overview**

```
Cosys-AirSim/
├── ros2/src/airsim_ros_pkgs/
│   ├── src/                           # Core C++ ROS2 nodes
│   │   ├── coordination_node.cpp      # Fleet coordination hub
│   │   ├── multirotor_node.cpp        # Individual vehicle control
│   │   ├── VehicleNodeBase.cpp        # Base vehicle functionality
│   │   └── (other vehicle nodes)
│   ├── include/airsim_ros_pkgs/       # C++ header files
│   │   ├── coordination_node.hpp
│   │   ├── multirotor_node.hpp  
│   │   ├── VehicleNodeBase.hpp
│   │   └── (matching headers)
│   ├── scripts/                       # Python AI vision nodes
│   │   ├── motion_detection_node.py   # Person following AI
│   │   ├── generalised_object_tracking_node.py  # Multi-object AI
│   │   └── (AI support scripts)
│   ├── launch/                        # ROS2 launch configurations
│   │   ├── multi_drone.launch.py      # Multi-vehicle launcher
│   │   ├── motion_detection_launch.py  # Person following launcher
│   │   ├── generalised_object_tracking_launch.py  # Object tracking
│   │   └── (other launch files)
│   ├── config/                        # Configuration files
│   │   ├── settings_multi_drone.json  # Multi-vehicle settings
│   │   └── (AI model configs)
│   ├── models/                        # AI model weights
│   │   ├── yolov7.pt                  # YOLO detection model
│   │   └── (DeepSORT configs)
│   └── requirements.txt               # Python dependencies
├── AirLib/                           # Core AirSim library
│   ├── include/physics/               # Physics engine headers
│   │   ├── WeatherPhysics.hpp         # Weather force calculations
│   │   ├── FastPhysicsEngine.hpp      # Enhanced physics engine
│   │   └── PhysicsEngineBase.hpp      # Physics base class
│   └── src/physics/                   # Physics implementations  
│       ├── WeatherPhysics.cpp         # Weather effects
│       ├── FastPhysicsEngine.cpp      # Physics engine
│       └── PhysicsEngineBase.cpp      # Base physics
├── Unreal/Plugins/AirSim/Source/     # Unreal Engine integration
│   ├── WeatherPhysicsBridge.h         # Unreal ↔ AirLib interface
│   ├── WeatherPhysicsBridge.cpp       # F10 weather implementation
│   └── Weather/
│       └── WeatherLib.h               # Unreal weather system
└── PythonClient/                     # Python API examples
    └── multirotor/
        └── generate_settings.py       # Multi-vehicle config generator
```

### **Key Configuration Files**

```
📁 settings.json location: Documents/AirSim/settings.json 
📁 YOLO model weights: ros2/src/airsim_ros_pkgs/models/
📁 Launch configurations: ros2/src/airsim_ros_pkgs/launch/
```

---

## ⚡ Physics Engine Enhancements

### **🔄 How All Systems Work Together**

```
📷 CAMERA FEED
    ↓
🧠 YOLO DETECTION ("I see a person at coordinates X,Y")  
    ↓
🎯 DEEPSORT TRACKING ("This is person #3, they moved from last frame")
    ↓  
📡 ROS 2 TOPIC (/target_detection)
    ↓
🚁 DRONE CONTROL ("Fly towards person #3's predicted position")
    ↓
⚖️ PHYSICS ENGINE ("Apply weather effects to flight")
```

**Real-World Example:**
1. **YOLO** detects person in drone's front camera
2. **DeepSORT** assigns Track ID #1 and remembers their appearance  
3. **ROS 2 Topic** publishes target data to flight control node
4. **ROS 2 Service** commands drone to start following
5. **Weather Physics** applies rain effects to make following more challenging
6. Person moves to different camera → **YOLO** detects in side camera
7. **DeepSORT** confirms same person (Track ID #1) 
8. **ROS 2 Parameters** adjust following distance dynamically
9. Process repeats 30 times per second for real-time tracking!

This integrated system enables **fully autonomous person following** with **realistic physics simulation** - exactly what this internship project achieved! 🎯

## 🔧 Technical Implementation Deep Dive

### **📁 Implementation Files Quick Reference**

| **Component** | **Primary File** | **Supporting Files** |
|---------------|------------------|---------------------|
| **Fleet Coordination** | `ros2/src/airsim_ros_pkgs/src/coordination_node.cpp` | `coordination_node.hpp`, `multi_drone.launch.py` |
| **Vehicle Control** | `ros2/src/airsim_ros_pkgs/src/multirotor_node.cpp` | `VehicleNodeBase.hpp`, `multirotor_node.hpp` |
| **Person Following** | `ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py` | `motion_detection_launch.py`, `requirements.txt` |
| **Object Tracking** | `ros2/src/airsim_ros_pkgs/scripts/generalised_object_tracking_node.py` | `generalised_object_tracking_launch.py` |
| **Weather Physics** | `AirLib/include/physics/WeatherPhysics.hpp` | `WeatherPhysics.cpp`, `WeatherPhysicsBridge.cpp` |
| **Physics Engine** | `AirLib/include/physics/FastPhysicsEngine.hpp` | `FastPhysicsEngine.cpp`, `PhysicsEngineBase.hpp` |
| **Unreal Bridge** | `Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.cpp` | `WeatherPhysicsBridge.h`, `WeatherLib.h` |

---

## 🤖 **1. ROS2 Modular Architecture Implementation**

#### **Before: Monolithic `airsim_ros_wrapper.cpp`**
```cpp
// OLD ARCHITECTURE: Single monolithic class handling ALL vehicles
class AirsimROSWrapper {
private:
    // Single RPC client for ALL vehicles - bottleneck!
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    
    // All vehicles share same processing thread - no parallelism
    void airsim_control_update_timer_cb() {
        for (auto& vehicle_name : vehicle_names_) {
            // Sequential processing - blocking operations
            auto state = airsim_client_->getMultirotorState(vehicle_name);
            // Process vehicle 1, then 2, then 3... (sequential bottleneck)
        }
    }
    
    // Single point of failure - one vehicle crash kills entire fleet
    // No fault isolation, no parallel processing, no scalability
};
```

#### **After: Modular Architecture with Inheritance**

**Base Class**  
**📁 Header**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/VehicleNodeBase.hpp`  
**📁 Implementation**: `ros2/src/airsim_ros_pkgs/src/VehicleNodeBase.cpp`

```cpp
// NEW ARCHITECTURE: Modular inheritance-based system
class VehicleNodeBase : public rclcpp::Node {
protected:
    // Each vehicle gets its own isolated RPC connection
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_;
    
    // CRITICAL: Global RPC mutex prevents simultaneous AirSim calls
    // Fixes race condition crashes when multiple vehicles call AirSim API
    static std::mutex airsim_rpc_mutex_;
    
    // Per-vehicle callback groups enable TRUE parallel processing
    rclcpp::CallbackGroup::SharedPtr sensor_group_;  // High-frequency sensor data
    rclcpp::CallbackGroup::SharedPtr control_group_; // Flight commands
    
    // Extensible sensor publishing framework
    virtual void setup_vehicle_publishers() = 0;
    virtual void setup_vehicle_services() = 0;
    virtual nav_msgs::msg::Odometry get_vehicle_odometry() = 0;
    
    // Settings-driven sensor configuration (dynamic sensor discovery)
    void create_sensor_publishers() {
        for (const auto& sensor_config : vehicle_sensors_) {
            if (sensor_config.type == "Imu") {
                create_imu_publisher(sensor_config.name);
            }
            // Extensible for any sensor type defined in settings.json
        }
    }
};
```

**Multirotor Implementation**  
**📁 Source**: `ros2/src/airsim_ros_pkgs/src/multirotor_node.cpp`  
**📁 Header**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/multirotor_node.hpp`

```cpp
class MultirotorNode : public VehicleNodeBase {
private:
    // DUAL CLIENT ARCHITECTURE: Bandwidth optimization
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> sensor_client_;  // High-bandwidth (LiDAR, cameras)
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> state_client_;   // Low-latency (IMU, GPS)
    
public:
    // THREAD-SAFE VEHICLE STATE ACCESS
    nav_msgs::msg::Odometry get_vehicle_odometry() override {
        // GLOBAL RPC MUTEX: Prevents simultaneous AirSim API calls
        decltype(state_client_->getMultirotorState(vehicle_name_)) drone_state;
        {
            std::lock_guard<std::mutex> rpc_lock(airsim_rpc_mutex_);
            drone_state = state_client_->getMultirotorState(vehicle_name_);
        }
        return get_odom_from_multirotor_state(drone_state);
    }
    
    // FAULT ISOLATION: Each vehicle has independent sensors and services
    void setup_vehicle_control_services() override {
        std::string topic_prefix = vehicle_name_ + "/";
        
        takeoff_srv_ = this->create_service<airsim_interfaces::srv::Takeoff>(
            topic_prefix + "takeoff", 
            std::bind(&MultirotorNode::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Each vehicle has completely isolated services - no shared state!
        // /Drone1/takeoff, /Drone2/takeoff, /Drone3/takeoff (independent)
    }
};
```

**Fleet Coordination**  
**📁 Source**: `ros2/src/airsim_ros_pkgs/src/coordination_node.cpp`  
**📁 Header**: `ros2/src/airsim_ros_pkgs/include/airsim_ros_pkgs/coordination_node.hpp`

```cpp
class CoordinationNode : public Node {
public:
    // FLEET-WIDE SYNCHRONIZED OPERATIONS
    bool takeoff_all_callback(/* ... */) {
        // Step 1: Enable API control for ALL vehicles
        for (const auto& vehicle_name : vehicle_names_) {
            airsim_client_->enableApiControl(true, vehicle_name);
            airsim_client_->armDisarm(true, vehicle_name);
        }
        
        // Step 2: Simultaneous takeoff using async calls
        std::vector<std::future<bool>> futures;
        for (const auto& vehicle_name : vehicle_names_) {
            auto future = airsim_client_->takeoffAsync(20.0f, vehicle_name);
            futures.push_back(std::async(std::launch::async, [future]() {
                future->waitOnLastTask();
                return true;
            }));
        }
        
        // Step 3: Wait for ALL vehicles to complete
        for (auto& future : futures) {
            future.wait();
        }
        
        return true; // All vehicles successfully coordinated
    }
    
    // AI-POWERED FLEET COORDINATION
    bool search_target_all_callback(/* ... */) {
        // Coordinate AI search across entire fleet
        // Best detection from ANY vehicle becomes fleet target
        float best_confidence = 0.0f;
        std::string best_vehicle;
        
        for (const auto& [vehicle_name, target] : vehicle_targets_) {
            if (target.confidence > best_confidence) {
                best_confidence = target.confidence;
                best_vehicle = vehicle_name;
            }
        }
        
        response->message = "Target found by " + best_vehicle;
        return true;
    }
};
```

---

## 🤖 **2. AI Vision Systems (YOLO + DeepSORT)**

### **Person Following System Implementation**  
**📁 Main Script**: `ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py`  
**📁 Launch File**: `ros2/src/airsim_ros_pkgs/launch/motion_detection_launch.py`  
**📁 Dependencies**: `ros2/src/airsim_ros_pkgs/requirements.txt` (YOLO, OpenCV, etc.)

```python
# Advanced AI capabilities implemented
parameters=[{
    'vehicle_name': LaunchConfiguration('vehicle_name'),
    'confidence_threshold': 0.08,      # AI confidence filtering
    'motion_threshold': 15.0,          # Motion detection sensitivity  
    'enable_following': True,          # Autonomous following mode
    'follow_distance': 0.1,            # Dynamic distance control
    'takeoff_height': 5.0,             # 3D spatial tracking
    'image_width': 1280,               # High-resolution processing
    'image_height': 720,               # Real-time performance
}]
```

**Key Capabilities**:
- 🎯 **Multi-Camera YOLO Detection**: Person detection across 4 drone cameras
- 🚁 **Autonomous Following**: Dynamic 3D position control for person tracking
- 📷 **Motion Analysis**: Pixel-level movement detection for target validation
- ⚡ **Real-time Processing**: 30fps image processing with 1280x720 resolution
- 🛫 **Auto Takeoff**: Automatic takeoff when target detected

### **Generalized Object Tracking System**
**📁 Main Script**: `ros2/src/airsim_ros_pkgs/scripts/generalised_object_tracking_node.py`  
**📁 Launch File**: `ros2/src/airsim_ros_pkgs/launch/generalised_object_tracking_launch.py`  
**📁 YOLO Models**: `ros2/src/airsim_ros_pkgs/models/` (YOLOv7 weights location)

```python
# Multi-class object tracking capabilities
parameters=[{
    'target_class_id': LaunchConfiguration('target_class_id'),  # 0=person, 2=car, 4=drone
    'confidence_threshold': 0.08,      # Adaptive threshold for different objects
    'iou_threshold': 0.45,             # Intersection over Union for tracking
    'trail_length': 30,                # Motion history tracking
    'follow_distance': 3.0,            # Object-specific following distance
}]
```

**Advanced Capabilities**:
- 🎨 **Multi-Class Detection**: Track cars, motorcycles, airplanes, drones, people
- 🔄 **Persistent Tracking**: IoU-based object association across frames
- 📈 **Motion Prediction**: Trail-based movement forecasting  
- 🎛️ **Runtime Configuration**: Change target class without restart
- 📊 **Performance Optimization**: DeepSORT integration for robust tracking

### **2. AI Detection System (YOLO + DeepSORT Integration)**

#### **Multi-Camera Person Following**  
**📁 Main File**: `ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py`  
**📁 Launch Config**: `ros2/src/airsim_ros_pkgs/launch/motion_detection_launch.py`

```python
class MultiCameraMotionDetectionNode(Node):
    def __init__(self):
        # MULTI-CAMERA ARCHITECTURE: 4 simultaneous camera feeds
        self.camera_topics = [
            f'/{self.vehicle_name}/Camera_0_Scene/image',  # Front camera
            f'/{self.vehicle_name}/Camera_1_Scene/image',  # Right camera  
            f'/{self.vehicle_name}/Camera_2_Scene/image',  # Back camera
            f'/{self.vehicle_name}/Camera_3_Scene/image'   # Left camera
        ]
        
        # PER-CAMERA TRACKING DATA STRUCTURES
        for cam_id in range(4):
            self.camera_data_deques[cam_id] = {}     # Track history per camera
            self.camera_locks[cam_id] = threading.Lock()  # Thread safety
            self.all_camera_targets[cam_id] = []     # Detections per camera
            
            # Subscribe to each camera independently
            self.image_subs[cam_id] = self.create_subscription(
                Image, self.camera_topics[cam_id],
                lambda msg, cam_id=cam_id: self.image_callback(msg, cam_id), 10)

    def setup_yolo_deepsort_integration(self):
        """Initialize YOLOv7 + DeepSORT for real-time object detection"""
        # YOLO MODEL LOADING
        self.device = select_device('0')  # GPU acceleration
        self.model = attempt_load('/path/to/yolov7.pt', map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.img_size = check_img_size(640, s=self.stride)
        
        # DEEPSORT TRACKER INITIALIZATION  
        cfg_deep = get_config()
        cfg_deep.merge_from_file('/path/to/deep_sort.yaml')
        self.deepsort = DeepSort(
            cfg_deep.DEEPSORT.REID_CKPT,
            max_dist=cfg_deep.DEEPSORT.MAX_DIST,
            min_confidence=cfg_deep.DEEPSORT.MIN_CONFIDENCE,
            max_iou_distance=cfg_deep.DEEPSORT.MAX_IOU_DISTANCE,
            max_age=cfg_deep.DEEPSORT.MAX_AGE,
            n_init=cfg_deep.DEEPSORT.N_INIT
        )
        
    def detect_and_track_yolo_deepsort(self, image, camera_id):
        """Real-time YOLO detection + DeepSORT tracking per camera"""
        # YOLO INFERENCE
        img = self.letterbox(image, self.img_size, stride=self.stride)[0]
        img = torch.from_numpy(img).to(self.device).half() / 255.0
        
        with torch.no_grad():
            pred = self.model(img)[0]
        
        # NON-MAX SUPPRESSION  
        pred = non_max_suppression(pred, self.conf_threshold, self.iou_threshold, 
                                  classes=[0])  # Person class only
        
        moving_targets = []
        for det in pred:
            if len(det):
                # SCALE COORDINATES back to original image size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], image.shape).round()
                
                # DEEPSORT TRACKING
                xywhs = xyxy2xywh(det[:, 0:4])
                confs = det[:, 4]
                clss = det[:, 5]
                
                outputs = self.deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), image)
                
                # MOTION ANALYSIS: Check if tracked objects are moving
                for output in outputs:
                    track_id = output[4]
                    bbox = output[:4]
                    
                    if self.is_track_moving_camera(track_id, camera_id):
                        # WORLD COORDINATE CONVERSION
                        center = [bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2]
                        world_coords = self.pixel_to_world(center)
                        
                        moving_targets.append({
                            'track_id': track_id,
                            'bbox': bbox.tolist(),
                            'center': center,
                            'world_x': world_coords[0],
                            'world_y': world_coords[1],
                            'world_z': -2.0,  # Assume ground level
                            'confidence': output[5],
                            'camera_id': camera_id
                        })
        
        return moving_targets

    def control_loop(self):
        """Autonomous flight control based on AI vision"""
        if not self.following_active or not hasattr(self, 'target_person_data'):
            return
            
        person = self.target_person_data
        target_camera = person.get('camera_id', 0)
        
        # MULTI-CAMERA MOVEMENT LOGIC
        bbox = person.get('bbox', [0, 0, 100, 100])
        center = person.get('center', [bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2])
        
        # DISTANCE ESTIMATION from bounding box size
        estimated_distance = self.estimate_person_distance(bbox)
        
        # CAMERA-SPECIFIC MOVEMENT COMMANDS
        vel_cmd = VelCmd()
        if target_camera == 0:      # Front camera - forward/backward
            vel_cmd.twist.linear.x = 1.0 if estimated_distance > self.follow_distance else 0.0
        elif target_camera == 1:    # Right camera - sideways right
            vel_cmd.twist.linear.y = -1.0
        elif target_camera == 2:    # Back camera - backward movement  
            vel_cmd.twist.linear.x = -1.0
        elif target_camera == 3:    # Left camera - sideways left
            vel_cmd.twist.linear.y = 1.0
            
        # YAW CONTROL: Turn to face target
        image_center = [self.image_width / 2, self.image_height / 2]
        yaw_angle, _ = self.pixel_to_world_direction(center, image_center)
        vel_cmd.twist.angular.z = yaw_angle * 2.0  # Proportional yaw control
        
        self.cmd_vel_pub.publish(vel_cmd)
```

#### **Generalized Object Tracking**  
**📁 Main File**: `ros2/src/airsim_ros_pkgs/scripts/generalised_object_tracking_node.py`  
**📁 Launch Config**: `ros2/src/airsim_ros_pkgs/launch/generalised_object_tracking_launch.py`

```python
class GeneralisedObjectTrackingNode(Node):
    """Track ANY COCO class object (cars, motorcycles, drones, people)"""
    
    def __init__(self):
        # CONFIGURABLE TARGET CLASS (runtime selection)
        self.declare_parameter('target_class_id', 0)  # 0=person, 2=car, 4=airplane/drone
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.target_class_name = self.names[self.target_class_id]
        
        # COCO DATASET CLASSES (80+ object types)
        self.names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 
                     'train', 'truck', 'boat', 'traffic light', 'fire hydrant', ...]
        
    def select_target_multi_camera(self, merged_targets):
        """Select best target of specified class across all cameras"""
        # FILTER BY TARGET CLASS
        targets = [target for target in merged_targets 
                  if target.get('class', -1) == self.target_class_id]
        
        if not targets:
            return None, None
            
        # TARGET CONFIDENCE SCORING
        best_target = None
        best_confidence = 0.0
        
        for target in targets:
            camera_id = target.get('camera_id')
            confidence = self.calculate_target_confidence(target, camera_id)
            
            # MULTI-FACTOR CONFIDENCE: detection confidence + size + center + consistency
            if confidence > best_confidence:
                best_confidence = confidence
                best_target = target
                best_camera = camera_id
        
        # TARGET LOCKING MECHANISM
        if best_target and best_confidence > self.target_lock_threshold:
            self.target_locked = True
            self.target_lock_confidence = best_confidence
            return best_target, best_camera
            
        return best_target, best_camera if best_target else (None, None)
        
    def calculate_target_confidence(self, target, camera_id):
        """Multi-factor confidence scoring for target selection"""
        base_confidence = target.get('confidence', 0.5)
        bbox = target.get('bbox', [0, 0, 1, 1])
        
        # SIZE FACTOR: Larger targets more reliable
        bbox_area = bbox[2] * bbox[3]
        size_factor = min(bbox_area / (self.image_width * self.image_height * 0.1), 1.0)
        
        # CENTER FACTOR: Targets near image center more reliable  
        center = target.get('center', [self.image_width/2, self.image_height/2])
        center_distance = np.sqrt((center[0] - self.image_width/2)**2 + 
                                 (center[1] - self.image_height/2)**2)
        max_distance = np.sqrt((self.image_width/2)**2 + (self.image_height/2)**2)
        center_factor = 1.0 - (center_distance / max_distance)
        
        # CONSISTENCY FACTOR: Previously tracked targets more reliable
        consistency_factor = 0.8 if target.get('track_id') in self.camera_data_deques.get(camera_id, {}) else 0.5
        
        # CAMERA PRIORITY: Primary camera detections preferred
        camera_factor = 1.2 if camera_id == self.primary_camera else 1.0
        
        total_confidence = base_confidence * size_factor * center_factor * consistency_factor * camera_factor
        return min(total_confidence, 1.0)
```

---

## 🌪️ **3. Weather Physics Engine Integration**

### **3. Physics Engine F10 Weather Integration**

#### **WeatherPhysics Core Implementation**:

**WeatherPhysics Core Implementation**  
**📁 Header**: `AirLib/include/physics/WeatherPhysics.hpp`  
**📁 Implementation**: `AirLib/src/physics/WeatherPhysics.cpp`

```cpp
namespace msr { namespace airlib {
    class WeatherPhysics {
    public:
        struct WeatherForces {
            Vector3r turbulence_force = Vector3r::Zero();     // Real atmospheric effects
            Vector3r wind_gust = Vector3r::Zero();           // Dynamic wind gusts  
            real_T drag_multiplier = 1.0f;                   // Weather-based drag changes
            real_T air_density_multiplier = 1.0f;           // Altitude/weather density
            bool has_effects = false;                        // Enable/disable system
        };

        // WEATHER TYPE SPECIFIC CALCULATIONS
        static WeatherForces calculateWeatherEffects(
            float rain_intensity,      // 0.0-1.0 from F10 slider
            float snow_intensity,      // 0.0-1.0 from F10 slider  
            float dust_intensity,      // 0.0-1.0 from F10 slider
            float fog_intensity,       // 0.0-1.0 from F10 slider
            float falling_leaves_intensity, // 0.0-1.0 from F10 slider
            const Vector3r& drone_position,
            const Vector3r& drone_velocity, 
            real_T delta_time
        );

    private:
        // INDIVIDUAL WEATHER TYPE FORCE CALCULATIONS
        static Vector3r calculateRainTurbulence(float intensity, const Vector3r& velocity);
        static Vector3r calculateSnowTurbulence(float intensity, const Vector3r& velocity);  
        static Vector3r calculateDustTurbulence(float intensity, const Vector3r& velocity);
        static Vector3r calculateFogTurbulence(float intensity, const Vector3r& velocity);
        static Vector3r calculateLeavesTurbulence(float intensity, const Vector3r& velocity);
        
        static Vector3r generateRandomTurbulence(float intensity, float scale = 1.0f);
    };
}}
```

**WeatherPhysics.cpp - Physics Calculations**:
```cpp
WeatherPhysics::WeatherForces WeatherPhysics::calculateWeatherEffects(
    float rain_intensity, float snow_intensity, float dust_intensity,
    float fog_intensity, float falling_leaves_intensity,
    const Vector3r& drone_position, const Vector3r& drone_velocity, real_T delta_time)
{
    WeatherForces forces;
    
    float total_intensity = rain_intensity + snow_intensity + dust_intensity + 
                           fog_intensity + falling_leaves_intensity;
    
    if (total_intensity <= 0.001f) {
        return forces; // No weather effects
    }
    
    forces.has_effects = true;
    
    // RAIN EFFECTS: Downward pressure + increased drag
    if (rain_intensity > 0.0f) {
        forces.turbulence_force += Vector3r(
            generateRandomTurbulence(rain_intensity * 0.3f).x(),
            generateRandomTurbulence(rain_intensity * 0.3f).y(), 
            rain_intensity * 2.5f  // Downward force (positive Z = down in AirSim)
        );
        forces.drag_multiplier += rain_intensity * 0.3f; // 30% drag increase at full intensity
    }
    
    // SNOW EFFECTS: Swirling turbulence + moderate drag
    if (snow_intensity > 0.0f) {
        Vector3r snow_turbulence = generateRandomTurbulence(snow_intensity * 0.4f);
        forces.turbulence_force += snow_turbulence;
        forces.drag_multiplier += snow_intensity * 0.2f; // 20% drag increase
        forces.air_density_multiplier += snow_intensity * 0.03f; // Cold air is denser
    }
    
    // DUST EFFECTS: Chaotic turbulence + high drag  
    if (dust_intensity > 0.0f) {
        forces.turbulence_force += generateRandomTurbulence(dust_intensity * 0.7f);
        forces.drag_multiplier += dust_intensity * 0.5f; // 50% drag increase
        forces.air_density_multiplier += dust_intensity * 0.15f; // Particles increase density
    }
    
    // FOG EFFECTS: Subtle turbulence + increased air density
    if (fog_intensity > 0.0f) {
        forces.turbulence_force += generateRandomTurbulence(fog_intensity * 0.1f);
        forces.air_density_multiplier += fog_intensity * 0.05f; // Higher humidity = denser air
    }
    
    // FALLING LEAVES: Gentle upward forces + light turbulence
    if (falling_leaves_intensity > 0.0f) {
        forces.turbulence_force += Vector3r(
            generateRandomTurbulence(falling_leaves_intensity * 0.2f).x(),
            generateRandomTurbulence(falling_leaves_intensity * 0.2f).y(),
            -falling_leaves_intensity * 0.8f  // Gentle updraft (negative Z = up)
        );
    }
    
    // WIND GUSTS: Random directional forces
    forces.wind_gust = generateRandomTurbulence(total_intensity * 0.5f) * 3.0f;
    
    return forces;
}

Vector3r WeatherPhysics::generateRandomTurbulence(float intensity, float scale) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> dist(0.0f, 1.0f);
    
    return Vector3r(
        dist(gen) * intensity * scale,
        dist(gen) * intensity * scale, 
        dist(gen) * intensity * scale
    );
}
```

#### **Unreal Engine Bridge Integration**:

**Unreal Engine Bridge Interface**  
**📁 Header**: `Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.h`  
**📁 Implementation**: `Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.cpp`  
**📁 Weather API**: `Unreal/Plugins/AirSim/Source/Weather/WeatherLib.h`

```cpp
#pragma once
#include "CoreMinimal.h"
#include "Weather/WeatherLib.h"

// UNREAL-COMPATIBLE WEATHER FORCES (no AirLib dependencies)
USTRUCT(BlueprintType)
struct AIRSIM_API FWeatherForces {
    GENERATED_BODY()
    
    FVector TurbulenceForce = FVector::ZeroVector;    // Atmospheric turbulence
    FVector WindGust = FVector::ZeroVector;           // Random wind gusts
    float DragMultiplier = 1.0f;                     // Drag coefficient multiplier
    float AirDensityMultiplier = 1.0f;               // Air density changes
    bool HasEffects = false;                         // Weather system enabled
};

UCLASS(BlueprintType) 
class AIRSIM_API UWeatherPhysicsBridge : public UObject {
    GENERATED_BODY()
    
public:
    // MAIN BRIDGE METHOD: Get weather forces for physics engine
    UFUNCTION(BlueprintCallable, Category = "Weather Physics")
    static FWeatherForces GetWeatherForces(
        UWorld* World,
        const FVector& Position,
        const FVector& Velocity, 
        float DeltaTime
    );

private:
    // F10 WEATHER MENU INTEGRATION
    static float GetWeatherSliderValue(UWorld* World, EWeatherParamScalar Parameter);
    static bool IsWeatherEnabled(UWorld* World);
};
```

**`Unreal/Plugins/AirSim/Source/WeatherPhysicsBridge.cpp` - Real F10 Slider Integration**:
```cpp
FWeatherForces UWeatherPhysicsBridge::GetWeatherForces(
    UWorld* World, const FVector& Position, const FVector& Velocity, float DeltaTime)
{
    FWeatherForces forces;
    
    if (!World || !IsWeatherEnabled(World)) {
        return forces; // No weather system active
    }

    // GET REAL F10 WEATHER SLIDER VALUES
    float rain_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_RAIN);
    float snow_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_SNOW);
    float dust_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_DUST);  
    float fog_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_FOG);
    float leaves_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_MAPLELEAF);

    float total_intensity = rain_intensity + snow_intensity + dust_intensity + fog_intensity + leaves_intensity;
    
    if (total_intensity <= 0.001f) {
        return forces; // No weather effects
    }

    forces.HasEffects = true;

    // REAL-TIME WEATHER CALCULATIONS (matching FastPhysicsEngine integration)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> dist(0.0f, 1.0f);

    // RAIN: Downward forces + lateral turbulence + increased drag
    if (rain_intensity > 0.0f) {
        forces.TurbulenceForce.Z += rain_intensity * 250.0f;  // Downward pressure (cm units)
        forces.TurbulenceForce.X += dist(gen) * rain_intensity * 150.0f;
        forces.TurbulenceForce.Y += dist(gen) * rain_intensity * 150.0f; 
        forces.DragMultiplier += rain_intensity * 0.3f; // 30% drag increase
    }

    // SNOW: Swirling effects + moderate drag
    if (snow_intensity > 0.0f) {
        forces.TurbulenceForce.Z += snow_intensity * 120.0f;
        forces.TurbulenceForce.X += dist(gen) * snow_intensity * 200.0f;
        forces.TurbulenceForce.Y += dist(gen) * snow_intensity * 200.0f;
        forces.DragMultiplier += snow_intensity * 0.2f; // 20% drag increase
        forces.AirDensityMultiplier += snow_intensity * 0.03f; // Cold air denser
    }

    // DUST: Strong chaotic turbulence + high drag
    if (dust_intensity > 0.0f) {
        forces.TurbulenceForce.X += dist(gen) * dust_intensity * 350.0f;
        forces.TurbulenceForce.Y += dist(gen) * dust_intensity * 350.0f;
        forces.TurbulenceForce.Z -= dust_intensity * 100.0f; // Updraft from heat
        forces.DragMultiplier += dust_intensity * 0.5f; // 50% drag increase
        forces.AirDensityMultiplier += dust_intensity * 0.15f; // Particles increase density
    }

    // FOG: Subtle atmospheric effects
    if (fog_intensity > 0.0f) {
        forces.TurbulenceForce += FVector(dist(gen), dist(gen), dist(gen)) * fog_intensity * 80.0f * 0.5f;
        forces.AirDensityMultiplier += fog_intensity * 0.05f; // Humidity increases density
    }

    // FALLING LEAVES: Gentle environmental effects  
    if (leaves_intensity > 0.0f) {
        forces.TurbulenceForce.X += dist(gen) * leaves_intensity * 100.0f * 0.7f;
        forces.TurbulenceForce.Y += dist(gen) * leaves_intensity * 100.0f * 0.7f;
        forces.TurbulenceForce.Z += leaves_intensity * 80.0f; // Gentle downward
    }

    // WIND GUSTS: Random directional forces
    forces.WindGust = FVector(
        dist(gen) * total_intensity * 300.0f,
        dist(gen) * total_intensity * 300.0f,
        dist(gen) * total_intensity * 300.0f * 0.6f
    );

    return forces;
}

float UWeatherPhysicsBridge::GetWeatherSliderValue(UWorld* World, EWeatherParamScalar Parameter) {
    if (!World) return 0.0f;
    
    // DIRECT F10 WEATHER MENU INTEGRATION
    return UWeatherLib::getWeatherParamScalar(World, Parameter);
}

bool UWeatherPhysicsBridge::IsWeatherEnabled(UWorld* World) {
    if (!World) return false;
    
    // CHECK IF F10 WEATHER SYSTEM IS ACTIVE
    return UWeatherLib::getIsWeatherEnabled(World);
}
```

#### **FastPhysicsEngine Integration**:

**FastPhysicsEngine Integration**  
**📁 Header**: `AirLib/include/physics/FastPhysicsEngine.hpp`  
**📁 Implementation**: `AirLib/src/physics/FastPhysicsEngine.cpp`  
**📁 Base Class**: `AirLib/include/physics/PhysicsEngineBase.hpp`

```cpp
class FastPhysicsEngine : public PhysicsEngineBase {
private:
    // WEATHER SYSTEM STATE
    bool weather_effects_enabled_ = false;
    static WeatherPhysics::WeatherForces current_weather_forces_;

public:
    // WEATHER EFFECTS METHODS (override from base class)
    void enableWeatherEffects(bool enable) override { 
        weather_effects_enabled_ = enable; 
    }
    
    bool isWeatherEffectsEnabled() const override { 
        return weather_effects_enabled_; 
    }
    
    WeatherPhysics::WeatherForces getWeatherEffects(const PhysicsBody& body, real_T dt) override {
        unused(body); unused(dt);
        return current_weather_forces_;
    }

    // STATIC BRIDGE METHOD: Called from Unreal WeatherPhysicsBridge
    static void setCurrentWeatherForces(const WeatherPhysics::WeatherForces& forces) {
        current_weather_forces_ = forces;
    }

private:
    // ENHANCED DRAG CALCULATION WITH WEATHER EFFECTS
    static Wrench getDragWrench(const PhysicsBody& body,
                               const Quaternionr& orientation,
                               const Vector3r& linear_vel,
                               const Vector3r& angular_vel_body,
                               const Vector3r& wind_world) {
        Wrench wrench = Wrench::zero();
        
        // WEATHER-MODIFIED AIR DENSITY
        const real_T air_density = body.getEnvironment().getState().air_density 
                                  * current_weather_forces_.air_density_multiplier;

        // PER-VERTEX DRAG CALCULATION WITH WEATHER MULTIPLIERS
        for (uint vi = 0; vi < body.dragVertexCount(); ++vi) {
            const auto& vertex = body.getDragVertex(vi);
            const Vector3r vel_vertex = linear_vel_body + angular_vel_body.cross(vertex.getPosition());
            const real_T vel_comp = vertex.getNormal().dot(vel_vertex);
            
            if (vel_comp > kDragMinVelocity) {
                // APPLY WEATHER DRAG MULTIPLIER TO EACH VERTEX
                const Vector3r drag_force = vertex.getNormal() * 
                    (-vertex.getDragFactor() * air_density * 
                     current_weather_forces_.drag_multiplier * vel_comp * vel_comp);
                     
                wrench.force += drag_force;
                wrench.torque += vertex.getPosition().cross(drag_force);
            }
        }

        return wrench;
    }

    // MAIN PHYSICS INTEGRATION POINT
    static void getNextKinematicsNoCollision(TTimeDelta dt, PhysicsBody& body, 
                                            const Kinematics::State& current,
                                            Kinematics::State& next, 
                                            Wrench& next_wrench, 
                                            const Vector3r& wind, 
                                            const Vector3r& ext_force) {
        // ... existing physics calculations ...
        
        // ADD WEATHER EFFECTS TO TOTAL FORCES
        Wrench weather_wrench = Wrench::zero();
        if (current_weather_forces_.has_effects) {
            // COMBINE TURBULENCE AND WIND GUST FORCES
            weather_wrench.force = current_weather_forces_.turbulence_force + 
                                  current_weather_forces_.wind_gust;
        }

        // TOTAL FORCE = BODY FORCES + DRAG + EXTERNAL + WEATHER
        next_wrench = body_wrench + drag_wrench + ext_force_wrench + weather_wrench;
        
        // Weather now affects actual drone movement!
        next.accelerations.linear = (next_wrench.force / body.getMass()) + 
                                   body.getEnvironment().getState().gravity;
    }
};
```

#### **Complete Weather Integration Flow**:

```cpp
// STEP 1: User presses F10 → Weather menu opens
// STEP 2: User adjusts weather sliders (rain, snow, dust, fog, leaves)
// STEP 3: UWeatherLib stores slider values in Unreal Engine
// STEP 4: WeatherPhysicsBridge.GetWeatherForces() reads slider values  
// STEP 5: Bridge converts Unreal forces to AirLib format
// STEP 6: FastPhysicsEngine.setCurrentWeatherForces() receives forces
// STEP 7: getDragWrench() applies weather multipliers to drag calculation
// STEP 8: getNextKinematicsNoCollision() adds weather forces to total forces
// STEP 9: Drone physics responds with realistic weather-affected movement!

// RESULT: F10 weather sliders now control actual drone flight dynamics
//         Rain creates downward forces + increased drag
//         Snow creates swirling turbulence + cold air density
//         Dust creates chaotic turbulence + high drag
//         All effects combine additively for realistic mixed weather
```

#### **Settings.json**:

```json
{
  "SettingsVersion": 2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "RpcEnabled": true,
      "TcpPort": 4560,
      "ControlIp": "remote",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "172.22.112.1",
      "X": -5.0,
      "Y": 0.0,
      "Z": 0.5,
      "Yaw": 0.0,
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled": true
        },
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled": true
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled": true
        },
        "Lidar1": {
          "SensorType": 6,
          "Enabled": true,
          "NumberOfChannels": 16,
          "Range": 100,
          "PointsPerSecond": 10000,
          "DrawDebugPoints": true,
          "X": 0,
          "Y": 0,
          "Z": 0,
          "Roll": 0,
          "Pitch": 0,
          "Yaw": 0
        }
      }
    }
  }
}
```

This comprehensive implementation transforms AirSim's F10 weather menu from purely visual effects into a fully functional weather physics system that realistically affects drone flight behavior.

---
## 🛠️ Installation & Deployment

### **Prerequisites & Setup**

**System Requirements**:
- Windows 10/11 or Ubuntu 22.04
- Unreal Engine 5.5
- ROS2 Humble
- Python 3.8+ with OpenCV, PyTorch
- PX4 SITL (optional for hardware-in-loop)
- ArduPilot (in the works)

### **Quick Start Guide**

## Startup Instructions (WSL 2.5.10.0 & Windows 10)

### Step 1: Generate AirSim Settings
Run the Python generate settings.py file to determine the number of drones:
```bash
wsl
cd Cosys-AirSim/PythonClient/multirotor
python3 generate_settings.py 2
```

### Step 2: Launch Cosys-AirSim 
- Start Cosys-AirSim in Unreal Engine 5.5
- Ensure multi-camera configuration is enabled in settings.json

### Step 3: Launch PX4 SITL (for multiple drones)
For each drone, run in separate terminals:
```bash
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i <instance_id>
=======

- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
>>>>>>> main
```
Example for two drones:
```bash
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 1
```
Or use for single drone:
```bash
make px4_sitl_default none_iris
```

### Step 4: Build and Source ROS 2 Workspace

```bash
colcon build
source install/setup.bash 
```

### Step 5: Launch ROS2 Nodes

- **Single drone for testing ONLY (simple):** 
```bash
ros2 launch airsim_ros_pkgs simple_single_drone.launch.py
```

- **Single drone** 
```bash
ros2 launch airsim_ros_pkgs single_drone.launch.py
```

- **Multi-drone**
```bash
ros2 launch airsim_ros_pkgs multi_drone.launch.py
``` 

- **Launch Single drone with AI motion detection**
```bash
# Terminal 1: Launch drone node
ros2 launch airsim_ros_pkgs single_drone.launch.py

# Terminal 2: Launch AI motion detection 
ros2 launch airsim_ros_pkgs motion_detection_launch.py vehicle_name:=Drone1
```

- **Multi-drone with AI motion detection**
```bash
# Terminal 1: Launch multi-drone fleet
ros2 launch airsim_ros_pkgs multi_drone.launch.py

# Terminal 2: Launch motion detection for each drone 
ros2 launch airsim_ros_pkgs motion_detection_launch.py vehicle_name:=Drone1
ros2 launch airsim_ros_pkgs motion_detection_launch.py vehicle_name:=Drone2
```

### Step 6: Test System 

### **AI Vision System Deployment**

**📁 Launch File Locations:**

ros2/src/airsim_ros_pkgs/launch/motion_detection_launch.py
ros2/src/airsim_ros_pkgs/launch/generalised_object_tracking_launch.py
ros2/src/airsim_ros_pkgs/launch/multi_drone.launch.py
```

```bash  
# Deploy person following on Drone1
ros2 launch airsim_ros_pkgs motion_detection_launch.py \
  vehicle_name:=Drone1 \
  confidence_threshold:=0.8 \
  enable_following:=true \
  follow_distance:=5.0

# Deploy car tracking on Drone2  
ros2 launch airsim_ros_pkgs generalised_object_tracking_launch.py \
  vehicle_name:=Drone2 \
  target_class_id:=2 \
  confidence_threshold:=0.6 \
  follow_distance:=3.0

# Deploy drone detection on Drone3
ros2 launch airsim_ros_pkgs generalised_object_tracking_launch.py \
  vehicle_name:=Drone3 \
  target_class_id:=4 \
  confidence_threshold:=0.7
```

```bash
# Takeoff drone
ros2 service call /drone1/takeoff airsim_interfaces/srv/Takeoff "{}"

# Start AI powered target search (drone hovers and uses vision)
ros2 service call /drone1/search_target airsim_interfaces/srv/SearchTarget "{
  search_radius: 10.0,
  search_time: 30.0,
  min_confidence: 0.7
}"

# Monitor AI detections
ros2 topic echo /target_detection 

# Track detected target
ros2 service call /drone1/track_target airsim_interfaces/srv/TrackTarget "{
  target_x: 5.0,
  target_y: 3.0,
  target_z: -2.0
}"
```

---

## Architecture Overview

### System Architecture Diagram

```
                    ┌─────────────────────────────────────────┐
                    │         COORDINATION NODE               │
                    │    (Fleet Command & Control)           │
                    │  • Global Services (/takeoff_all)      │
                    │  • System Monitoring                   │  
                    │  • Mission Coordination                │
                    └─────────────┬───────────────────────────┘
                                  │
                ┌─────────────────┼─────────────────┐
                │                 │                 │
                ▼                 ▼                 ▼
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
    │  MULTIROTOR     │ │  MULTIROTOR     │ │  MULTIROTOR     │
    │  NODE - Drone1  │ │  NODE - Drone2  │ │  NODE - DroneN  │
    │                 │ │                 │ │                 │
    │ • Individual    │ │ • Individual    │ │ • Individual    │
    │   Control       │ │   Control       │ │   Control       │
    │ • Sensor Data   │ │ • Sensor Data   │ │ • Sensor Data   │
    │ • RPC Client    │ │ • RPC Client    │ │ • RPC Client    │
    └─────────┬───────┘ └─────────┬───────┘ └─────────┬───────┘
              │                   │                   │
              ▼                   ▼                   ▼
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
    │   AI MOTION     │ │   AI MOTION     │ │   AI MOTION     │
    │ DETECTION NODE  │ │ DETECTION NODE  │ │ DETECTION NODE  │
    │ (Optional)      │ │ (Optional)      │ │ (Optional)      │
    │                 │ │                 │ │                 │
    │ • YOLO Object   │ │ • YOLO Object   │ │ • YOLO Object   │
    │   Detection     │ │   Detection     │ │   Detection     │
    │ • Motion Track  │ │ • Motion Track  │ │ • Motion Track  │
    │ • Target Class  │ │ • Target Class  │ │ • Target Class  │
    └─────────────────┘ └─────────────────┘ └─────────────────┘
              │                   │                   │
              ▼                   ▼                   ▼
        ┌─────────────────────────────────────────────────┐
        │              AIRSIM SIMULATION                  │
        │         (Unreal Engine 5.5 + Physics)          │
        └─────────────────────────────────────────────────┘
```

### Key Components

#### A. Vehicle Nodes

- **VehicleNodeBase**: Abstract base for all vehicle types. Handles parameter management, AirSim connections, publishers/services/timers and callback groups for parallel sensor processing. 
  - **Role**: Abstract foundation for all vehicle types
  - **Features**: Parameter management, AirSim connections, parallel sensor processing
  - **Benefits**: Code reuse, consistent interfaces, extensible design

- **MultirotorNode**: Inherits from VehicleNodeBase. Implements drone-specific publishers (odom, GPS, IMU, environment, camera, lidar), services (takeoff, land, gps_waypoint, track_target), velocity command subscribers, and sensor data processing. 
  - **Role**: Drone-specific control and operations
  - **Inherits**: VehicleNodeBase
  - **Publishers**: Odometry, GPS, IMU, cameras, LiDAR
  - **Services**: Takeoff, land, waypoint navigation, target tracking

- **SimpleMultirotorNode**: FOR TESTING ONLY. Minimal node for single-drone testing/debugging. No inheritance, direct AirSim connection.

#### B. Coordination Node

- **CoordinationNode**: Manages global services (reset all, takeoff all, land all, pause simulation, health check), publishes system status and GPS origin, monitors all vehicles.
  - **Role**: Fleet-wide command and control
  - **Features**: Global services, system monitoring, armed checks
  - **Services**: `/takeoff_all`, `/land_all`, `/reset_all`

#### C. Settings Parser 
- **VehicleSettingsParser**: Parses AirSim `settings.json` to extract vehicle configurations and global parameters, enabling dynamic node creation. 

---

## ROS Topics and Services

### Topics Published Per Vehicle 

| Topic Name                | Message Type                       | Description                                          |
|---------------------------|----------------------------------------|--------------------------------------------------|
| `/droneX/odom`            | `nav_msgs/msg/Odometry`                | Vehicle odometry (position, orientation, velocity)|
| `/droneX/gps`             | `sensor_msgs/msg/NavSatFix`            | GPS data (lat, lon, alt)                         |
| `/droneX/imu`             | `sensor_msgs/msg/Imu`                  | IMU data (orientation, angular/linear accel)     |
| `/droneX/environment`     | `airsim_interfaces/msg/Environment`    | AirSim environment state (pressure, temp, etc.)  |
| `/droneX/camera0/image`   | `sensor_msgs/msg/Image`                | Front camera image (640x480)                     |
| `/droneX/camera1/image`   | `sensor_msgs/msg/Image`                | Right camera image (640x480)                     |
| `/droneX/camera2/image`   | `sensor_msgs/msg/Image`                | Back camera image (640x480)                      |
| `/droneX/camera3/image`   | `sensor_msgs/msg/Image`                | Left camera image (640x480)                      |
| `/droneX/camera{0-3}/camera_info` | `sensor_msgs/msg/CameraInfo`   | Camera calibration info                          |
| `/droneX/lidar0/points`   | `sensor_msgs/msg/PointCloud2`          | Lidar point cloud                                |
| `/droneX/mag`             | `sensor_msgs/msg/MagneticField`        | Magnetometer data                                |
| `/droneX/baro`            | `sensor_msgs/msg/Range`                | Barometer/altimeter data                         |

### Topics Published by Coordination Node 

| Topic Name                | Message Type                       | Description                                      |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/origin_geo_point`       | `airsim_interfaces/msg/GPSYaw`     | Global GPS origin for all vehicles               |
| `/system_status`          | `airsim_interfaces/msg/StringArray`| Status of all vehicles (READY/ERROR)             |
| `/clock`                  | `rosgraph_msgs/msg/Clock`          | Simulation time                                  |

### Services Per Vehicle

| Service Name              | Service Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/droneX/takeoff`         | `airsim_interfaces/srv/Takeoff`    | Takeoff command for this vehicle                 |
| `/droneX/land`            | `airsim_interfaces/srv/Land`       | Land command for this vehicle                    |
| `/droneX/reset`           | `airsim_interfaces/srv/Reset`      | Reset this vehicle in AirSim                     |
| `/droneX/gps_waypoint`    | `airsim_interfaces/srv/GpsWaypoint`| Move to GPS waypoint                             |
| `/droneX/search_target`   | `airsim_interfaces/srv/SearchTarget`| AI-powered moving target search (hover + detect)|
| `/droneX/track_target`    | `airsim_interfaces/srv/TrackTarget`| Move to and track specific target coordinates    |

### Global Services (Coordination Node)

| Service Name              | Service Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/reset_all`              | `airsim_interfaces/srv/Reset`      | Reset all vehicles                               |
| `/takeoff_all`            | `airsim_interfaces/srv/Takeoff`    | Takeoff all vehicles                             |
| `/land_all`               | `airsim_interfaces/srv/Land`       | Land all vehicles                                |
| `/pause_simulation`       | `std_srvs/srv/SetBool`             | Pause/unpause AirSim simulation                  |
| `/armed_check`           | `airsim_interfaces/srv/ListSceneObjectTags` | Check armed/disarmed status of all vehicles     |

### Command Topics (Subscribers)

| Topic Name                | Message Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/droneX/vel_cmd_body_frame` | `airsim_interfaces/msg/VelCmd`  | Velocity command in body frame                   |
| `/droneX/vel_cmd_world_frame`| `airsim_interfaces/msg/VelCmd`  | Velocity command in world frame                  |

---

### Example: Search Target Service

The new `/droneX/search_target` service allows a drone to hover in place and use AI vision to detect moving targets.

**Service definition (`airsim_interfaces/srv/SearchTarget.srv`):**
```plaintext
float64 search_radius     # Not used in AI mode (drone hovers)
float64 search_time       # Time to search in seconds
float64 min_confidence    # Minimum detection confidence (0.0-1.0)
---
bool success
float64 target_x          # Target world coordinates
float64 target_y
float64 target_z
float64 confidence        # Detection confidence
string message
```

**Example usage:**
```bash
ros2 service call /Drone1/search_target airsim_interfaces/srv/SearchTarget "{
  search_radius: 0.0,
  search_time: 30.0,
  min_confidence: 0.7
}"
```

**AI Detection Response:**
```json
{
  "success": true,
  "target_x": 15.2,
  "target_y": -8.7,
  "target_z": -2.0,
  "confidence": 0.85,
  "message": "Moving target found via AI vision system"
}
```

### New Modular Architecture Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `vehicle_node_base.hpp/cpp`       | Abstract base for all vehicle nodes. Handles parameters, connections, publishers, timers, callback groups.|
| `multirotor_node.hpp/cpp`         | Implements drone-specific logic: sensor publishers, command subscribers, takeoff/land/gps_waypoint/track_target services.         |
| `simple_multirotor_node.cpp`      | Minimal node for single-drone testing/debugging. Direct AirSim connection, basic publishers/services.   |
| `coordination_node.hpp/cpp`       | Global node for system-wide services, status monitoring, GPS origin publishing, health checks.          |
| `vehicle_settings_parser.hpp/cpp` | Parses AirSim `settings.json` for dynamic vehicle configuration. Used by launch files for node creation.|
| `multirotor_main.cpp`             | Main entry for launching a multirotor node (per vehicle).                                              |
| `coordination_main.cpp`           | Main entry for launching the coordination node.                                                        |

### Launch Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `simple_single_drone.launch.py`   | Launches a single `simple_multirotor_node` in `/drone1` namespace. For quick testing/debugging.         |
| `single_drone.launch.py`          | Launches one `multirotor_node` instance and the coordination node.                                     |
| `multi_drone.launch.py`           | Dynamically launches nodes for all vehicles in `settings.json`, plus the coordination node.             |
| `airsim_node.launch.py`           | Legacy: launches the old monolithic node.                                                               |

### Legacy (Old Architecture) Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `airsim_ros_wrapper.h/cpp`        | Monolithic node managing all vehicles in one process. Single point of failure, sequential processing.   |
| `airsim_node.cpp`                 | Main for launching the old monolithic node.                                                             |

---

## Detailed Comparisons: Old vs New Architecture

| Aspect                | Old (Monolithic)                      | New (Modular, Multi-Node)                | Why New is Better                        |
|-----------------------|---------------------------------------|------------------------------------------|------------------------------------------|
| Node Structure        | Single node for all vehicles          | One node per vehicle, plus coordination  | Fault isolation, parallelism             |
| Extensibility         | Hard to add new vehicle types         | Easy to add new vehicle types/classes    | Clean inheritance, modular files         |
| Fault Isolation       | Failure in one vehicle affects all    | Each vehicle node is independent         | One crash doesn't affect others          |
| Performance           | Single-threaded, bottlenecked         | Multi-threaded, scalable                 | Parallel sensor processing               |
| Launch Flexibility    | Static, hardcoded                     | Dynamic, based on settings.json          | Add/remove vehicles without code change  |
| Coordination          | Ad-hoc, limited                       | Dedicated coordination node              | Centralized global services              |
| Testing/Debugging     | Hard to isolate issues                | Can launch/test nodes individually       | Per-vehicle logs, easier debugging       |
| Code Organization     | Large, monolithic classes             | Clean, separated by vehicle type         | Easier maintenance, less code coupling   |
| ROS2 Best Practices   | Not followed                          | Follows ROS2 node/component patterns     | Modern, maintainable, scalable           |
| Resource Management   | All processing on single core/thread  | Per-node threading, callback groups      | Better CPU utilization                   |
| RPC Connections       | Shared for all vehicles               | Independent per vehicle                  | No RPC contention, isolated failures     |
| Sensor Timers         | Shared, sequential                    | Per-vehicle, parallel                    | No sensor bottlenecks                    |

**Summary:**  
The new architecture is modular, robust, and scalable. Each vehicle runs in its own node and namespace, with isolated connections and timers. The coordination node manages global services and monitoring. This design enables parallel sensor processing, fault isolation, and dynamic vehicle management, making it ideal for large-scale multi-vehicle simulation.

## Design Decisions

- **Modularity:** Each vehicle type gets its own node class, making it easy to extend for new vehicle types (cars, drones, etc.).
- **Isolation:** Per-vehicle nodes mean a crash or RPC error in one vehicle does not affect others.
- **Parallelism:** Isolated callback groups and timers allow sensors to be processed in parallel, improving performance.
- **Dynamic Launch:** VehicleSettingsParser enables dynamic node creation based on `settings.json`, so you can add/remove vehicles without changing code.
- **Coordination Node:** Centralizes global services (reset, takeoff, land, pause, health check) and system status monitoring.
- **Legacy Compatibility:** Old files (`airsim_ros_wrapper.*`, `airsim_node.cpp`) are retained for reference and backward compatibility, but are not recommended for new deployments.

## Troubleshooting & FAQ

- **bad_weak_ptr errors:** Ensure you have rebuilt your workspace and are not running old binaries.
- **Nodes not connecting:** Check that AirSim is running and vehicle names match those in `settings.json`.
- **Adding vehicles:** Update `settings.json` and use `multi_drone.launch.py`—nodes will be created automatically.
- **PX4 SITL:** Each drone instance needs its own PX4 SITL process.
- **Logs:** Each vehicle node logs independently; use `ros2 node list` and `ros2 topic list` to inspect running nodes and topics.

### Common Commands

- **List nodes:**
  ```bash
  ros2 node list
  ```
- **List topics:**
  ```bash
  ros2 topic list
  ```
- **Echo odometry:**
  ```bash
  ros2 topic echo /drone1/odom
  ```
- **Takeoff:**
  ```bash
  ros2 service call /drone1/takeoff airsim_interfaces/srv/Takeoff "{}"
  ```
- **Land:**
  ```bash
  ros2 service call /drone1/land airsim_interfaces/srv/Land "{}"
  ```
- **Global takeoff:**
  ```bash
  ros2 service call /takeoff_all airsim_interfaces/srv/Takeoff "{}"
  ```
- **Move to GPS waypoint:**
  ```bash
  ros2 service call /drone1/gps_waypoint airsim_interfaces/srv/GpsWaypoint "{latitude: 47.641468, longitude: -122.140165, altitude: 10.0, speed: 5.0, tolerance: 1.0}"
  ```
---

## 🎓 Internship Project Conclusion

This **6-month internship project** successfully transformed Cosys-AirSim from a single-vehicle simulation tool into a **production-ready multi-vehicle platform** with **advanced AI capabilities** and **realistic physics simulation**. The modular architecture enables unlimited scalability, while the AI vision systems provide autonomous operation capabilities previously unavailable.

## 📞 Contact & Acknowledgments

**🎓 Internship Duration**: 6 months (13 May 2025 - 5 December 2025)  
**📧 Contact**: Paul Cheng 
* Feel free to reach out if you ever need any help.  

**Special Thanks**:
- Cosys-Lab team for building on Microsoft AirSim
- Microsoft AirSim team for the foundational platform
- ROS2 community for the robotics middleware
- Open-source computer vision community (YOLO, OpenCV)

---

## 📄 License & Citation

This enhanced Cosys-AirSim implementation builds upon the excellent work of Microsoft AirSim and Cosys-Lab. The [original AirSim MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) applies to all native AirSim source files and internship modifications.

**Citation for Internship Work**:
```bibtex
@misc{internship_project_2025,
  author = {Paul Cheng},
  title = {6-Month Internship: Modular Multi-Vehicle Simulation with AI Integration},
  organization = {},
  year = {2025},
  note = {Advanced ROS2 architecture, autonomous vision systems, and physics enhancements for drone fleets},
  url = {https://github.com/itsapaulblem/Cosys_Airsim_Exploration}
}
```

**Original AirSim Citation**:
```bibtex
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
``` 

* Updated for Unreal 5.
* Added [multi-layer annotation](https://cosys-lab.github.io/Cosys-AirSim/annotation) for groundtruth label generation with RGB, greyscale and texture options. Extensive API integration and available for camera and GPU-LiDAR sensors.
* Added [Instance Segmentation](https://cosys-lab.github.io/Cosys-AirSim/instance_segmentation). 
* Added [Echo sensor type](https://cosys-lab.github.io/Cosys-AirSim/echo) for simulation of sensors like sonar and radar.
* Added [GPU LIDAR sensor type](https://cosys-lab.github.io/Cosys-AirSim/gpulidar): Uses GPU acceleration to simulate a LiDAR sensor. Can support much higher point density then normal LiDAR and behaves more authentic and has realistic intensity generation.
* Added [skid steering SimMode and vehicle type](https://cosys-lab.github.io/Cosys-AirSim/skid_steer_vehicle). ClearPath Husky and Pioneer P3DX implemented as vehicle types using this new vehicle model. 
* Added [Matlab API Client](https://cosys-lab.github.io/Cosys-AirSim/matlab) implementation as an easy to install Matlab toolbox.
* Added various [random but deterministic dynamic object types and world configuration options](https://cosys-lab.github.io/Cosys-AirSim/dynamic_objects).
* Added [Artificial Lights](https://cosys-lab.github.io/Cosys-AirSim/lights). 
* Added BoxCar vehicle model to the Car SimMode to have a smaller vehicle to use in indoor spaces.
* Added a new image type called [Lighting](https://cosys-lab.github.io/Cosys-AirSim/image_apis) which only shows the light information and no materials.
* Updated [ComputerVision mode](https://cosys-lab.github.io/Cosys-AirSim/image_apis#computer-vision-mode-1): Now has full API and Simulation just like other vehicle types. It mostly means it can now have sensors attached (outside of IMU). Improved handling and camera operation.
* Updated [LIDAR sensor type](https://cosys-lab.github.io/Cosys-AirSim/lidar): Fixed not tracing correctly, added ground truth (point labels) generation, added range-noise generation. Improved API pointcloud delivery to be full scan instead of being frame-rate dependent and partial.
* Updated the camera, Echo and (GPU-)LiDAR sensors to be uncoupled from the vehicle and be placed as external world sensors.
* Updated sensors like cameras, Echo sensor and GPU-LiDAR to ignore certain objects with the _MarkedIgnore_ Unreal tag and enabling the "IgnoreMarked" setting in [the settings file](https://cosys-lab.github.io/Cosys-AirSim/settings).
* Updated cameras sensor with more distortion features such as chromatic aberration, motion blur and lens distortion. 
* Updated Python [ROS implementation](https://cosys-lab.github.io/Cosys-AirSim/ros_python) with completely new implementation and feature set.
* **ENHANCED BY INTERNSHIP**: Updated C++ [ROS2 implementation](https://cosys-lab.github.io/Cosys-AirSim/ros_cplusplus) with modular architecture, AI integration, and weather physics.
* Dropped support for Unity Environments.

---

## References

- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [ROS2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [Cosys-AirSim GitHub](https://github.com/Cosys-Lab/Cosys-AirSim)

This project is released under the MIT License. Please review the [License file](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) for more details.

**🎯 Project Status**: ✅ **Successfully Completed**  
**📈 Impact**: Production-ready multi-vehicle simulation platform with AI capabilities  
*Project completed: December 5, 2025 | Documentation version: 1.0*
