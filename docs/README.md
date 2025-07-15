# Cosys-AirSim Documentation

Cosys-AirSim is a simulator for drones, cars and more, with extensive API support, built on [Unreal Engine](https://www.unrealengine.com/). It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment.

This fork is based on last public AirSim release from Microsoft's GitHub.
Cosys-Lab made extensive modifications to the AirSim platform to support multiple projects and research goals. 
Please contact a Cosys-Lab researcher to get more in depth information on our work or if you wish to collaborate. 
The [original AirSim MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) applies to all native AirSim source files. 
Please note that we use that same [MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) as which applies to all changes made by Cosys-Lab in case you plan to do anything within this repository.
Do note that this repository is provided as is, will not be actively updated and comes without warranty or support. 
Please contact a Cosys-Lab researcher to get more in depth information on which branch or version is best for your work.

This documentation is for the latest stable Unreal Version v5.5 on the [main branch](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main), maintained for support, and is available for builds in the [releases](https://github.com/Cosys-Lab/Cosys-AirSim/releases).
Unreal [5.2.1](https://github.com/Cosys-Lab/Cosys-AirSim/tree/5.2.1) is also available for long term support builds but the documentation is best read from [source](https://github.com/Cosys-Lab/Cosys-AirSim/tree/5.2.1/docs).

## 📚 Documentation Organization

This documentation is organized into specialized sections for different development workflows:

### 🚀 [Installation & Setup](installation/)
Complete installation guides for all platforms and deployment methods:
- **[Precompiled Binaries](installation/run_packaged.md)** - Quick start (5 min)
- **[Precompiled Plugin](installation/install_precompiled.md)** - Custom environments (15 min)
- **[Windows Source Build](installation/install_windows.md)** - Full development (60+ min)
- **[Linux Source Build](installation/install_linux.md)** - Full development (60+ min)
- **[Docker Environment](installation/docker_ubuntu.md)** - Containerized development

### 🔌 [Core Features & APIs](core/)
Essential AirSim functionality and APIs:
- **[API Documentation](core/apis.md)** - Core API concepts and C++ reference
- **[Vehicle Types](core/custom_drone.md)** - Drones, cars, and specialized vehicles
- **[Sensor Systems](core/sensors.md)** - LiDAR, cameras, echo sensors, and more
- **[Configuration](core/settings.md)** - Complete settings.json reference
- **[Computer Vision](core/image_apis.md)** - Image capture and processing

### 🚁 [PX4 & MAVLink Development](px4/)
Flight controller integration and MAVLink communication:
- **[PX4 Setup](px4/px4_setup.md)** - Complete PX4 configuration
- **[SITL Simulation](px4/px4_sitl.md)** - Software-in-the-Loop setup
- **[Multi-Vehicle](px4/px4_multi_vehicle.md)** - Multi-drone coordination
- **[MAVLink Architecture](px4/mavlink_architecture.md)** - Communication protocols
- **[Troubleshooting](px4/px4_logging.md)** - Debugging and diagnostics

### 🤖 [ROS2 Development](ros2/)
ROS2 integration and robotics framework support:
- **[C++ ROS2 Wrapper](ros2/ros_cplusplus.md)** - Production-ready ROS2 integration
- **[Python Integration](ros2/ros_python.md)** - Python-based ROS2 nodes
- **Complete ROS2 Documentation**: [Additional guides in /ros2/](../ros2/)

### 🎮 [Unreal Engine Development](unreal/)
Environment creation and plugin development:
- **[Custom Environments](unreal/unreal_custenv.md)** - Creating simulation environments
- **[Plugin Development](unreal/working_with_plugin_contents.md)** - Extending AirSim
- **[Visual Features](unreal/lights.md)** - Lighting, materials, and rendering
- **[Asset Management](unreal/meshes.md)** - 3D models and optimization

### 🎯 [Specialized Topics](image_segmentation/)
Advanced features and specialized use cases:
- **[Instance Segmentation](image_segmentation/)** - Object-level segmentation
- **[Annotation Systems](image_segmentation/annotation.md)** - Ground truth generation

## Associated publications

- [Cosys-AirSim: A Real-Time Simulation Framework Expanded for Complex Industrial Applications](https://arxiv.org/abs/2303.13381)
```
@inproceedings{cosysairsim2023jansen,
  author={Jansen, Wouter and Verreycken, Erik and Schenck, Anthony and Blanquart, Jean-Edouard and Verhulst, Connor and Huebel, Nico and Steckel, Jan},
  booktitle={2023 Annual Modeling and Simulation Conference (ANNSIM)}, 
  title={COSYS-AIRSIM: A Real-Time Simulation Framework Expanded for Complex Industrial Applications}, 
  year={2023},
  volume={},
  number={},
  pages={37-48},
  doi={}}
```

You can also find the presentation of the live tutorial of Cosys-AirSim at ANNSIM '23 conference [here](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/annsim23_tutorial) together with the associated videos.


- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
```
@inproceedings{lidarsim2022jansen,
  author={Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  booktitle={2022 IEEE Sensors}, 
  title={Physical LiDAR Simulation in Real-Time Engine}, 
  year={2022},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/SENSORS52175.2022.9967197}}
}
```
- [Simulation of Pulse-Echo Radar for Vehicle Control and SLAM](https://www.mdpi.com/1424-8220/21/2/523)
```
@Article{echosim2021schouten,
  author={Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title={Simulation of Pulse-Echo Radar for Vehicle Control and SLAM},
  JOURNAL={Sensors},
  volume={21},
  year={2021},
  number={2},
  article-number={523},
  doi={10.3390/s21020523}
}
```

## Cosys-Lab Modifications
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
* Updated C++ [ROS2 implementation](https://cosys-lab.github.io/Cosys-AirSim/ros_cplusplus) to support custom Cosys-AirSim features.
* Dropped support for Unity Environments.

Some more details on our changes can be found in the [changelog](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/CHANGELOG.md).

## 🚀 Quick Start

Choose the installation method that best fits your needs:

### For Quick Evaluation (5 minutes)
**[→ Download Precompiled Binaries](installation/run_packaged.md)**
- Ready-to-run simulation environments
- No compilation required
- Perfect for testing and evaluation

### For Custom Environments (15 minutes)  
**[→ Install Precompiled Plugin](installation/install_precompiled.md)**
- Drop into existing Unreal projects
- Create custom simulation environments
- Faster than building from source

### For Full Development (60+ minutes)
**[→ Build from Source](installation/)**
- **Windows**: [Complete build guide](installation/install_windows.md)
- **Linux**: [Complete build guide](installation/install_linux.md)
- **Docker**: [Containerized development](installation/docker_ubuntu.md)

## 🎯 Common Workflows

### Researchers & Academics
1. **[Quick Start](installation/run_packaged.md)** → **[Core APIs](core/apis.md)** → **[Vehicle Configuration](core/settings.md)**
2. For custom scenarios: **[Unreal Development](unreal/unreal_custenv.md)**

### Robotics Developers  
1. **[Installation](installation/)** → **[ROS2 Integration](ros2/ros_cplusplus.md)** → **[Multi-Vehicle Setup](core/multi_vehicle.md)**
2. For autonomous systems: **[PX4 Integration](px4/px4_setup.md)**

### Game Developers & 3D Artists
1. **[Unreal Plugin](installation/install_precompiled.md)** → **[Environment Creation](unreal/unreal_custenv.md)** → **[Visual Features](unreal/lights.md)**

### System Integrators
1. **[Docker Setup](installation/docker_ubuntu.md)** → **[API Integration](core/apis.md)** → **[Production Deployment](installation/)**

## Original AirSim Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

## License

This project is released under the MIT License. Please review the [License file](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) for more details.
