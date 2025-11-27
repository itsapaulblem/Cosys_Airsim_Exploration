cmake_minimum_required(VERSION 3.12.0)
project(airsim_ros_pkgs)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_extras REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(mission_search_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosgraph_msgs REQUIRED)

<<<<<<< HEAD
=======

>>>>>>> main
# Set AIRSIM_ROOT to the workspace directory where AirLib, MavLinkCom, etc. are located
# For Docker environment, use absolute path
if(EXISTS "/airsim_ros2_ws/cmake")
    set(AIRSIM_ROOT "/airsim_ros2_ws")
else()
    # Fallback to relative path for local development
    set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
endif()

# Only add the required subdirectories for ROS2, not the examples/tests
# Set CMAKE variable to prevent building examples in subdirectories
set(BUILD_AIRSIM_EXAMPLES OFF CACHE BOOL "Disable AirSim examples" FORCE)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

# Compiler settings
<<<<<<< HEAD
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -fPIC -fPIE")

set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-lstdc++fs -fmax-errors=10")
=======
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -Wno-class-memaccess -Wno-sign-compare -fPIC")
>>>>>>> main

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# Include directories
set(INCLUDE_DIRS include 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${INCLUDE_DIRS})

#============================================================================
# LIBRARIES
#============================================================================

# Legacy libraries (keep for backward compatibility)
add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
target_link_libraries(pd_position_controller_simple AirLib)
ament_target_dependencies(pd_position_controller_simple 
  rclcpp
  nav_msgs
  geometry_msgs
  airsim_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)




add_library(airsim_ros src/airsim_ros_wrapper.cpp)
target_link_libraries(airsim_ros ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)
ament_target_dependencies(airsim_ros 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  tf2_ros
  cv_bridge
  airsim_interfaces
  mavros_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2
  pcl_conversions
  PCL
)

# Phase 2: Multi-node architecture libraries
add_library(vehicle_node_base src/vehicle_node_base.cpp)
target_link_libraries(vehicle_node_base AirLib)
ament_target_dependencies(vehicle_node_base
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

<<<<<<< HEAD
# Multirotor implementation library
=======
# ADDED: Multirotor implementation library
>>>>>>> main
add_library(multirotor_node_lib src/multirotor_node.cpp)
target_link_libraries(multirotor_node_lib vehicle_node_base AirLib)
ament_target_dependencies(multirotor_node_lib
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

<<<<<<< HEAD
=======
# ADDED: Mission-capable multirotor library
add_library(mission_multirotor_node_lib src/mission_multirotor_node.cpp)
target_link_libraries(mission_multirotor_node_lib multirotor_node_lib vehicle_node_base AirLib)
ament_target_dependencies(mission_multirotor_node_lib
  rclcpp
  rclcpp_action
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
  mission_search_interfaces
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

>>>>>>> main
# Phase 3: Service coordination and settings parser
add_library(vehicle_settings_parser src/vehicle_settings_parser.cpp)
target_link_libraries(vehicle_settings_parser AirLib)
ament_target_dependencies(vehicle_settings_parser
  rclcpp
)

<<<<<<< HEAD
# Coordination implementation library
add_library(coordination_impl src/coordination_node.cpp)
target_link_libraries(coordination_impl AirLib)
ament_target_dependencies(coordination_impl
  rclcpp
  airsim_interfaces
  std_srvs
=======
# FIXED: Add coordination implementation library including mission coordination
add_library(coordination_impl
  src/coordination_node.cpp
  src/mission_coordination_node.cpp)
target_link_libraries(coordination_impl vehicle_node_base AirLib)  # Link vehicle_node_base for GPS mutex
ament_target_dependencies(coordination_impl
  rclcpp
  rclcpp_action
  airsim_interfaces
  mission_search_interfaces
  std_srvs
  std_msgs
>>>>>>> main
  rosgraph_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

#============================================================================
# EXECUTABLES
#============================================================================

# Legacy executables (keep for backward compatibility)
# TODO: Fix constructor signature mismatch
# add_executable(airsim_node src/airsim_node.cpp)
# target_link_libraries(airsim_node airsim_ros AirLib)
<<<<<<< HEAD
# ament_target_dependencies(airsim_node 
=======
# ament_target_dependencies(airsim_node
>>>>>>> main
#   rclcpp
# )

add_executable(pd_position_controller_simple_node
  src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node
  pd_position_controller_simple airsim_ros AirLib)
ament_target_dependencies(pd_position_controller_simple_node 
  rclcpp
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

<<<<<<< HEAD
# Phase 2: Multi-node architecture executables
add_executable(multirotor_node src/multirotor_main.cpp)
target_link_libraries(multirotor_node multirotor_node_lib AirLib)
ament_target_dependencies(multirotor_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

# Simple single drone executable (bypasses complex base class)
add_executable(simple_multirotor_node src/simple_multirotor_node.cpp)
target_link_libraries(simple_multirotor_node AirLib)
ament_target_dependencies(simple_multirotor_node
=======

# Phase 2: Multi-node architecture executables
# FIXED: Link to multirotor_node_lib instead of vehicle_node_base
add_executable(multirotor_node src/multirotor_main.cpp)
target_link_libraries(multirotor_node multirotor_node_lib AirLib)
ament_target_dependencies(multirotor_node
>>>>>>> main
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
<<<<<<< HEAD
)

=======
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

# ADDED: Simple single drone executable (bypasses complex base class)
add_executable(simple_multirotor_node src/simple_multirotor_node.cpp)
target_link_libraries(simple_multirotor_node AirLib)
ament_target_dependencies(simple_multirotor_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
)

# ADDED: Mission-capable multirotor executable
add_executable(mission_multirotor_node src/mission_multirotor_main.cpp)
target_link_libraries(mission_multirotor_node mission_multirotor_node_lib AirLib)
ament_target_dependencies(mission_multirotor_node
  rclcpp
  rclcpp_action
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  airsim_interfaces
  mission_search_interfaces
  cv_bridge
  image_transport
  pcl_conversions
  PCL
)

>>>>>>> main
# Phase 3: Coordination node
add_executable(coordination_node src/coordination_main.cpp)
target_link_libraries(coordination_node coordination_impl AirLib)
ament_target_dependencies(coordination_node
  rclcpp
  airsim_interfaces
  std_srvs
  rosgraph_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

<<<<<<< HEAD
=======
# Phase 3.5: Mission coordination node (C++ implementation with fixed interface mismatches)
add_executable(mission_coordination_node src/mission_coordination_main.cpp)
target_link_libraries(mission_coordination_node coordination_impl AirLib)
ament_target_dependencies(mission_coordination_node
  rclcpp
  rclcpp_action
  airsim_interfaces
  mission_search_interfaces
  std_srvs
  std_msgs
  rosgraph_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)


# REP 105 Compliant Localization Node
add_executable(localization_node src/localization_main.cpp src/localization_node.cpp)
target_link_libraries(localization_node vehicle_node_base AirLib)  # Link vehicle_node_base for GPS mutex
target_include_directories(localization_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${AIRSIM_ROOT}
)
ament_target_dependencies(localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

# Point Cloud Merger for OctoMap (4-drone LiDAR merging)
add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp)
target_link_libraries(pointcloud_merger_node ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_merger_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_sensor_msgs
)


>>>>>>> main
#============================================================================
# INSTALL TARGETS
#============================================================================

# Install executables
<<<<<<< HEAD
install(TARGETS 
=======
install(TARGETS
>>>>>>> main
  # Legacy executables
  # airsim_node  # TODO: Fix constructor signature mismatch
  pd_position_controller_simple_node
  # Phase 2 executables
  multirotor_node
<<<<<<< HEAD
  simple_multirotor_node
  # Phase 3 executables
  coordination_node
=======
  simple_multirotor_node       # This was missing the executable definition above
  mission_multirotor_node      # Mission-capable multirotor node
  # Phase 3 executables
  coordination_node
  mission_coordination_node    # Mission coordination node

  # REP 105 Compliance
  localization_node            # REP 105 compliant localization component

  # OctoMap 3D Mapping
  pointcloud_merger_node       # Multi-drone LiDAR point cloud merger
>>>>>>> main
  DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(TARGETS 
  # Legacy libraries
  airsim_ros 
  pd_position_controller_simple
  airsim_settings_parser
  # Phase 2 libraries
  vehicle_node_base
<<<<<<< HEAD
  multirotor_node_lib
=======
  multirotor_node_lib          # ADDED: Include multirotor implementation
  mission_multirotor_node_lib  # Mission-capable multirotor library
>>>>>>> main
  # Phase 3 libraries
  vehicle_settings_parser
  coordination_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

<<<<<<< HEAD
# Install Python scripts
install(PROGRAMS
  scripts/motion_detection_node.py
  scripts/generalised_object_tracking_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install YOLOv7-DeepSORT directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/YOLOv7-DeepSORT-Object-Tracking")
  install(DIRECTORY
    scripts/YOLOv7-DeepSORT-Object-Tracking/
    DESTINATION lib/${PROJECT_NAME}/YOLOv7-DeepSORT-Object-Tracking
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
    PATTERN "*.git*" EXCLUDE
  )
endif()

# Install car_joy scripts if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/car_joy")
  install(DIRECTORY
    scripts/car_joy/
    DESTINATION lib/${PROJECT_NAME}/car_joy
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
endif()

# Install additional files
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

# Install Python package requirements
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
  install(FILES 
    requirements.txt
    DESTINATION share/${PROJECT_NAME}
  )
endif()
=======
# Install additional files
# install(FILES README.md DESTINATION share/${PROJECT_NAME})  # Commented out: README.md does not exist
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

# Install Python test scripts and demos
install(PROGRAMS
  src/test_mission_client.py
  src/demo_mission_planner.py
  src/simple_architecture_demo.py
  src/simple_mission_coordination_demo.py
  src/mission_coordination_demo_client.py
  scripts/motion_detection_node.py
  scripts/generalised_object_tracking_node.py
  scripts/drone_stats_logger.py
  DESTINATION lib/${PROJECT_NAME})
>>>>>>> main

#============================================================================
# EXPORT DEPENDENCIES
#============================================================================

# Core ROS2 dependencies
ament_export_dependencies(rclcpp)
<<<<<<< HEAD
ament_export_dependencies(rclpy)
=======
>>>>>>> main
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)

# Transform and coordinate frame dependencies
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_sensor_msgs)

# Image and point cloud processing dependencies
ament_export_dependencies(cv_bridge)
ament_export_dependencies(image_transport)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(PCL)

<<<<<<< HEAD
# Build system dependencies
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_cpp)
ament_export_dependencies(ament_cmake)

# Custom interfaces
ament_export_dependencies(airsim_interfaces)
=======
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(
  airsim_settings_parser 
  pd_position_controller_simple
  airsim_ros
  vehicle_node_base
  multirotor_node
  car_node
  computer_vision_node
  vehicle_node_factory
)

# Build system dependencies
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_cpp)
ament_export_dependencies(ament_cmake)

# Custom interfaces
ament_export_dependencies(airsim_interfaces)
ament_export_dependencies(mission_search_interfaces)
ament_export_dependencies(rclcpp_action)
>>>>>>> main

# Export include directories and libraries
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(
  # Legacy libraries
  airsim_settings_parser 
  pd_position_controller_simple
  airsim_ros 
  # Phase 2 libraries
  vehicle_node_base
  multirotor_node_lib
<<<<<<< HEAD
=======
  mission_multirotor_node_lib
>>>>>>> main
  # Phase 3 libraries
  vehicle_settings_parser
  coordination_impl
)

ament_package()