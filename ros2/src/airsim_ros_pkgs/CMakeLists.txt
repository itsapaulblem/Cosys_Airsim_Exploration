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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosgraph_msgs REQUIRED)
<<<<<<< HEAD
=======

>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a

<<<<<<< HEAD
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
=======
# AirSim paths
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
>>>>>>> bcc393d6c (Update README and finalise code)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
<<<<<<< HEAD
<<<<<<< HEAD
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -fPIC -fPIE")
=======

# Suppress warnings from external libraries
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -Wno-class-memaccess -Wno-sign-compare")

>>>>>>> bcc393d6c (Update README and finalise code)
=======

# Suppress warnings from external libraries
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -Wno-class-memaccess -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel -fPIC -fPIE")

>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-lstdc++fs -fmax-errors=10")

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

# ADDED: Multirotor implementation library
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

# Phase 3: Service coordination and settings parser
add_library(vehicle_settings_parser src/vehicle_settings_parser.cpp)
target_link_libraries(vehicle_settings_parser AirLib)
ament_target_dependencies(vehicle_settings_parser
  rclcpp
)

# FIXED: Add coordination implementation library
add_library(coordination_impl src/coordination_node.cpp)
target_link_libraries(coordination_impl AirLib)
ament_target_dependencies(coordination_impl
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

#============================================================================
# EXECUTABLES
#============================================================================

# Legacy executables (keep for backward compatibility)
add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros AirLib)
ament_target_dependencies(airsim_node 
  rclcpp
)

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
<<<<<<< HEAD
# Test executable for concurrent RPC connections
add_executable(test_concurrent_rpc src/test_concurrent_rpc.cpp)
target_link_libraries(test_concurrent_rpc vehicle_node_factory AirLib)
ament_target_dependencies(test_concurrent_rpc 
=======

# Phase 2: Multi-node architecture executables
# FIXED: Link to multirotor_node_lib instead of vehicle_node_base
add_executable(multirotor_node src/multirotor_main.cpp)
target_link_libraries(multirotor_node multirotor_node_lib AirLib)
ament_target_dependencies(multirotor_node
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
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

#============================================================================
# INSTALL TARGETS
#============================================================================

# Install executables
install(TARGETS 
  # Legacy executables
  airsim_node 
  pd_position_controller_simple_node
  # Phase 2 executables
  multirotor_node
  simple_multirotor_node       # This was missing the executable definition above
  # Phase 3 executables
  coordination_node
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
  multirotor_node
  car_node
  computer_vision_node
  vehicle_node_factory
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib)
=======
# Phase 2: Multi-node architecture executables
# FIXED: Link to multirotor_node_lib instead of vehicle_node_base
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

#============================================================================
# INSTALL TARGETS
#============================================================================

# Install executables
install(TARGETS 
  # Legacy executables
  airsim_node 
  pd_position_controller_simple_node
  # Phase 2 executables
  multirotor_node
  simple_multirotor_node       # This was missing the executable definition above
  # Phase 3 executables
  coordination_node
  DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(TARGETS 
  # Legacy libraries
  airsim_ros 
  pd_position_controller_simple
  airsim_settings_parser
  # Phase 2 libraries
  vehicle_node_base
  multirotor_node_lib          # ADDED: Include multirotor implementation
  # Phase 3 libraries
  vehicle_settings_parser
  coordination_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
>>>>>>> bcc393d6c (Update README and finalise code)

=======
  multirotor_node_lib          # ADDED: Include multirotor implementation
  # Phase 3 libraries
  vehicle_settings_parser
  coordination_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
# Install additional files
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
#============================================================================
# EXPORT DEPENDENCIES
#============================================================================

# Core ROS2 dependencies
<<<<<<< HEAD
>>>>>>> bcc393d6c (Update README and finalise code)
=======
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
ament_export_dependencies(rclcpp)
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
=======
# Additional ROS2 dependencies
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(rclpy)
ament_export_dependencies(mavros_msgs)
ament_export_dependencies(rosgraph_msgs)
>>>>>>> bcc393d6c (Update README and finalise code)

# Build system dependencies
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_cpp)
ament_export_dependencies(ament_cmake)

# Custom interfaces
ament_export_dependencies(airsim_interfaces)

# Export include directories and libraries
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(
  # Legacy libraries
  airsim_settings_parser 
  pd_position_controller_simple
  airsim_ros 
  # Phase 2 libraries
  vehicle_node_base
<<<<<<< HEAD
  multirotor_node_lib          # ADDED: Include multirotor implementation
=======
  multirotor_node_lib
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
  # Phase 3 libraries
  vehicle_settings_parser
  coordination_impl
)

ament_package()