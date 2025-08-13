cmake_minimum_required(VERSION 3.12.0)
project(AirSim)

add_subdirectory("rpclib_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")

# Only build examples and tests if not in ROS2/Docker environment
if(NOT BUILD_AIRSIM_EXAMPLES STREQUAL "OFF" AND NOT EXISTS "/airsim_ros2_ws")
    add_subdirectory("AirLibUnitTests")
    add_subdirectory("HelloDrone")
    add_subdirectory("HelloSpawnedDrones")
    add_subdirectory("HelloCar")
    add_subdirectory("DroneShell")
    add_subdirectory("DroneServer")
endif()


