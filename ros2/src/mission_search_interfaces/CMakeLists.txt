cmake_minimum_required(VERSION 3.8)
project(mission_search_interfaces)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Interface files
set(MSG_FILES
  "msg/MissionStatus.msg"
  "msg/SearchProgress.msg"
  "msg/TargetDetection.msg"
  "msg/SearchZone.msg"
  "msg/VehicleCapabilities.msg"
  "msg/MissionPlan.msg"
  "msg/WaypointArray.msg"
  "msg/MissionEvent.msg"
)

set(SRV_FILES
  "srv/SetSearchPattern.srv"
  "srv/AssignSearchZone.srv"
  "srv/GetMissionStatus.srv"
  "srv/GetVehicleCapabilities.srv"
  "srv/PlanMission.srv"
  "srv/ValidateSearchArea.srv"
)

set(ACTION_FILES
  "action/SearchArea.action"
  "action/NavigateToTarget.action"
  "action/TrackTarget.action"
  "action/ExecuteMission.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()