cmake_minimum_required(VERSION 3.8)
project(airsim_mission_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/SearchArea.msg"
  "msg/GridParams.msg"
  "msg/SafetyParams.msg"
  "msg/SearchStats.msg"
  "msg/MissionProgress.msg"
  "msg/CurrentWaypoint.msg"
  "msg/Waypoint.msg"
  "msg/WaypointArray.msg"
  "msg/GridSearchStatus.msg"
  
  # Services
  "srv/UpdateMission.srv"
  "srv/GetMissionStatus.srv"
  "srv/GenerateGrid.srv"
  
  # Actions
  "action/GridSearch.action"
  "action/WaypointNavigation.action"
  
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()