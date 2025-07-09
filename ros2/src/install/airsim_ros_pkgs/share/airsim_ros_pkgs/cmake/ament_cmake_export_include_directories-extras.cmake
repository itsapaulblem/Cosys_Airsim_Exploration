# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${airsim_ros_pkgs_DIR}/../../../include;${airsim_ros_pkgs_DIR}/../../../ /home/paulc/Cosys-AirSim/ros2/src/airsim_ros_pkgs/../../..//external/rpclib/rpclib-2.3.1/include;/home/paulc/Cosys-AirSim/ros2/src/airsim_ros_pkgs/../../..//AirLib/deps/eigen3;/home/paulc/Cosys-AirSim/ros2/src/airsim_ros_pkgs/../../..//AirLib/include;/home/paulc/Cosys-AirSim/ros2/src/airsim_ros_pkgs/../../..//MavLinkCom/include;/home/paulc/Cosys-AirSim/ros2/src/airsim_ros_pkgs/../../..//MavLinkCom/common_utils;/usr/include/opencv4")

# append include directories to airsim_ros_pkgs_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'airsim_ros_pkgs' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND airsim_ros_pkgs_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
