#!/bin/bash
# Quick ROS2 development aliases

alias launch-mission='ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true'
alias launch-rviz='rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz'
alias monitor-drones='ros2 topic list | grep -E "(PX4_Drone|odom|mission)"'