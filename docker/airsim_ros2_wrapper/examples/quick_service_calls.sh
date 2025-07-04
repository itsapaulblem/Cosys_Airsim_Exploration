#!/bin/bash

# Quick service call examples for AirSim custom services
# Copy and paste these commands for quick testing

echo "🚁 AirSim Custom Service Quick Commands"
echo "====================================="

echo "📋 Basic Commands:"
echo ""

echo "1️⃣ Change altitude to -20 meters (absolute):"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '"'"'{target_altitude: -20.0, climb_rate: 2.0, reference_frame: \"absolute\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"'"'"'
echo ""

echo "2️⃣ Climb 5 meters higher (relative):"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '"'"'{target_altitude: -5.0, climb_rate: 1.5, reference_frame: \"relative\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"'"'"'
echo ""

echo "3️⃣ Small circle around current position:"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '"'"'{radius: 5.0, altitude: -15.0, speed: 1.5, duration: 20.0, center_frame: \"body\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"'"'"'
echo ""

echo "4️⃣ Large circle around world origin:"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '"'"'{radius: 30.0, altitude: -25.0, speed: 3.0, duration: 60.0, center_frame: \"world\", vehicle_name: \"Drone1\", wait_on_last_task: false}'"'"'"'
echo ""

echo "5️⃣ Fast high circle:"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '"'"'{radius: 15.0, altitude: -35.0, speed: 5.0, duration: 30.0, center_frame: \"body\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"'"'"'
echo ""

echo "📊 Utility Commands:"
echo ""

echo "🔍 List all available services:"
echo './ros2_exec.bat "ros2 service list | grep Drone1"'
echo ""

echo "📋 Show service interface:"
echo './ros2_exec.bat "ros2 interface show airsim_interfaces/srv/FlyCircle"'
echo './ros2_exec.bat "ros2 interface show airsim_interfaces/srv/ChangeAltitude"'
echo ""

echo "📡 Monitor drone position:"
echo './ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom_local_ned --once"'
echo ""

echo "🛑 Emergency land:"
echo './ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '"'"'{wait_on_last_task: true}'"'"'"'
echo ""

echo "🔄 Reset simulation:"
echo './ros2_exec.bat "ros2 service call /airsim_node/reset std_srvs/srv/Empty"' 