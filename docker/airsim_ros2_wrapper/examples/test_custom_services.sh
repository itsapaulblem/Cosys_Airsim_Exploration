#!/bin/bash

# Test script for custom AirSim ROS2 services
# Make sure your drone is armed and ready before running these tests

echo "🚁 Testing Custom AirSim ROS2 Services"
echo "======================================"

# Check if services are available
echo "📋 Checking service availability..."
ros2 service list | grep -E "(fly_circle|change_altitude)"

if [ $? -ne 0 ]; then
    echo "❌ Custom services not found. Make sure the container is running with updated code."
    exit 1
fi

echo "✅ Services found!"
echo ""

# Test 1: Change altitude
echo "🔧 Test 1: Change altitude to -15 meters"
ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '{
    target_altitude: -15.0, 
    climb_rate: 2.0, 
    reference_frame: "absolute", 
    vehicle_name: "Drone1", 
    wait_on_last_task: true
}'

echo "⏳ Waiting 5 seconds..."
sleep 5

# Test 2: Fly in circle
echo "🔧 Test 2: Fly in circle (radius: 10m, speed: 2m/s)"
ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '{
    radius: 10.0, 
    altitude: -20.0, 
    speed: 2.0, 
    duration: 30.0, 
    center_frame: "body", 
    vehicle_name: "Drone1", 
    wait_on_last_task: true
}'

echo "⏳ Waiting 10 seconds to observe circle flight..."
sleep 10

# Test 3: Relative altitude change
echo "🔧 Test 3: Relative altitude change (+5 meters)"
ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '{
    target_altitude: -5.0, 
    climb_rate: 1.5, 
    reference_frame: "relative", 
    vehicle_name: "Drone1", 
    wait_on_last_task: false
}'

echo "⏳ Waiting 3 seconds..."
sleep 3

# Test 4: Large circle around world origin
echo "🔧 Test 4: Large circle around world origin"
ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '{
    radius: 25.0, 
    altitude: -25.0, 
    speed: 3.0, 
    duration: 60.0, 
    center_frame: "world", 
    vehicle_name: "Drone1", 
    wait_on_last_task: false
}'

echo "🎉 All tests completed!"
echo ""
echo "📊 You can monitor the drone's behavior in AirSim or RViz2"
echo "🛑 Use 'ros2 service call /airsim_node/Drone1/land' to land when done" 