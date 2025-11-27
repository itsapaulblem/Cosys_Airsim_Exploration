#!/usr/bin/env python3
"""
Debug script to check why RViz2 isn't displaying LiDAR points
Checks TF frames, point cloud data, and RViz configuration
"""

import subprocess
import sys
import time

def run_command(cmd, timeout=5):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "Command timed out", -1

def check_tf_frames():
    """Check available TF frames"""
    print("="*60)
    print("TF Frame Check")
    print("="*60)
    
    # List all TF frames
    print("\n1. Available TF frames:")
    stdout, stderr, _ = run_command("ros2 run tf2_ros tf2_echo --list")
    if stdout:
        print(stdout)
    else:
        print("   No TF frames found or tf2_ros not available")
    
    # Check specific transforms
    print("\n2. Looking for Drone_1 related transforms:")
    stdout, stderr, _ = run_command("ros2 topic echo /tf --once", timeout=3)
    if "Drone_1" in stdout:
        print("   ✓ Found Drone_1 transforms")
    else:
        print("   ✗ No Drone_1 transforms found")
    
    # Check static transforms
    print("\n3. Static transforms:")
    stdout, stderr, _ = run_command("ros2 topic echo /tf_static --once", timeout=3)
    if "LidarSensor1" in stdout:
        print("   ✓ Found LidarSensor1 static transform")
    else:
        print("   ✗ No LidarSensor1 static transform")
        
def check_pointcloud_details():
    """Get detailed info about the point cloud topic"""
    print("\n" + "="*60)
    print("PointCloud2 Topic Details")
    print("="*60)
    
    vehicle = "Drone_1"
    sensor = "LidarSensor1"
    topic = f"/{vehicle}/{sensor}/points"
    
    print(f"\nTopic: {topic}")
    
    # Check topic info
    print("\n1. Topic info:")
    stdout, stderr, _ = run_command(f"ros2 topic info {topic}")
    print(stdout if stdout else "   Topic not found")
    
    # Check publishing rate
    print("\n2. Publishing rate:")
    stdout, stderr, _ = run_command(f"ros2 topic hz {topic}", timeout=3)
    if stdout:
        lines = stdout.split('\n')
        for line in lines:
            if "average rate" in line:
                print(f"   {line.strip()}")
                break
    else:
        print("   Unable to determine rate")

def suggest_rviz_config():
    """Suggest RViz2 configuration"""
    print("\n" + "="*60)
    print("RViz2 Configuration Suggestions")
    print("="*60)
    
    print("\nTo display the point cloud in RViz2:")
    print("\n1. Add PointCloud2 display:")
    print("   - Click 'Add' → 'By topic' → '/Drone_1/LidarSensor1/points'")
    print("   - Or manually set Topic to: /Drone_1/LidarSensor1/points")
    
    print("\n2. Set Fixed Frame:")
    print("   Try these in order until one works:")
    print("   a) 'world_ned' (if coordination node is running)")
    print("   b) 'Drone_1/odom_local_ned'") 
    print("   c) 'Drone_1_base_link'")
    print("   d) 'Drone_1/LidarSensor1' (sensor frame directly)")
    
    print("\n3. PointCloud2 Display Settings:")
    print("   - Style: Points or Flat Squares")
    print("   - Size (m): 0.01 to 0.05")
    print("   - Color Transformer: FlatColor or AxisColor")
    print("   - Position Transformer: XYZ")
    
    print("\n4. If still no points visible:")
    print("   - Reliability Policy: Best Effort")
    print("   - Durability Policy: Volatile")
    print("   - History Policy: Keep Last")
    print("   - Depth: 5")
    
def check_ros2_nodes():
    """Check which ROS2 nodes are running"""
    print("\n" + "="*60)
    print("ROS2 Nodes Check")
    print("="*60)
    
    stdout, stderr, _ = run_command("ros2 node list")
    if stdout:
        nodes = stdout.strip().split('\n')
        print(f"\nFound {len(nodes)} nodes:")
        for node in nodes:
            print(f"  - {node}")
            
        # Check for coordination node
        if any("coordination" in node for node in nodes):
            print("\n✓ Coordination node is running (provides world_ned frame)")
        else:
            print("\n✗ No coordination node found (world_ned frame may be missing)")
            
        # Check for vehicle nodes
        drone_nodes = [n for n in nodes if "Drone" in n]
        if drone_nodes:
            print(f"\n✓ Found {len(drone_nodes)} drone node(s)")
        else:
            print("\n✗ No drone nodes found")
    else:
        print("No nodes found or ROS2 not running")

def generate_rviz_config():
    """Generate a working RViz2 config file"""
    print("\n" + "="*60)
    print("RViz2 Config File")
    print("="*60)
    
    config = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Plane Cell Count: 100
      Line Style:
        Value: Lines
      Color: [160, 160, 164]
      Alpha: 0.5
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
      Frame Timeout: 15
      Show Names: true
      Show Axes: true
      Show Arrows: false
      Marker Scale: 1
    - Class: rviz_default_plugins/PointCloud2
      Name: Drone_1 LiDAR
      Topic: /Drone_1/LidarSensor1/points
      Reliability Policy: Best Effort
      Durability Policy: Volatile
      History Policy: Keep Last
      Depth: 5
      Style: Flat Squares
      Size (m): 0.03
      Color Transformer: FlatColor
      Color: [255, 255, 0]
      Position Transformer: XYZ
      Alpha: 1
      Use Fixed Frame: true
      Use Rainbow: false
  Global Options:
    Fixed Frame: world_ned
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20
      Focal Point: [0, 0, 0]
      Pitch: 0.5
      Yaw: 0.5
Window Geometry:
  Width: 1200
  Height: 800"""
    
    print("\nSave this configuration as 'airsim_lidar.rviz' and load it in RViz2:")
    print("-" * 40)
    print(config)
    print("-" * 40)
    print("\nTo use: rviz2 -d airsim_lidar.rviz")

def main():
    print("RViz2 PointCloud Debug Tool")
    print("="*60)
    
    # Run all checks
    check_ros2_nodes()
    check_tf_frames()
    check_pointcloud_details()
    suggest_rviz_config()
    
    print("\n" + "="*60)
    print("Quick Fixes to Try")
    print("="*60)
    print("\n1. Check Fixed Frame in RViz2 (top left):")
    print("   ros2 run tf2_ros tf2_echo world_ned Drone_1_base_link")
    print("\n2. Verify transforms are being published:")
    print("   ros2 topic hz /tf")
    print("\n3. Test with simple visualization:")
    print("   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix airsim_ros_pkgs)/share/airsim_ros_pkgs/rviz/default.rviz")
    print("\n4. If using multi-node architecture, ensure coordination node is running:")
    print("   ros2 run airsim_ros_pkgs airsim_coordination_node")

if __name__ == "__main__":
    main()