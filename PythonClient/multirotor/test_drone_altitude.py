#!/usr/bin/env python3
"""
Check if drone altitude is causing LiDAR to be underground
"""

try:
    import cosysairsim as airsim
    print("Using cosysairsim")
except ImportError:
    try:
        import airsim
        print("Using airsim")
    except ImportError:
        print("Error: Neither cosysairsim nor airsim package found")
        exit(1)

def check_drone_position():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Check Drone1 position and LiDAR data
    try:
        state = client.getMultirotorState("Drone1")
        pos = state.kinematics_estimated.position
        
        print(f"Drone1 position:")
        print(f"  X: {pos.x_val:.2f}m")
        print(f"  Y: {pos.y_val:.2f}m") 
        print(f"  Z: {pos.z_val:.2f}m")
        
        # In AirSim, negative Z is up, so Z near 0 means on ground
        altitude = -pos.z_val
        print(f"  Altitude above ground: {altitude:.2f}m")
        
        # LiDAR sensor is at Z=-1 relative to drone
        lidar_altitude = altitude - 1.0
        print(f"  LiDAR sensor altitude: {lidar_altitude:.2f}m")
        
        if lidar_altitude < 0:
            print("❌ PROBLEM: LiDAR sensor is underground!")
            print("   This explains why you're getting origin points (0,0,0)")
            print("   Solution: Take off the drone first, then test LiDAR")
        else:
            print("✅ LiDAR sensor is above ground")
            
        # Also try to get LiDAR data and check pose
        lidar_data = client.getLidarData("LidarSensor1", "Drone1")
        lidar_pose = lidar_data.pose
        
        print(f"\nLiDAR sensor pose:")
        print(f"  Position: ({lidar_pose.position.x_val:.2f}, {lidar_pose.position.y_val:.2f}, {lidar_pose.position.z_val:.2f})")
        print(f"  Points in cloud: {len(lidar_data.point_cloud) // 3}")
        
        # Check if all points are at origin
        if len(lidar_data.point_cloud) >= 3:
            # Check first few points
            non_origin_count = 0
            for i in range(0, min(30, len(lidar_data.point_cloud)), 3):
                x, y, z = lidar_data.point_cloud[i], lidar_data.point_cloud[i+1], lidar_data.point_cloud[i+2]
                if abs(x) > 0.01 or abs(y) > 0.01 or abs(z) > 0.01:
                    non_origin_count += 1
            
            print(f"Non-origin points in first 10: {non_origin_count}/10")
            
            if non_origin_count == 0:
                print("❌ All points are at origin - sensor likely underground or no objects detected")
            else:
                print("✅ Some valid detections found")
                
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    check_drone_position()