import sys
import os
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

print("🔍 Testing AirSim GPS Issue...")

try:
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ AirSim connection successful")
    
    # Test PX4_Drone1
    print("\n🔍 Testing PX4_Drone1...")
    try:
        state1 = client.getMultirotorState(vehicle_name="PX4_Drone1")
        pos = state1.kinematics_estimated.position
        print(f"✅ PX4_Drone1 state available")
        print(f"   Position: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
        
        # Test GPS data
        gps1 = client.getGpsData(vehicle_name="PX4_Drone1")
        print(f"✅ GPS data available: Lat={gps1.gnss.geo_point.latitude:.6f}, Lon={gps1.gnss.geo_point.longitude:.6f}")
        
        # Test if we can enable API control
        client.enableApiControl(True, "PX4_Drone1")
        print("✅ API control enabled")
        
        # This is where the error should occur
        print("🔧 Attempting to arm drone...")
        client.armDisarm(True, "PX4_Drone1")
        print("✅ Drone armed successfully!")
        
        # Clean up
        client.armDisarm(False, "PX4_Drone1")
        client.enableApiControl(False, "PX4_Drone1")
        
    except Exception as e:
        print(f"❌ PX4_Drone1 error: {str(e)}")
        
except Exception as e:
    print(f"❌ AirSim connection failed: {str(e)}") 