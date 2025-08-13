#!/usr/bin/env python3

import os
import sys
import time
import setup_path

# Add parent directory to path for AirSim imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import airsim

def test_airsim_connection():
    """Test connection to AirSim and display basic information."""
    print("=" * 50)
    print("🔧 AIRSIM CONNECTION TEST")
    print("=" * 50)
    
    try:
        print("\n📡 Attempting to connect to AirSim...")
        
        # Create client and connect
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("✅ Successfully connected to AirSim!")
        
        # Get basic information
        print("\n📊 SYSTEM INFORMATION")
        
        # API version
        try:
            api_version = client.getVersion()
            print(f"AirSim API Version: {api_version}")
        except:
            print("AirSim API Version: Unknown")
        
        # Check if API control is enabled
        client.enableApiControl(True)
        print("✅ API control enabled")
        
        # Get drone state
        state = client.getMultirotorState()
        position = state.kinematics_estimated.position
        
        print(f"\n🚁 DRONE STATUS")
        print(f"Position: ({position.x_val:.2f}, {position.y_val:.2f}, {position.z_val:.2f})")
        # print(f"Armed: {state.armed}")
        print(f"Landed: {state.landed_state}")
        
        # Test image capture
        print(f"\n📸 TESTING IMAGE CAPTURE")
        try:
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ])
            
            if responses and len(responses) > 0:
                response = responses[0]
                print(f"✅ Image capture successful")
                print(f"Image size: {response.width}x{response.height}")
                print(f"Image format: {'Float' if response.pixels_as_float else 'Uint8'}")
            else:
                print("⚠️  Image capture returned empty response")
                
        except Exception as e:
            print(f"❌ Image capture failed: {str(e)}")
        
        # Cleanup
        client.enableApiControl(False)
        
        print(f"\n🎉 CONNECTION TEST COMPLETED SUCCESSFULLY!")
        print(f"💡 Your system is ready to run drone missions.")
        
        return True
        
    except Exception as e:
        print(f"\n❌ CONNECTION FAILED: {str(e)}")
        print(f"\n🔧 TROUBLESHOOTING TIPS:")
        print(f"1. Make sure AirSim/Unreal Engine is running")
        print(f"2. Check that AirSim is configured for multirotor")
        print(f"3. Verify no firewall is blocking the connection")
        print(f"4. Try restarting AirSim and running this test again")
        
        return False

def main():
    """Main function to run the connection test."""
    success = test_airsim_connection()
    
    if success:
        print(f"\n🚀 Ready to run missions! Try:")
        print(f"   python box_mission.py --preview")
        print(f"   python spiral_search_mission.py --preview")
        print(f"   python grid_survey_mission.py --preview")
    else:
        print(f"\n⚠️  Please fix connection issues before running missions.")
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main()) 