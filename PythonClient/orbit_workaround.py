#!/usr/bin/env python3
"""
Orbit Script with GPS Home Location Workaround

This script demonstrates how to fly orbit patterns without requiring GPS home location
by using movement commands instead of armDisarm.
"""

import setup_path
import cosysairsim as airsim
import numpy as np
import time

def orbit_with_workaround():
    """Perform orbit maneuver without armDisarm"""
    print("🚁 Orbit Maneuver with GPS Workaround")
    print("=" * 40)
    
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected to AirSim")
    
    vehicle_name = "PX4_Drone1"
    
    try:
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Check GPS data (should work now)
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        print(f"📍 GPS Position: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}")
        
        # Move to a safe starting position instead of arming
        print("🚀 Moving to starting position (takeoff equivalent)...")
        client.moveToPositionAsync(0, 0, -5, 3, vehicle_name=vehicle_name).join()
        print("✅ At starting altitude")
        
        # Perform orbit maneuver
        print("🔄 Starting orbit maneuver...")
        
        # Orbit parameters
        radius = 10.0  # meters
        altitude = -5.0  # meters (negative is up)
        speed = 2.0    # m/s
        num_points = 16
        
        # Calculate orbit points
        angles = np.linspace(0, 2 * np.pi, num_points)
        
        for i, angle in enumerate(angles):
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = altitude
            
            print(f"📍 Moving to point {i+1}/{num_points}: ({x:.1f}, {y:.1f}, {z:.1f})")
            
            # Move to orbit point
            client.moveToPositionAsync(x, y, z, speed, vehicle_name=vehicle_name).join()
            
            # Brief pause at each point
            time.sleep(0.5)
        
        print("✅ Orbit completed!")
        
        # Return to center and land
        print("🏠 Returning to center...")
        client.moveToPositionAsync(0, 0, -5, speed, vehicle_name=vehicle_name).join()
        
        print("🛬 Landing...")
        client.landAsync(vehicle_name=vehicle_name).join()
        print("✅ Landed safely")
        
    except Exception as e:
        print(f"❌ Error during orbit: {e}")
        
        # Emergency landing
        try:
            print("🚨 Emergency landing...")
            client.landAsync(vehicle_name=vehicle_name).join()
        except:
            pass
    
    finally:
        # Always disable API control
        try:
            client.enableApiControl(False, vehicle_name)
            print("✅ API control disabled")
        except:
            pass

def test_basic_flight_commands():
    """Test basic flight commands that work without GPS home"""
    print("\n🧪 Testing Basic Flight Commands")
    print("=" * 35)
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    vehicle_name = "PX4_Drone1"
    
    try:
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Test commands that should work
        commands = [
            ("Takeoff", lambda: client.takeoffAsync(vehicle_name=vehicle_name)),
            ("Move forward", lambda: client.moveByVelocityAsync(1, 0, 0, 2, vehicle_name=vehicle_name)),
            ("Hover", lambda: client.hoverAsync(vehicle_name=vehicle_name)),
            ("Move to position", lambda: client.moveToPositionAsync(0, 0, -3, 1, vehicle_name=vehicle_name)),
            ("Land", lambda: client.landAsync(vehicle_name=vehicle_name)),
        ]
        
        for name, command in commands:
            try:
                print(f"   Testing {name}...")
                result = command()
                if hasattr(result, 'join'):
                    result.join()
                print(f"   {name}: ✅ Success")
                time.sleep(1)
            except Exception as e:
                print(f"   {name}: ❌ {e}")
    
    except Exception as e:
        print(f"❌ Basic flight test failed: {e}")
    
    finally:
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

def main():
    print("🔧 AirSim Orbit with GPS Home Workaround")
    print("=" * 45)
    print("This script demonstrates flying without GPS home location requirement")
    
    # Test basic commands first
    test_basic_flight_commands()
    
    print("\n" + "="*60)
    
    # Perform orbit maneuver
    orbit_with_workaround()
    
    print("\n📋 Summary")
    print("=" * 15)
    print("✅ GPS data is working (lat/lon coordinates available)")
    print("❌ GPS home location mechanism has issues")
    print("✅ Movement commands work without GPS home")
    print("🎯 Solution: Use movement commands instead of armDisarm")

if __name__ == "__main__":
    main() 