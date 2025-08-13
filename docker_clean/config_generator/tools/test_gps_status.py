#!/usr/bin/env python3
"""
Quick GPS status test for AirSim multi-drone setup
"""
import airsim
import time

def test_gps_status():
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicles = client.listVehicles()
        print(f"Found vehicles: {vehicles}")
        
        for vehicle in vehicles:
            try:
                gps_data = client.getGpsData(vehicle_name=vehicle)
                print(f"\n{vehicle} GPS Status:")
                print(f"  Fix Type: {gps_data.gnss.fix_type}")
                print(f"  Latitude: {gps_data.gnss.geo_point.latitude}")
                print(f"  Longitude: {gps_data.gnss.geo_point.longitude}")
                print(f"  Altitude: {gps_data.gnss.geo_point.altitude}")
                print(f"  Velocity: {gps_data.gnss.velocity}")
                
                if gps_data.gnss.fix_type >= 3:
                    print(f"  ✅ {vehicle} has 3D GPS fix")
                else:
                    print(f"  ⚠️  {vehicle} does not have GPS fix")
                    
            except Exception as e:
                print(f"  ❌ Error getting GPS data for {vehicle}: {e}")
                
    except Exception as e:
        print(f"❌ Could not connect to AirSim: {e}")
        print("Make sure AirSim is running!")

if __name__ == "__main__":
    test_gps_status()
