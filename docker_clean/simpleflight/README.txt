SimpleFlight Configuration Generated
=====================================

This SimpleFlight configuration includes 5 drone(s).
SimpleFlight is AirSim's built-in flight controller - no external Docker containers required.

Usage:
1. Copy settings.json to your AirSim Documents folder (already done automatically)
2. Launch AirSim (Unreal Engine environment)
3. All 5 drones will be available for control via AirSim's Python/C++ APIs

Connection Info:
- All drones connect to AirSim server on localhost:4561
- Vehicle names: Drone1, Drone2, Drone3, Drone4, Drone5
- Use these names when connecting via airsim.MultirotorClient()

Python Client Example:
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
for drone_name in ['Drone1', 'Drone2', 'Drone3', 'Drone4', 'Drone5']:
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)
    client.takeoffAsync(vehicle_name=drone_name)

Benefits of SimpleFlight:
✅ No Docker setup required
✅ Fast startup and lightweight  
✅ Perfect for development and testing
✅ Supports up to 27 drones
✅ Built-in physics simulation
