import sys
import os
import time
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim 

client = airsim.MultirotorClient()
client.confirmConnection()
vehicles = client.listVehicles()
print('Discovered vehicles:', vehicles)
for vehicle in vehicles:
    try:
        state = client.getMultirotorState(vehicle)
        pos = state
        print(f'{vehicle}: [{pos.x():.3f}, {pos.y():.3f}, {pos.z():.3f}]')
    except Exception as e:
        print(f'{vehicle}: ERROR - {e}')
