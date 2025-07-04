import setup_path
import cosysairsim as airsim
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Get drone state
state = client.getMultirotorState()
print(f"Drone position: {state.kinematics_estimated.position}")
print(f"Drone orientation: {state.kinematics_estimated.orientation}")
print("Connection successful!") 