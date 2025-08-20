<<<<<<< HEAD
<<<<<<< HEAD
import setup_path
import cosysairsim as airsim
import sys
import time

# For high speed ascent and descent on PX4 you may need to set these properties:
# param set MPC_Z_VEL_MAX_UP 5
# param set MPC_Z_VEL_MAX_DN 5

z = 5
if len(sys.argv) > 1:
    z = float(sys.argv[1])

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()

print("make sure we are hovering at {} meters...".format(z))

if z > 5:
    # AirSim uses NED coordinates so negative axis is up.
    # z of -50 is 50 meters above the original launch point.
    client.moveToZAsync(-z, 5).join()
    client.hoverAsync().join()
    time.sleep(5)

if z > 10:
    print("come down quickly to 10 meters...")
    z = 10
    client.moveToZAsync(-z, 3).join()
    client.hoverAsync().join()

print("landing...")
client.landAsync().join()
print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")
=======
=======
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
import setup_path
import cosysairsim as airsim
import sys
import time

# For high speed ascent and descent on PX4 you may need to set these properties:
# param set MPC_Z_VEL_MAX_UP 5
# param set MPC_Z_VEL_MAX_DN 5

z = 5
if len(sys.argv) > 1:
    z = float(sys.argv[1])

<<<<<<< HEAD
client = airsim.MultirotorClient(ip="172.22.112.1")
client.confirmConnection()
client.enableApiControl(True, "Drone1")

client.armDisarm(True, "Drone1")

landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync(vehicle_name="Drone1").join()
else:
    print("already flying...")
    client.hoverAsync(vehicle_name="Drone1").join()
=======
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a

print("make sure we are hovering at {} meters...".format(z))

if z > 5:
    # AirSim uses NED coordinates so negative axis is up.
    # z of -50 is 50 meters above the original launch point.
    client.moveToZAsync(-z, 5).join()
    client.hoverAsync().join()
    time.sleep(5)

if z > 10:
    print("come down quickly to 10 meters...")
    z = 10
    client.moveToZAsync(-z, 3).join()
    client.hoverAsync().join()

print("landing...")
client.landAsync().join()
print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")
<<<<<<< HEAD
>>>>>>> bcc393d6c (Update README and finalise code)
=======

>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
