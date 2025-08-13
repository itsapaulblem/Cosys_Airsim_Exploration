# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import airsim

import pprint
import tempfile
import os
import time


def main():
    # Connect to AirSim
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    
    # state = client.getMultirotorState()
    # s = pprint.pformat(state)
    # print("state: %s" % s)

    # rotor_state_data = client.getRotorStates()
    # s = pprint.pformat(rotor_state_data)
    # print("Rotor Data: %s" % s)

    # barometer_data = client.getBarometerData()
    # s = pprint.pformat(barometer_data)
    # print("barometer_data: %s" % s)

    # magnetometer_data = client.getMagnetometerData()
    # s = pprint.pformat(magnetometer_data)
    # print("magnetometer_data: %s" % s)

    # gps_data = client.getGpsData()
    # s = pprint.pformat(gps_data)
    # print("gps_data: %s" % s)
    png_image = client.simGetImage("0", airsim.ImageType.Scene)
    responses = client.simGetImages([
        # png format
        airsim.ImageRequest(0, airsim.ImageType.Scene), 
        # uncompressed RGB array bytes
        airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
        # floating point uncompressed image
        airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])
    tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
    print ("Saving images to %s" % tmp_dir)
    # print('Retrieved images: %d' % len(responses))
    
    for idx, response in enumerate(responses):

        filename = os.path.join(tmp_dir, str(idx))

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress: #png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
            img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

    # airsim.wait_key('Press any key to reset to original state')

    # client.reset()
    # client.armDisarm(False)

    # # that's enough fun for now. let's quit cleanly
    # client.enableApiControl(False)
    
    # state = client.getMultirotorState()
    # print("state: %s" % pprint.pformat(state))

    airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
    client.moveToPositionAsync(-10, 10, -10, 5).join()

    client.hoverAsync().join()

if __name__ == "__main__":
    main() 