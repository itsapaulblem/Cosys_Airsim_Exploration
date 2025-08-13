import airsim
#!/usr/bin/env python3
import math
import time
import argparse
import numpy as np
import setup_path
import cosysairsim as airsim
import sys
import cv2


# client = airsim.VehicleClient()
client = airsim.MultirotorClient(ip="172.22.112.1")
client.confirmConnection()

objects = client.simListSceneObjects()
for ob in objects:
    print(ob)

object_name_list = ["road","building","Landscape","Sky","car","building","vegetation","grass","house","tree","Door","wall"]

print("Reset all object id")
found = client.simSetSegmentationObjectID("[\w]*", 0, True)
print("all object: %r" % (found))
time.sleep(1)

for idx,obj_name in enumerate(object_name_list):
    obj_name_reg = r"[\w]*" + obj_name + r"[\w]"
    found = client.simSetSegmentationObjectID(obj_name_reg, (idx + 1) % 256, True)
print("%s: %r" % (obj_name, found))