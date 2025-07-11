#!/usr/bin/env python3
"""
Test script to debug camera issues and test different camera configurations.
This helps identify which camera names work in your setup.
"""

import os
import sys
from datetime import datetime

# Add parent directory to path for AirSim imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import cosysairsim as airsim
import numpy as np
import cv2

def test_cameras():
    """Test different camera configurations to find what works."""
    print("Camera Configuration Test")
    print("=" * 40)
    
    try:
        # Connect to AirSim
        print("Connecting to AirSim...")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        # Test different camera names
        camera_names = ["0", "1", "front_center", "front_left", "front_right", "fpv"]
        working_cameras = []
        
        for camera_name in camera_names:
            print(f"\n🔍 Testing camera: '{camera_name}'")
            
            try:
                # Try to get camera info first
                try:
                    camera_info = client.simGetCameraInfo(camera_name)
                    print(f"  ✅ Camera info retrieved:")
                    print(f"     Position: ({camera_info.pose.position.x_val:.2f}, {camera_info.pose.position.y_val:.2f}, {camera_info.pose.position.z_val:.2f})")
                    print(f"     FOV: {camera_info.fov:.1f}°")
                except Exception as e:
                    print(f"  ❌ Camera info failed: {str(e)}")
                    continue
                
                # Try to capture image
                responses = client.simGetImages([
                    airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
                ])
                
                if responses and len(responses) > 0:
                    response = responses[0]
                    print(f"  ✅ Image captured successfully:")
                    print(f"     Type: {response.image_type}")
                    print(f"     Size: {response.width}x{response.height}")
                    print(f"     Pixels as float: {response.pixels_as_float}")
                    print(f"     Compress: {response.compress}")
                    
                    # Test saving using the same logic as hello_drone.py
                    test_dir = os.path.join(os.path.dirname(__file__), "camera_test")
                    os.makedirs(test_dir, exist_ok=True)
                    
                    timestamp = datetime.now().strftime("%H%M%S")
                    filename_base = f"test_{camera_name}_{timestamp}"
                    
                    success = False
                    
                    if response.pixels_as_float:
                        # Float images - save as PFM
                        print(f"     Float data size: {len(response.image_data_float)}")
                        filepath = os.path.join(test_dir, filename_base + '.pfm')
                        airsim.write_pfm(os.path.normpath(filepath), airsim.get_pfm_array(response))
                        if os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                            print(f"  ✅ Test image saved (PFM): {filename_base}.pfm ({os.path.getsize(filepath)} bytes)")
                            success = True
                        
                    elif response.compress:  # PNG format - compressed
                        # Compressed PNG images
                        print(f"     Compressed PNG data size: {len(response.image_data_uint8)}")
                        filepath = os.path.join(test_dir, filename_base + '.png')
                        if len(response.image_data_uint8) > 0:
                            airsim.write_file(os.path.normpath(filepath), response.image_data_uint8)
                            if os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                                print(f"  ✅ Test image saved (compressed PNG): {filename_base}.png ({os.path.getsize(filepath)} bytes)")
                                success = True
                            else:
                                print(f"  ❌ Compressed PNG save failed")
                        else:
                            print(f"  ❌ Empty compressed image data")
                            
                    else:  # Uncompressed array
                        # Uncompressed RGB arrays
                        print(f"     Uncompressed array data size: {len(response.image_data_uint8)}")
                        filepath = os.path.join(test_dir, filename_base + '.png')
                        if len(response.image_data_uint8) > 0:
                            # Convert to numpy array and save with cv2
                            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                            img_rgb = img1d.reshape(response.height, response.width, 3)  # H x W x 3
                            
                            # Use cv2 to write PNG
                            cv2_success = cv2.imwrite(os.path.normpath(filepath), img_rgb)
                            
                            if cv2_success and os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                                print(f"  ✅ Test image saved (uncompressed PNG): {filename_base}.png ({os.path.getsize(filepath)} bytes)")
                                success = True
                            else:
                                print(f"  ❌ Uncompressed PNG save failed")
                        else:
                            print(f"  ❌ Empty uncompressed image data")
                    
                    if success:
                        working_cameras.append(camera_name)
                        print(f"  ✅ Camera '{camera_name}' is WORKING")
                    else:
                        print(f"  ❌ Camera '{camera_name}' failed to save image")
                        
                else:
                    print(f"  ❌ No image response")
                    
            except Exception as e:
                print(f"  ❌ Camera test failed: {str(e)}")
        
        print(f"\n📋 SUMMARY:")
        if working_cameras:
            print(f"✅ Working cameras found: {', '.join(working_cameras)}")
            print(f"🎯 Recommended: Use --camera_name \"{working_cameras[0]}\" in your missions")
            print(f"📁 Test images saved in: camera_test/")
        else:
            print(f"❌ No working cameras found!")
            print(f"🔧 Troubleshooting:")
            print(f"   - Make sure AirSim is running")
            print(f"   - Make sure a drone is spawned in the simulation")
            print(f"   - Check your AirSim settings.json for camera configuration")
        
    except Exception as e:
        print(f"❌ Connection failed: {str(e)}")
        print(f"Make sure AirSim is running and the drone is spawned.")

if __name__ == "__main__":
    test_cameras() 