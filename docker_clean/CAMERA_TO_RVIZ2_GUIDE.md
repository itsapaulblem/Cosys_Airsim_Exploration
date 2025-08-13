#!/usr/bin/env python3
"""
Camera-Enhanced AirSim Settings Generator
Extends the unified generator with comprehensive camera support.
"""

import json
import sys
import os
from pathlib import Path
from typing import Dict, List, Optional
import argparse

def create_camera_config(name: str, image_types: List[int] = [0], 
                        width: int = 640, height: int = 480, 
                        fov: float = 90.0, position: Dict = None):
    """Create a camera configuration."""
    if position is None:
        position = {"X": 0.50, "Y": 0.00, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
    
    capture_settings = []
    for image_type in image_types:
        capture_settings.append({
            "ImageType": image_type,
            "Width": width,
            "Height": height,
            "FOV_Degrees": fov
        })
    
    return {
        "CaptureSettings": capture_settings,
        **position
    }

def get_camera_presets():
    """Get predefined camera configurations."""
    return {
        "front_rgb": {
            "image_types": [0],  # RGB
            "width": 1920, "height": 1080,
            "position": {"X": 0.50, "Y": 0.00, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "front_depth": {
            "image_types": [1],  # Depth
            "width": 640, "height": 480,
            "position": {"X": 0.50, "Y": 0.00, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "downward_cam": {
            "image_types": [0],  # RGB downward
            "width": 640, "height": 480,
            "position": {"X": 0.00, "Y": 0.00, "Z": 0.00, "Pitch": -90.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "segmentation_cam": {
            "image_types": [5],  # Segmentation
            "width": 640, "height": 480,
            "position": {"X": 0.50, "Y": 0.00, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "stereo_left": {
            "image_types": [0],  # RGB left
            "width": 640, "height": 480,
            "position": {"X": 0.50, "Y": -0.20, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "stereo_right": {
            "image_types": [0],  # RGB right
            "width": 640, "height": 480,
            "position": {"X": 0.50, "Y": 0.20, "Z": 0.10, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}
        },
        "gimbal_cam": {
            "image_types": [0],  # RGB gimbal
            "width": 1920, "height": 1080,
            "position": {"X": 0.30, "Y": 0.00, "Z": -0.20, "Pitch": -45.0, "Roll": 0.0, "Yaw": 0.0}
        }
    }

def create_subwindows(vehicle_cameras: Dict[str, Dict], vehicle_name: str):
    """Create SubWindows configuration for camera visualization in Unreal."""
    subwindows = []
    window_id = 0
    
    for camera_name, camera_config in vehicle_cameras.items():
        for capture_setting in camera_config["CaptureSettings"]:
            subwindows.append({
                "WindowID": window_id,
                "CameraName": camera_name,
                "ImageType": capture_setting["ImageType"],
                "VehicleName": vehicle_name,
                "Visible": True
            })
            window_id += 1
    
    return subwindows

def generate_camera_settings(num_drones: int = 1, camera_presets: List[str] = None,
                           enable_subwindows: bool = True):
    """Generate complete AirSim settings with camera configuration."""
    
    if camera_presets is None:
        camera_presets = ["front_rgb", "front_depth"]
    
    preset_configs = get_camera_presets()
    
    # Base settings
    settings = {
        "SettingsVersion": 2.0,
        "SimMode": "Multirotor",
        "ClockType": "SteppableClock",
        "PawnPaths": {
            "DefaultQuadrotor": {
                "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
            }
        },
        "Vehicles": {}
    }
    
    all_subwindows = []
    
    # Generate vehicles with cameras
    for i in range(1, num_drones + 1):
        vehicle_name = f"PX4_Drone{i}"
        
        # Vehicle base config
        vehicle_config = {
            "VehicleType": "PX4Multirotor",
            "X": (i-1) * 5, "Y": 0, "Z": -2,
            "UseSerial": False,
            "LockStep": True,
            "TcpPort": 4560 + i,
            "Cameras": {}
        }
        
        # Add cameras based on presets
        for preset_name in camera_presets:
            if preset_name in preset_configs:
                preset = preset_configs[preset_name]
                camera_config = create_camera_config(
                    name=preset_name,
                    image_types=preset["image_types"],
                    width=preset["width"],
                    height=preset["height"],
                    position=preset["position"]
                )
                vehicle_config["Cameras"][preset_name] = camera_config
        
        settings["Vehicles"][vehicle_name] = vehicle_config
        
        # Generate subwindows for this vehicle
        if enable_subwindows:
            vehicle_subwindows = create_subwindows(vehicle_config["Cameras"], vehicle_name)
            all_subwindows.extend(vehicle_subwindows)
    
    # Add subwindows to settings
    if enable_subwindows and all_subwindows:
        settings["SubWindows"] = all_subwindows
    
    return settings

def create_rviz_config(settings: Dict, output_path: Path):
    """Create RViz2 configuration file for camera visualization."""
    
    displays = [
        {
            "Alpha": 0.5,
            "Cell Size": 1,
            "Class": "rviz_default_plugins/Grid",
            "Color": "160; 160; 164",
            "Enabled": True,
            "Name": "Grid"
        },
        {
            "Class": "rviz_default_plugins/TF",
            "Enabled": True,
            "Name": "TF"
        }
    ]
    
    # Add image displays for each camera
    for vehicle_name, vehicle_config in settings["Vehicles"].items():
        if "Cameras" in vehicle_config:
            for camera_name, camera_config in vehicle_config["Cameras"].items():
                for capture_setting in camera_config["CaptureSettings"]:
                    image_type = capture_setting["ImageType"]
                    
                    # Map image type to topic suffix
                    type_mapping = {
                        0: "Scene",
                        1: "DepthPlanar", 
                        2: "DepthPerspective",
                        3: "DepthVis",
                        4: "DisparityNormalized",
                        5: "Segmentation",
                        6: "SurfaceNormals",
                        7: "Infrared"
                    }
                    
                    suffix = type_mapping.get(image_type, "Scene")
                    topic = f"/airsim_node/{vehicle_name}/{camera_name}/{suffix}"
                    
                    display = {
                        "Class": "rviz_default_plugins/Image",
                        "Enabled": True,
                        "Name": f"{vehicle_name} {camera_name} {suffix}",
                        "Topic": {
                            "Value": topic
                        },
                        "Transport Hint": "raw"
                    }
                    displays.append(display)
    
    rviz_config = {
        "Panels": [
            {
                "Class": "rviz_common/Displays",
                "Name": "Displays"
            }
        ],
        "Visualization Manager": {
            "Class": "",
            "Displays": displays,
            "Global Options": {
                "Background Color": "48; 48; 48",
                "Fixed Frame": "world_ned"
            },
            "Tools": [
                {"Class": "rviz_default_plugins/Interact"},
                {"Class": "rviz_default_plugins/MoveCamera"}
            ]
        }
    }
    
    rviz_file = output_path / "airsim_cameras.rviz"
    with open(rviz_file, 'w') as f:
        import yaml
        yaml.dump(rviz_config, f, default_flow_style=False)
    
    return rviz_file

def main():
    parser = argparse.ArgumentParser(description="Camera-Enhanced AirSim Settings Generator")
    
    parser.add_argument('--num-drones', type=int, default=1,
                       help='Number of drones (default: 1)')
    parser.add_argument('--cameras', nargs='+', 
                       choices=['front_rgb', 'front_depth', 'downward_cam', 'segmentation_cam', 
                               'stereo_left', 'stereo_right', 'gimbal_cam'],
                       default=['front_rgb', 'front_depth'],
                       help='Camera presets to include (default: front_rgb front_depth)')
    parser.add_argument('--no-subwindows', action='store_true',
                       help='Disable SubWindows in Unreal')
    parser.add_argument('--output-dir', type=str, default='.',
                       help='Output directory (default: current)')
    parser.add_argument('--create-rviz', action='store_true',
                       help='Create RViz2 configuration file')
    
    args = parser.parse_args()
    
    print("🎥 Camera-Enhanced AirSim Settings Generator")
    print("=" * 60)
    print(f"Drones: {args.num_drones}")
    print(f"Cameras: {', '.join(args.cameras)}")
    print(f"SubWindows: {'Disabled' if args.no_subwindows else 'Enabled'}")
    
    # Generate settings
    settings = generate_camera_settings(
        num_drones=args.num_drones,
        camera_presets=args.cameras,
        enable_subwindows=not args.no_subwindows
    )
    
    # Output settings
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    settings_file = output_path / "settings.json"
    with open(settings_file, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"✅ Generated: {settings_file}")
    
    # Copy to AirSim default location
    default_settings = Path.home() / "Documents" / "AirSim" / "settings.json"
    default_settings.parent.mkdir(parents=True, exist_ok=True)
    with open(default_settings, 'w') as f:
        json.dump(settings, f, indent=2)
    print(f"✅ Copied to: {default_settings}")
    
    # Create RViz config if requested
    if args.create_rviz:
        rviz_file = create_rviz_config(settings, output_path)
        print(f"✅ Generated: {rviz_file}")
    
    # Print camera topics that will be available
    print(f"\n📡 Available Camera Topics:")
    print("=" * 60)
    for vehicle_name, vehicle_config in settings["Vehicles"].items():
        if "Cameras" in vehicle_config:
            for camera_name, camera_config in vehicle_config["Cameras"].items():
                for capture_setting in camera_config["CaptureSettings"]:
                    image_type = capture_setting["ImageType"]
                    type_mapping = {0: "Scene", 1: "DepthPlanar", 5: "Segmentation"}
                    suffix = type_mapping.get(image_type, f"Type{image_type}")
                    topic = f"/airsim_node/{vehicle_name}/{camera_name}/{suffix}"
                    print(f"  {topic}")
    
    print(f"\n💡 Next Steps:")
    print("1. Start AirSim/Unreal with generated settings")
    print("2. Start ROS2 wrapper: docker/airsim_ros2_wrapper/run_vnc.bat")
    print("3. Access VNC: http://localhost:6901/vnc.html")
    print("4. Launch RViz2: rviz2")
    if args.create_rviz:
        print(f"5. Load config: rviz2 -d {output_path}/airsim_cameras.rviz")

if __name__ == "__main__":
    main()