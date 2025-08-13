#!/usr/bin/env python3
"""
AirSim Level Manager

Programmatically load different UE5 levels/environments based on mission requirements.
"""

import airsim
import time
from enum import Enum

class EnvironmentType(Enum):
    """Available AirSim environments"""
    NEIGHBORHOOD = "Neighborhood"
    LANDSCAPE_MOUNTAINS = "LandscapeMountains" 
    CITY = "CityEnviron"
    FOREST = "ForestEnviron"
    DESERT = "DesertEnviron"
    ARCTIC = "ArcticEnviron"
    BLOCKS = "Blocks"  # Simple test environment

class LevelManager:
    """Manages level loading and environment switching in AirSim"""
    
    def __init__(self):
        self.client = None
        self.current_level = None
        self.available_levels = self._get_available_levels()
        
    def connect(self):
        """Connect to AirSim"""
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            print(" Connected to AirSim")
            return True
        except Exception as e:
            print(f" Failed to connect to AirSim: {e}")
            return False
    
    def _get_available_levels(self):
        """Get list of available levels"""
        return {
            EnvironmentType.NEIGHBORHOOD: {
                "name": "Neighborhood",
                "description": "Suburban residential area with houses and streets",
                "best_for": ["box_mission", "spiral_search"],
                "spawn_point": {"x": 0, "y": 0, "z": -2}
            },
            EnvironmentType.LANDSCAPE_MOUNTAINS: {
                "name": "LandscapeMountains", 
                "description": "Mountainous terrain with valleys and peaks",
                "best_for": ["grid_survey", "search_rescue"],
                "spawn_point": {"x": 0, "y": 0, "z": -10}
            },
            EnvironmentType.CITY: {
                "name": "CityEnviron",
                "description": "Urban cityscape with tall buildings",
                "best_for": ["building_inspection", "urban_patrol"],
                "spawn_point": {"x": 0, "y": 0, "z": -50}
            },
            EnvironmentType.BLOCKS: {
                "name": "Blocks",
                "description": "Simple geometric test environment",
                "best_for": ["testing", "development"],
                "spawn_point": {"x": 0, "y": 0, "z": -2}
            }
        }
    
    def load_level(self, environment_type: EnvironmentType):
        """Load a specific level"""
        if not self.client:
            print(" Not connected to AirSim")
            return False
            
        level_info = self.available_levels.get(environment_type)
        if not level_info:
            print(f" Unknown environment: {environment_type}")
            return False
        
        level_name = level_info["name"]
        print(f" Loading environment: {level_name}")
        
        try:
            # Execute level change command
            result = self.client.simRunConsoleCommand(f"open {level_name}")
            print(f"Level change command result: {result}")
            
            # Wait for level to load
            print(" Waiting for level to load...")
            time.sleep(5)
            
            # Reset drone position
            spawn_point = level_info["spawn_point"]
            pose = airsim.Pose()
            pose.position = airsim.Vector3r(
                spawn_point["x"], 
                spawn_point["y"], 
                spawn_point["z"]
            )
            
            self.client.simSetVehiclePose(pose, True)
            self.current_level = environment_type
            
            print(f" Successfully loaded: {level_name}")
            return True
            
        except Exception as e:
            print(f" Failed to load level {level_name}: {e}")
            return False
    
    def get_recommended_environment(self, mission_type: str):
        """Get recommended environment for mission type"""
        mission_env_map = {
            "box_mission": EnvironmentType.NEIGHBORHOOD,
            "spiral_search": EnvironmentType.LANDSCAPE_MOUNTAINS,
            "grid_survey": EnvironmentType.LANDSCAPE_MOUNTAINS,
            "figure8_mission": EnvironmentType.BLOCKS,
            "star_mission": EnvironmentType.NEIGHBORHOOD,
            "wave_mission": EnvironmentType.LANDSCAPE_MOUNTAINS
        }
        
        return mission_env_map.get(mission_type, EnvironmentType.NEIGHBORHOOD)
    
    def load_for_mission(self, mission_type: str):
        """Load appropriate environment for mission type"""
        env_type = self.get_recommended_environment(mission_type)
        return self.load_level(env_type)
    
    def list_available_levels(self):
        """List all available environments"""
        print("\n Available Environments:")
        print("=" * 50)
        
        for env_type, info in self.available_levels.items():
            print(f" {info['name']}")
            print(f"   Description: {info['description']}")
            print(f"   Best for: {', '.join(info['best_for'])}")
            print()
    
    def get_current_level_info(self):
        """Get information about current level"""
        if self.current_level:
            return self.available_levels[self.current_level]
        return None

def main():
    """Demo/test the level manager"""
    print(" AirSim Level Manager Demo")
    print("=" * 40)
    
    manager = LevelManager()
    
    # Connect to AirSim
    if not manager.connect():
        return
    
    # List available levels
    manager.list_available_levels()
    
    # Demo: Load different environments
    test_environments = [
        EnvironmentType.BLOCKS,
        EnvironmentType.NEIGHBORHOOD,
        EnvironmentType.LANDSCAPE_MOUNTAINS
    ]
    
    for env in test_environments:
        print(f"\n Testing environment: {env.value}")
        if manager.load_level(env):
            time.sleep(3)  # Let user see the environment
        else:
            print(f"  Failed to load {env.value}")
    
    print("\n Level manager demo complete!")

if __name__ == "__main__":
    main() 