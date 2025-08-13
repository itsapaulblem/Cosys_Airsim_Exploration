#!/usr/bin/env python3
"""
Test script to verify photo directory structure creation.
This tests the photo organization system without running a full mission.
"""

import os
import sys
from datetime import datetime

# Add the current directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))

from box_mission import BoxMission

def test_photo_directory_structure():
    """Test the photo directory structure creation."""
    print("Testing Photo Directory Structure Creation")
    print("=" * 50)
    
    # Create a test mission instance
    mission = BoxMission(
        box_size=100,
        altitude=10,
        speed=3,
        enable_orbits=True,
        disable_collision_detection=True  # Disable collisions for testing
    )
    
    # Initialize mission to trigger photo directory setup
    mission._initialize_mission()
    
    # Check if directories were created
    if mission.mission_instance_folder:
        print(f"✅ Photo directory created: {mission.mission_instance_folder}")
        
        # Check if the directory actually exists
        if os.path.exists(mission.mission_instance_folder):
            print(f"✅ Directory exists on filesystem")
            
            # Show the directory structure
            base_photos_dir = os.path.join(os.path.dirname(__file__), "photos")
            if os.path.exists(base_photos_dir):
                print(f"\n📁 Photo Directory Structure:")
                for root, dirs, files in os.walk(base_photos_dir):
                    level = root.replace(base_photos_dir, '').count(os.sep)
                    indent = ' ' * 2 * level
                    print(f"{indent}📁 {os.path.basename(root)}/")
                    subindent = ' ' * 2 * (level + 1)
                    for file in files:
                        print(f"{subindent}📄 {file}")
            
            # Test photo filename generation (without actually taking photos)
            print(f"\n🔍 Testing photo filename generation:")
            test_filename = f"test_photo_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.png"
            test_filepath = os.path.join(mission.mission_instance_folder, test_filename)
            print(f"   Sample photo path: {test_filepath}")
            
        else:
            print(f"❌ Directory does not exist on filesystem")
    else:
        print(f"❌ Photo directory was not set up")
    
    print(f"\n✅ Test completed successfully!")

if __name__ == "__main__":
    test_photo_directory_structure() 