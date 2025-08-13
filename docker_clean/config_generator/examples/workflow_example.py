#!/usr/bin/env python3
"""
Example script demonstrating the standardized AirSim configuration workflow
This script shows how to programmatically generate and validate configurations
"""

import subprocess
import sys
from pathlib import Path
import time

def run_command(cmd, description=""):
    """Run a command and handle errors"""
    print(f"\n{'='*60}")
    print(f"🔧 {description}")
    print(f"Command: {' '.join(cmd)}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print("✅ Success!")
        if result.stdout:
            print(f"Output:\n{result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed!")
        print(f"Error: {e.stderr}")
        return False
    except FileNotFoundError:
        print(f"❌ Command not found: {cmd[0]}")
        return False

def check_prerequisites():
    """Check if required tools are available"""
    print("🔍 Checking Prerequisites...")
    
    required_tools = [
        (["python", "--version"], "Python"),
        (["docker", "--version"], "Docker")
    ]
    
    all_ok = True
    for cmd, tool in required_tools:
        if not run_command(cmd, f"Checking {tool}"):
            all_ok = False
    
    return all_ok

def workflow_example_single():
    """Example: Single drone workflow"""
    print("\n🚁 WORKFLOW EXAMPLE: Single Drone Setup")
    print("="*80)
    
    # Step 1: Generate configuration
    if not run_command([
        "python", "unified_generator.py", "single", 
        "--output-dir", "./example_single"
    ], "Generating single drone configuration"):
        return False
    
    # Step 2: Validate configuration
    if not run_command([
        "python", "config_validator.py", 
        "--settings", "./example_single/settings.json",
        "--compose", "./example_single/docker-compose.yml",
        "--all"
    ], "Validating single drone configuration"):
        return False
    
    print("\n✅ Single drone workflow completed successfully!")
    print("Generated files in ./example_single/:")
    for file in Path("./example_single").glob("*"):
        print(f"  - {file.name}")
    
    return True

def workflow_example_multi():
    """Example: Multi-drone workflow"""
    print("\n🚁 WORKFLOW EXAMPLE: Multi-Drone Setup (3 drones, grid layout)")
    print("="*80)
    
    # Step 1: Generate configuration
    if not run_command([
        "python", "unified_generator.py", "multi",
        "--num-drones", "3",
        "--layout", "grid",
        "--output-dir", "./example_multi"
    ], "Generating multi-drone configuration"):
        return False
    
    # Step 2: Validate configuration  
    if not run_command([
        "python", "config_validator.py",
        "--settings", "./example_multi/settings.json", 
        "--compose", "./example_multi/docker-compose.yml",
        "--all"
    ], "Validating multi-drone configuration"):
        return False
    
    print("\n✅ Multi-drone workflow completed successfully!")
    print("Generated files in ./example_multi/:")
    for file in Path("./example_multi").glob("*"):
        print(f"  - {file.name}")
    
    return True

def workflow_example_custom_gps():
    """Example: Custom GPS location workflow"""
    print("\n🌍 WORKFLOW EXAMPLE: Custom GPS Location (New York City)")
    print("="*80)
    
    # NYC coordinates: 40.7128° N, 74.0060° W
    if not run_command([
        "python", "unified_generator.py", "multi",
        "--num-drones", "2",
        "--layout", "circle",
        "--gps-location", "40.7128", "-74.0060", "10",
        "--output-dir", "./example_nyc"
    ], "Generating NYC configuration"):
        return False
    
    # Validate
    if not run_command([
        "python", "config_validator.py",
        "--settings", "./example_nyc/settings.json",
        "--compose", "./example_nyc/docker-compose.yml", 
        "--compatibility"
    ], "Validating NYC configuration"):
        return False
    
    print("\n✅ Custom GPS workflow completed successfully!")
    return True

def workflow_example_mixed():
    """Example: Mixed vehicle types workflow"""
    print("\n🚗🚁 WORKFLOW EXAMPLE: Mixed Vehicle Types")
    print("="*80)
    
    if not run_command([
        "python", "unified_generator.py", "mixed",
        "--output-dir", "./example_mixed"
    ], "Generating mixed vehicle configuration"):
        return False
    
    if not run_command([
        "python", "config_validator.py",
        "--settings", "./example_mixed/settings.json",
        "--compose", "./example_mixed/docker-compose.yml",
        "--all"
    ], "Validating mixed vehicle configuration"):
        return False
    
    print("\n✅ Mixed vehicle workflow completed successfully!")
    return True

def workflow_example_validation_only():
    """Example: Validation-only workflow"""
    print("\n🔍 WORKFLOW EXAMPLE: Validation Only")
    print("="*80)
    
    # Check if we have any existing configurations to validate
    configs_to_check = [
        ("./example_single", "Single drone example"),
        ("./example_multi", "Multi-drone example"),
        ("settings.json", "Current directory settings"),
        ("docker-compose.yml", "Current directory compose")
    ]
    
    found_configs = False
    
    for config_path, description in configs_to_check:
        path = Path(config_path)
        if path.exists():
            found_configs = True
            if path.is_dir():
                settings_file = path / "settings.json"
                compose_file = path / "docker-compose.yml"
                if settings_file.exists() and compose_file.exists():
                    run_command([
                        "python", "config_validator.py",
                        "--settings", str(settings_file),
                        "--compose", str(compose_file),
                        "--all"
                    ], f"Validating {description}")
            else:
                # Single file - try to validate if it's a known type
                if config_path.endswith(".json"):
                    run_command([
                        "python", "config_validator.py",
                        "--settings", config_path
                    ], f"Validating {description}")
    
    if not found_configs:
        print("ℹ️  No existing configurations found to validate.")
        print("   Run other examples first to generate configurations.")
    
    return True

def cleanup_examples():
    """Clean up example directories"""
    print("\n🧹 Cleaning up example directories...")
    
    example_dirs = ["./example_single", "./example_multi", "./example_nyc", "./example_mixed"]
    
    for dir_path in example_dirs:
        path = Path(dir_path)
        if path.exists():
            print(f"  Removing {dir_path}")
            import shutil
            shutil.rmtree(path)
    
    print("✅ Cleanup completed!")

def main():
    """Main workflow demonstration"""
    print("🚁 STANDARDIZED AIRSIM WORKFLOW DEMONSTRATION")
    print("="*80)
    print("This script demonstrates the standardized workflow for generating")
    print("and validating AirSim configurations using the unified tools.")
    print("="*80)
    
    # Check prerequisites
    if not check_prerequisites():
        print("\n❌ Prerequisites not met. Please install required tools.")
        return 1
    
    # Interactive menu
    while True:
        print("\n📋 Available Workflow Examples:")
        print("1. Single drone setup")
        print("2. Multi-drone setup (3 drones)")
        print("3. Custom GPS location (NYC)")
        print("4. Mixed vehicle types")
        print("5. Validation only")
        print("6. Run all examples")
        print("7. Clean up examples")
        print("0. Exit")
        
        try:
            choice = input("\nSelect an example (0-7): ").strip()
        except KeyboardInterrupt:
            print("\n\nExiting...")
            return 0
        
        if choice == "0":
            print("Goodbye! 👋")
            break
        elif choice == "1":
            workflow_example_single()
        elif choice == "2":
            workflow_example_multi()
        elif choice == "3":
            workflow_example_custom_gps()
        elif choice == "4":
            workflow_example_mixed()
        elif choice == "5":
            workflow_example_validation_only()
        elif choice == "6":
            print("\n🚀 Running all workflow examples...")
            workflow_example_single()
            time.sleep(1)
            workflow_example_multi()
            time.sleep(1)
            workflow_example_custom_gps()
            time.sleep(1)
            workflow_example_mixed()
            time.sleep(1)
            workflow_example_validation_only()
            print("\n✅ All examples completed!")
        elif choice == "7":
            cleanup_examples()
        else:
            print("❌ Invalid choice. Please select 0-7.")
        
        if choice != "0":
            input("\nPress Enter to continue...")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())