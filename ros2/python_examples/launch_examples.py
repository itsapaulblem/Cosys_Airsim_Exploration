#!/usr/bin/env python3
"""
Launch script for AirSim ROS2 Python Examples

This script provides a convenient way to launch AirSim ROS2 node along with
Python subscriber examples.

Usage:
    python3 launch_examples.py [--example simple|full] [--vehicle VEHICLE_NAME]

Author: Generated for Cosys-AirSim ROS2 Integration
"""

import os
import sys
import argparse
import subprocess
import signal
import time
from pathlib import Path

class AirSimLauncher:
    def __init__(self):
        self.processes = []
        self.current_dir = Path(__file__).parent.absolute()
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\nShutting down...")
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                time.sleep(1)
                if process.poll() is None:
                    process.kill()
        sys.exit(0)
    
    def run_command(self, cmd, name, wait=False):
        """Run a command in subprocess"""
        print(f"Starting {name}: {' '.join(cmd)}")
        
        try:
            if wait:
                result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                print(f"{name} output: {result.stdout}")
                if result.stderr:
                    print(f"{name} error: {result.stderr}")
                return result
            else:
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                self.processes.append(process)
                return process
        except subprocess.CalledProcessError as e:
            print(f"Error running {name}: {e}")
            return None
        except FileNotFoundError:
            print(f"Command not found for {name}: {cmd[0]}")
            return None
    
    def check_ros2_setup(self):
        """Check if ROS2 environment is properly set up"""
        print("Checking ROS2 environment...")
        
        # Check if ROS2 is available
        result = self.run_command(['which', 'ros2'], 'ROS2 Check', wait=True)
        if not result or result.returncode != 0:
            print("ERROR: ROS2 not found. Please install ROS2 and source the setup script.")
            return False
        
        # Check if airsim_interfaces is available
        result = self.run_command(['ros2', 'pkg', 'list'], 'Package Check', wait=True)
        if not result or 'airsim_interfaces' not in result.stdout:
            print("ERROR: airsim_interfaces not found. Please build the AirSim ROS2 workspace.")
            print("Run: cd /path/to/cosys-airsim/ros2 && colcon build && source install/setup.bash")
            return False
        
        print("ROS2 environment OK")
        return True
    
    def launch_airsim_node(self, vehicle_name=""):
        """Launch AirSim ROS2 node"""
        cmd = [
            'ros2', 'launch', 'airsim_ros_pkgs', 'airsim_node.launch.py',
            'enable_api_control:=true',
            'host_ip:=localhost',
            'host_port:=41451'
        ]
        
        if vehicle_name:
            # Note: Vehicle name handling would need to be implemented in launch file
            pass
        
        process = self.run_command(cmd, 'AirSim ROS2 Node')
        if process:
            print("Waiting for AirSim node to start...")
            time.sleep(5)  # Give the node time to start
            return True
        return False
    
    def launch_example(self, example_type, vehicle_name=""):
        """Launch Python example"""
        if example_type == 'simple':
            script_path = self.current_dir / 'simple_subscriber_example.py'
            cmd = ['python3', str(script_path)]
        elif example_type == 'full':
            script_path = self.current_dir / 'airsim_ros2_subscriber.py'
            cmd = ['python3', str(script_path)]
            
            if vehicle_name:
                cmd.extend(['--vehicle', vehicle_name])
            
            # Add demo mode for full example
            cmd.append('--demo')
        else:
            print(f"Unknown example type: {example_type}")
            return False
        
        # Check if script exists
        if not script_path.exists():
            print(f"Script not found: {script_path}")
            return False
        
        process = self.run_command(cmd, f'{example_type.capitalize()} Example')
        return process is not None
    
    def wait_for_processes(self):
        """Wait for all processes and handle output"""
        try:
            while True:
                # Check if any process has terminated
                for i, process in enumerate(self.processes):
                    if process.poll() is not None:
                        print(f"Process {i} terminated with code {process.returncode}")
                        # Read any remaining output
                        stdout, stderr = process.communicate()
                        if stdout:
                            print(f"Process {i} stdout: {stdout}")
                        if stderr:
                            print(f"Process {i} stderr: {stderr}")
                
                # Remove terminated processes
                self.processes = [p for p in self.processes if p.poll() is None]
                
                if not self.processes:
                    print("All processes have terminated")
                    break
                
                time.sleep(1)
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)
    
    def run(self, example_type='simple', vehicle_name=''):
        """Main run function"""
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("=== AirSim ROS2 Python Examples Launcher ===")
        print(f"Example type: {example_type}")
        print(f"Vehicle name: {vehicle_name or 'all vehicles'}")
        print()
        
        # Check ROS2 setup
        if not self.check_ros2_setup():
            return 1
        
        # Launch AirSim node
        print("Launching AirSim ROS2 node...")
        if not self.launch_airsim_node(vehicle_name):
            print("Failed to launch AirSim node")
            return 1
        
        # Launch Python example
        print(f"Launching {example_type} example...")
        if not self.launch_example(example_type, vehicle_name):
            print(f"Failed to launch {example_type} example")
            return 1
        
        print("All components launched successfully!")
        print("Press Ctrl+C to stop all processes")
        print()
        
        # Wait for processes
        self.wait_for_processes()
        
        return 0

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Launch AirSim ROS2 Python Examples')
    parser.add_argument('--example', choices=['simple', 'full'], default='simple',
                       help='Example type to run (default: simple)')
    parser.add_argument('--vehicle', type=str, default='',
                       help='Vehicle name (default: all vehicles)')
    parser.add_argument('--airsim-only', action='store_true',
                       help='Launch only AirSim node (no Python example)')
    
    args = parser.parse_args()
    
    launcher = AirSimLauncher()
    
    if args.airsim_only:
        # Set up signal handler
        signal.signal(signal.SIGINT, launcher.signal_handler)
        
        print("=== AirSim ROS2 Node Launcher ===")
        if not launcher.check_ros2_setup():
            return 1
        
        print("Launching AirSim ROS2 node...")
        if not launcher.launch_airsim_node(args.vehicle):
            print("Failed to launch AirSim node")
            return 1
        
        print("AirSim node launched successfully!")
        print("Press Ctrl+C to stop")
        launcher.wait_for_processes()
        return 0
    
    return launcher.run(args.example, args.vehicle)

if __name__ == '__main__':
    sys.exit(main())