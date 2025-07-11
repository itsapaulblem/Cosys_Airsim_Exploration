#!/usr/bin/env python3

import os
import sys
import json
import subprocess
import threading
from pathlib import Path

class UnrealMissionInterface:
    """
    Interface between Unreal Engine HUD and AirSim missions.
    Handles mission launching, parameter validation, and status reporting.
    """
    
    def __init__(self):
        self.current_mission = None
        self.mission_status = {
            'active': False,
            'type': None,
            'progress': 0,
            'phase': 'idle',
            'message': 'Ready to launch mission'
        }
        
        # Get the current script directory
        self.script_dir = Path(__file__).parent
        
        # Available mission types and their parameters
        self.mission_types = {
            'box': {
                'script': 'box_mission.py',
                'name': 'Box Orbital Mission',
                'description': 'Fly rectangular pattern with orbital photography at vertices',
                'parameters': {
                    'box_size': {'type': 'float', 'default': 400, 'min': 100, 'max': 1000, 'unit': 'm'},
                    'altitude': {'type': 'float', 'default': 20, 'min': 5, 'max': 100, 'unit': 'm'},
                    'speed': {'type': 'float', 'default': 5, 'min': 1, 'max': 20, 'unit': 'm/s'},
                    'orbit_radius': {'type': 'float', 'default': 50, 'min': 10, 'max': 200, 'unit': 'm'},
                    'orbit_speed': {'type': 'float', 'default': 3, 'min': 1, 'max': 10, 'unit': 'm/s'},
                    'photos_per_orbit': {'type': 'int', 'default': 8, 'min': 1, 'max': 20, 'unit': 'photos'},
                    'orbit_mode': {'type': 'choice', 'default': 'waypoint', 'choices': ['waypoint', 'velocity']},
                    'max_collisions': {'type': 'int', 'default': 5, 'min': 1, 'max': 10, 'unit': 'count'}
                }
            },
            'spiral': {
                'script': 'spiral_mission.py',
                'name': 'Spiral Search Mission',
                'description': 'Systematic spiral search pattern for area coverage',
                'parameters': {
                    'max_radius': {'type': 'float', 'default': 500, 'min': 50, 'max': 1000, 'unit': 'm'},
                    'altitude': {'type': 'float', 'default': 30, 'min': 5, 'max': 100, 'unit': 'm'},
                    'speed': {'type': 'float', 'default': 8, 'min': 1, 'max': 20, 'unit': 'm/s'},
                    'spiral_spacing': {'type': 'float', 'default': 25, 'min': 5, 'max': 100, 'unit': 'm'},
                    'photo_interval': {'type': 'float', 'default': 2.0, 'min': 0.5, 'max': 10.0, 'unit': 's'},
                    'pattern': {'type': 'choice', 'default': 'outward', 'choices': ['outward', 'inward']},
                    'max_collisions': {'type': 'int', 'default': 5, 'min': 1, 'max': 10, 'unit': 'count'}
                }
            },
            'grid': {
                'script': 'grid_survey_mission.py',
                'name': 'Grid Survey Mission',
                'description': 'Professional survey patterns for mapping and photogrammetry',
                'parameters': {
                    'width': {'type': 'float', 'default': 800, 'min': 100, 'max': 2000, 'unit': 'm'},
                    'height': {'type': 'float', 'default': 600, 'min': 100, 'max': 2000, 'unit': 'm'},
                    'altitude': {'type': 'float', 'default': 20, 'min': 5, 'max': 100, 'unit': 'm'},
                    'speed': {'type': 'float', 'default': 5, 'min': 1, 'max': 20, 'unit': 'm/s'},
                    'grid_spacing': {'type': 'float', 'default': 50, 'min': 10, 'max': 200, 'unit': 'm'},
                    'photo_overlap': {'type': 'float', 'default': 30, 'min': 10, 'max': 80, 'unit': '%'},
                    'pattern': {'type': 'choice', 'default': 'boustrophedon', 'choices': ['boustrophedon', 'parallel']},
                    'grid_rotation': {'type': 'float', 'default': 0, 'min': -180, 'max': 180, 'unit': '°'},
                    'max_collisions': {'type': 'int', 'default': 5, 'min': 1, 'max': 10, 'unit': 'count'}
                }
            }
        }
    
    def get_mission_types(self):
        """Return available mission types for UI population."""
        return json.dumps(self.mission_types)
    
    def get_mission_parameters(self, mission_type):
        """Get parameters for a specific mission type."""
        if mission_type in self.mission_types:
            return json.dumps(self.mission_types[mission_type]['parameters'])
        return json.dumps({})
    
    def validate_parameters(self, mission_type, parameters):
        """Validate mission parameters against constraints."""
        if mission_type not in self.mission_types:
            return {'valid': False, 'error': f'Unknown mission type: {mission_type}'}
        
        mission_config = self.mission_types[mission_type]
        validated_params = {}
        errors = []
        
        for param_name, param_config in mission_config['parameters'].items():
            value = parameters.get(param_name, param_config['default'])
            
            # Type validation and conversion
            if param_config['type'] == 'float':
                try:
                    value = float(value)
                    if 'min' in param_config and value < param_config['min']:
                        errors.append(f"{param_name} must be >= {param_config['min']}")
                    if 'max' in param_config and value > param_config['max']:
                        errors.append(f"{param_name} must be <= {param_config['max']}")
                except ValueError:
                    errors.append(f"{param_name} must be a number")
            
            elif param_config['type'] == 'int':
                try:
                    value = int(value)
                    if 'min' in param_config and value < param_config['min']:
                        errors.append(f"{param_name} must be >= {param_config['min']}")
                    if 'max' in param_config and value > param_config['max']:
                        errors.append(f"{param_name} must be <= {param_config['max']}")
                except ValueError:
                    errors.append(f"{param_name} must be an integer")
            
            elif param_config['type'] == 'choice':
                if value not in param_config['choices']:
                    errors.append(f"{param_name} must be one of: {param_config['choices']}")
            
            validated_params[param_name] = value
        
        return {
            'valid': len(errors) == 0,
            'errors': errors,
            'parameters': validated_params
        }
    
    def preview_mission(self, mission_type, parameters):
        """Generate mission preview without executing."""
        validation = self.validate_parameters(mission_type, parameters)
        if not validation['valid']:
            return {
                'success': False,
                'error': 'Parameter validation failed',
                'details': validation['errors']
            }
        
        try:
            # Build command for preview
            script_path = self.script_dir / self.mission_types[mission_type]['script']
            cmd = [sys.executable, str(script_path), '--preview']
            
            # Add parameters
            for param_name, value in validation['parameters'].items():
                cmd.extend([f'--{param_name}', str(value)])
            
            # Execute preview
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            return {
                'success': result.returncode == 0,
                'output': result.stdout,
                'error': result.stderr if result.returncode != 0 else None
            }
            
        except subprocess.TimeoutExpired:
            return {'success': False, 'error': 'Preview timeout'}
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def launch_mission(self, mission_type, parameters):
        """Launch mission with specified parameters."""
        if mission_type not in self.mission_types:
            return {'success': False, 'error': f'Unknown mission type: {mission_type}'}
        
        if self.mission_status['active']:
            return {'success': False, 'error': 'Mission already active'}
        
        try:
            # Build command
            script_path = self.script_dir / self.mission_types[mission_type]['script']
            cmd = [sys.executable, str(script_path)]
            
            # Add parameters
            for param_name, value in parameters.items():
                cmd.extend([f'--{param_name}', str(value)])
            
            # Launch mission in separate thread
            self.mission_status = {
                'active': True,
                'type': mission_type,
                'progress': 0,
                'phase': 'launching',
                'message': f'Launching {self.mission_types[mission_type]["name"]}...'
            }
            
            def run_mission():
                try:
                    self.current_mission = subprocess.Popen(cmd)
                    self.current_mission.wait()
                    
                    if self.current_mission.returncode == 0:
                        self.mission_status.update({
                            'active': False,
                            'phase': 'completed',
                            'progress': 100,
                            'message': 'Mission completed successfully'
                        })
                    else:
                        self.mission_status.update({
                            'active': False,
                            'phase': 'failed',
                            'progress': 0,
                            'message': 'Mission failed'
                        })
                        
                except Exception as e:
                    self.mission_status.update({
                        'active': False,
                        'phase': 'error',
                        'progress': 0,
                        'message': f'Mission error: {str(e)}'
                    })
            
            mission_thread = threading.Thread(target=run_mission)
            mission_thread.daemon = True
            mission_thread.start()
            
            return {'success': True, 'message': f'{self.mission_types[mission_type]["name"]} launched'}
            
        except Exception as e:
            self.mission_status['active'] = False
            return {'success': False, 'error': str(e)}
    
    def stop_mission(self):
        """Emergency stop current mission."""
        if self.current_mission and self.current_mission.poll() is None:
            self.current_mission.terminate()
            self.mission_status.update({
                'active': False,
                'phase': 'stopped',
                'progress': 0,
                'message': 'Mission stopped by user'
            })
            return {'success': True, 'message': 'Mission stopped'}
        
        return {'success': False, 'error': 'No active mission to stop'}
    
    def get_mission_status(self):
        """Get current mission status."""
        return json.dumps(self.mission_status)


# Global interface instance
mission_interface = UnrealMissionInterface()

# Convenience functions for Unreal Engine Blueprint calls
def get_mission_types():
    return mission_interface.get_mission_types()

def get_mission_parameters(mission_type):
    """Get parameters for specific mission type."""
    return mission_interface.get_mission_parameters(mission_type)

def preview_mission(mission_type, parameters_json):
    """Preview mission with parameters."""
    parameters = json.loads(parameters_json)
    result = mission_interface.preview_mission(mission_type, parameters)
    return json.dumps(result)

def launch_mission(mission_type, parameters_json):
    parameters = json.loads(parameters_json)
    result = mission_interface.launch_mission(mission_type, parameters)
    return json.dumps(result)

def stop_mission():
    """Stop current mission."""
    result = mission_interface.stop_mission()
    return json.dumps(result)

def get_mission_status():
    return mission_interface.get_mission_status()


if __name__ == "__main__":
    # Test the interface
    print("Testing Unreal Mission Interface...")
    
    print("\nAvailable mission types:")
    print(get_mission_types())
    
    print("\nBox mission parameters:")
    print(get_mission_parameters('box'))
    
    print("\nPreview box mission:")
    test_params = json.dumps({
        'box_size': 300,
        'altitude': 25,
        'speed': 6
    })
    print(preview_mission('box', test_params)) 