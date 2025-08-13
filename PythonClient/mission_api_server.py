#!/usr/bin/env python3
"""
Mission API Server

This lightweight HTTP server runs AirSim missions independently of Streamlit,
avoiding the tornado dependency conflict between msgpack-rpc-python and streamlit.

Usage: python mission_api_server.py
Then access via: http://localhost:8000
"""

import json
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import os

# Global state to persist across all request handlers
MISSION_STATE = {
    'process': None,
    'status': {
        'active': False,
        'type': None,
        'start_time': None,
        'progress': 0,
        'phase': 'Idle',
        'message': 'Ready'
    },
    'logs': []
}

class MissionAPIHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def mission_process(self):
        return MISSION_STATE['process']
    
    @mission_process.setter
    def mission_process(self, value):
        MISSION_STATE['process'] = value
    
    @property
    def mission_status(self):
        return MISSION_STATE['status']
    
    @mission_status.setter 
    def mission_status(self, value):
        MISSION_STATE['status'] = value
    
    @property
    def mission_logs(self):
        return MISSION_STATE['logs']

    def do_GET(self):
        """Handle GET requests"""
        path = urlparse(self.path).path
        
        if path == '/api/missions':
            self.get_available_missions()
        elif path == '/api/status':
            self.get_mission_status()
        elif path == '/api/logs':
            self.get_mission_logs()
        elif path == '/api/test_airsim':
            self.test_airsim_connection()
        elif path == '/':
            self.serve_index()
        else:
            self.send_error(404)

    def do_POST(self):
        """Handle POST requests"""
        path = urlparse(self.path).path
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return
        
        if path == '/api/launch':
            self.launch_mission(data)
        elif path == '/api/stop':
            self.stop_mission()
        else:
            self.send_error(404)

    def get_available_missions(self):
        """Return available mission types"""
        missions = {
            "box": {
                "name": "Box Waypoint",
                "description": "The drone will navigate a four waypoint route creating a box. Corner points will be interpolated with rectangular turns. Final position will return to origin.",
                "script": "box_mission.py",
                "parameters": {
                    "box_size": {"type": "float", "default": 400, "min": 100, "max": 1000},
                    "altitude": {"type": "float", "default": 20, "min": 5, "max": 100},
                    "speed": {"type": "float", "default": 5, "min": 1, "max": 20},
                    "orbit_radius": {"type": "float", "default": 50, "min": 10, "max": 200},
                    "orbit_speed": {"type": "float", "default": 3, "min": 1, "max": 10},
                    "photos_per_orbit": {"type": "int", "default": 8, "min": 1, "max": 20},
                    "orbit_mode": {"type": "choice", "default": "velocity", "choices": ["waypoint", "velocity"]},
                    "enable_orbits": {"type": "bool", "default": True}
                }
            },
            "spiral": {
                "name": "Spiral Search Mission",
                "description": "Systematic spiral search pattern",
                "script": "spiral_mission.py",
                "parameters": {
                    "max_radius": {"type": "float", "default": 500, "min": 50, "max": 1000},
                    "altitude": {"type": "float", "default": 30, "min": 5, "max": 100},
                    "speed": {"type": "float", "default": 8, "min": 1, "max": 20},
                    "spiral_spacing": {"type": "float", "default": 25, "min": 5, "max": 100}
                }
            },
            "grid": {
                "name": "Grid Survey Mission",
                "description": "Professional mapping patterns",
                "script": "grid_mission.py",
                "parameters": {
                    "width": {"type": "float", "default": 800, "min": 100, "max": 2000},
                    "height": {"type": "float", "default": 600, "min": 100, "max": 2000},
                    "altitude": {"type": "float", "default": 20, "min": 5, "max": 100},
                    "grid_spacing": {"type": "float", "default": 50, "min": 10, "max": 200}
                }
            }
        }
        
        self.send_json_response(missions)

    def get_mission_status(self):
        """Return current mission status with enhanced progress tracking"""
        # Update status based on process
        if self.mission_process:
            if self.mission_process.poll() is None:
                # Process still running
                self.mission_status['active'] = True
                
                # Enhanced progress calculation
                if self.mission_status['start_time']:
                    elapsed = time.time() - self.mission_status['start_time']
                    
                    # Estimate progress based on mission type and elapsed time
                    mission_type = self.mission_status.get('type', 'unknown')
                    
                    if mission_type == 'box':
                        # Box mission phases: Takeoff(1min) -> Flight(3-5min) -> Landing(1min)
                        estimated_total_time = 6 * 60  # 6 minutes estimate
                        progress = min(95, (elapsed / estimated_total_time) * 100)
                    elif mission_type == 'spiral':
                        # Spiral mission: typically longer
                        estimated_total_time = 10 * 60  # 10 minutes estimate  
                        progress = min(95, (elapsed / estimated_total_time) * 100)
                    elif mission_type == 'grid':
                        # Grid mission: longest
                        estimated_total_time = 15 * 60  # 15 minutes estimate
                        progress = min(95, (elapsed / estimated_total_time) * 100)
                    else:
                        # Generic progress
                        progress = min(95, elapsed * 2)  # 2% per second, max 95%
                    
                    self.mission_status['progress'] = progress
                    
                    # Update phase based on elapsed time and progress
                    if elapsed < 30:
                        self.mission_status['phase'] = 'Initializing'
                    elif elapsed < 60:
                        self.mission_status['phase'] = 'Taking Off'
                    elif progress < 20:
                        self.mission_status['phase'] = 'Climbing to Altitude'
                    elif progress < 80:
                        self.mission_status['phase'] = 'Executing Mission'
                    elif progress < 95:
                        self.mission_status['phase'] = 'Returning to Home'
                    else:
                        self.mission_status['phase'] = 'Landing'
                    
                    # Update message based on mission type and phase
                    phase = self.mission_status['phase']
                    if phase == 'Executing Mission':
                        if mission_type == 'box':
                            self.mission_status['message'] = 'Flying box pattern with orbital photography'
                        elif mission_type == 'spiral':
                            self.mission_status['message'] = 'Executing spiral search pattern'
                        elif mission_type == 'grid':
                            self.mission_status['message'] = 'Following grid survey pattern'
                        else:
                            self.mission_status['message'] = f'Executing {mission_type} mission'
                    else:
                        self.mission_status['message'] = f'{phase} - {mission_type} mission'
                        
            else:
                # Process finished
                return_code = self.mission_process.returncode
                self.mission_status['active'] = False
                self.mission_status['progress'] = 100
                
                if return_code == 0:
                    self.mission_status['phase'] = 'Completed Successfully'
                    self.mission_status['message'] = 'Mission completed successfully'
                else:
                    self.mission_status['phase'] = 'Failed'
                    self.mission_status['message'] = f'Mission failed with code {return_code}'
                
                self.mission_process = None
        
        self.send_json_response(self.mission_status)

    def get_mission_logs(self):
        """Return mission logs"""
        # Add some default logs if empty
        if not self.mission_logs:
            self.mission_logs.extend([
                f"[{datetime.now().strftime('%H:%M:%S')}] Mission API Server started",
                f"[{datetime.now().strftime('%H:%M:%S')}] Ready to accept mission requests"
            ])
        self.send_json_response({'logs': self.mission_logs})

    def test_airsim_connection(self):
        """Test AirSim connection and return status"""
        try:
            # Add the parent directory to path for imports
            import sys
            from pathlib import Path
            parent_dir = Path(__file__).parent
            if str(parent_dir) not in sys.path:
                sys.path.append(str(parent_dir))
            
            # Try different import methods
            try:
                import cosysairsim as airsim  # Try cosysairsim first
            except ImportError:
                try:
                    import airsim  # Fallback to regular airsim
                except ImportError:
                    # Try adding multirotor directory to path
                    multirotor_path = parent_dir / "multirotor" 
                    if multirotor_path.exists():
                        sys.path.append(str(multirotor_path))
                        # Import setup_path from multirotor directory
                        import setup_path
                        import cosysairsim as airsim
                    else:
                        raise ImportError("AirSim library not found")
            
            # Test connection
            client = airsim.MultirotorClient()
            client.confirmConnection()
            
            # Get simulation state
            sim_state = client.getMultirotorState()
            pose = client.simGetVehiclePose()
            
            result = {
                'success': True,
                'status': 'Connected',
                'details': {
                    'landed_state': str(sim_state.landed_state),
                    'position': {
                        'x': round(pose.position.x_val, 2),
                        'y': round(pose.position.y_val, 2), 
                        'z': round(pose.position.z_val, 2)
                    },
                    'api_control': client.isApiControlEnabled()
                }
            }
            
            self.send_json_response(result)
            
        except ImportError as e:
            self.send_json_response({
                'success': False,
                'status': f'AirSim library not available: {str(e)}',
                'details': {
                    'troubleshooting': [
                        'Make sure AirSim Python package is installed',
                        'Check that setup_path.py exists in multirotor directory',
                        'Verify cosysairsim or airsim is in Python path'
                    ]
                }
            })
        except Exception as e:
            self.send_json_response({
                'success': False,
                'status': f'Connection failed: {str(e)}',
                'details': {
                    'troubleshooting': [
                        'Make sure UE5 with AirSim is running',
                        'Click Play in UE5 to start simulation',
                        'Check AirSim settings.json exists',
                        'Verify no firewall is blocking connection'
                    ]
                }
            })

    def launch_mission(self, data):
        """Launch a mission with parameters"""
        mission_type = data.get('mission_type')
        parameters = data.get('parameters', {})
        
        print(f"Launch request: {mission_type} with params: {parameters}")
        
        if not mission_type:
            self.send_error(400, "Missing mission_type")
            return
        
        # Check if mission is already active
        if self.mission_process and self.mission_process.poll() is None:
            self.send_error(409, "Mission already active")
            return
        
        # Build command
        mission_planning_dir = Path(__file__).parent / "multirotor" / "mission_planning"
        script_name = f"{mission_type}_mission.py"
        script_path = mission_planning_dir / script_name
        
        print(f"Looking for script: {script_path}")
        
        if not script_path.exists():
            print(f"Script not found: {script_path}")
            self.send_error(404, f"Mission script not found: {script_name}")
            return
        
        # Build command with parameters
        cmd = [sys.executable, str(script_path)]
        for param_name, value in parameters.items():
            if isinstance(value, bool):
                if param_name == 'enable_orbits':
                    # Special handling for enable_orbits
                    if value:
                        cmd.append('--enable_orbits')
                    else:
                        cmd.append('--disable_orbits')
                else:
                    # Standard boolean handling for other parameters
                    if value:
                        cmd.append(f'--{param_name}')
            else:
                cmd.extend([f'--{param_name}', str(value)])
        
        print(f"Command: {' '.join(cmd)}")
        
        try:
            # Launch mission process
            self.mission_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=str(mission_planning_dir.parent.parent)  # Set working directory to PythonClient
            )
            
            print(f"Process started with PID: {self.mission_process.pid}")
            
            # Add launch log
            launch_msg = f"Mission {mission_type} launched with PID {self.mission_process.pid}"
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {launch_msg}")
            
            # Start a thread to monitor the process output
            threading.Thread(target=self._monitor_mission_output, daemon=True).start()
            
            # Update status
            self.mission_status = {
                'active': True,
                'type': mission_type,
                'start_time': time.time(),
                'progress': 0,
                'phase': 'Launching',
                'message': f'Mission {mission_type} launched'
            }
            
            self.send_json_response({
                'success': True,
                'message': f'Mission {mission_type} launched successfully',
                'pid': self.mission_process.pid
            })
            
        except Exception as e:
            error_msg = f"Launch failed: {str(e)}"
            print(f"{error_msg}")
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {error_msg}")
            self.send_error(500, f"Failed to launch mission: {str(e)}")
    
    def _monitor_mission_output(self):
        """Monitor mission process output in a separate thread"""
        if not self.mission_process:
            return
            
        try:
            # Monitor process in real-time
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] Monitoring mission process...")
            
            # Read output line by line for real-time updates
            while self.mission_process.poll() is None:
                try:
                    # Check for output with timeout
                    line = self.mission_process.stdout.readline()
                    if line:
                        line = line.strip()
                        print(f"Mission Output: {line}")
                        # Add important lines to logs (filter out too verbose output)
                        if any(keyword in line.lower() for keyword in ['error', 'warning', 'completed', 'failed', 'started', 'takeoff', 'landing', 'photo']):
                            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {line}")
                    
                    time.sleep(0.1)  # Small delay to prevent busy waiting
                except:
                    break
            
            # Process finished, get final output
            remaining_stdout, stderr = self.mission_process.communicate()
            
            if remaining_stdout:
                print("Final Mission Output:", remaining_stdout)
            if stderr:
                print("Mission Errors:", stderr)
                self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ERROR: {stderr}")
            
            # Log completion
            return_code = self.mission_process.returncode
            if return_code == 0:
                completion_msg = "Mission completed successfully"
            else:
                completion_msg = f"Mission failed with return code {return_code}"
            
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {completion_msg}")
            print(f"{completion_msg}")
                
        except subprocess.TimeoutExpired:
            timeout_msg = "Mission process timeout - killing process"
            print(f"{timeout_msg}")
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {timeout_msg}")
            self.mission_process.kill()
        except Exception as e:
            error_msg = f"Error monitoring mission: {e}"
            print(f"{error_msg}")
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] {error_msg}")

    def stop_mission(self):
        """Stop current mission"""
        if self.mission_process and self.mission_process.poll() is None:
            print("Stopping mission...")
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] Mission stop requested")
            
            self.mission_process.terminate()
            self.mission_process = None
            
            self.mission_status = {
                'active': False,
                'type': None,
                'start_time': None,
                'progress': 0,
                'phase': 'Stopped',
                'message': 'Mission stopped by user'
            }
            
            self.mission_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] Mission stopped by user")
            self.send_json_response({'success': True, 'message': 'Mission stopped'})
        else:
            self.send_json_response({'success': False, 'message': 'No active mission'})

    def serve_index(self):
        """Serve a simple status page"""
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Mission API Server</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 40px; background: #1a1a1a; color: #4AFF4A; }
                .container { max-width: 800px; margin: 0 auto; }
                .status { background: #2a2a2a; padding: 20px; border-radius: 10px; border: 2px solid #4AFF4A; }
                .api-endpoint { background: #333; padding: 10px; margin: 10px 0; border-radius: 5px; }
                h1 { text-align: center; }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>Mission API Server</h1>
                <div class="status">
                    <h2>Server Status: ONLINE</h2>
                    <p>The Mission API Server is running and ready to accept requests.</p>
                    
                    <h3>Available Endpoints:</h3>
                    <div class="api-endpoint">GET /api/missions - List available missions</div>
                    <div class="api-endpoint">GET /api/status - Get mission status</div>
                    <div class="api-endpoint">GET /api/logs - Get mission logs</div>
                    <div class="api-endpoint">GET /api/test_airsim - Test AirSim connection</div>
                    <div class="api-endpoint">POST /api/launch - Launch a mission</div>
                    <div class="api-endpoint">POST /api/stop - Stop current mission</div>
                    
                    <h3>Usage:</h3>
                    <p>Use this API server with the Streamlit Mission Control interface or make direct HTTP requests.</p>
                    <p>Server is running on: <strong>http://localhost:8000</strong></p>
                </div>
            </div>
        </body>
        </html>
        """
        
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(html.encode())

    def send_json_response(self, data):
        """Send JSON response"""
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data, indent=2).encode())

    def log_message(self, format, *args):
        """Override to customize logging"""
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {format % args}")

def main():
    port = 8000
    server_address = ('', port)
    
    print("Mission API Server")
    print("=" * 40)
    print(f"Starting server on port {port}...")
    print(f"Access at: http://localhost:{port}")
    print("Press Ctrl+C to stop")
    print("=" * 40)
    
    try:
        httpd = HTTPServer(server_address, MissionAPIHandler)
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped by user")

if __name__ == "__main__":
    main() 