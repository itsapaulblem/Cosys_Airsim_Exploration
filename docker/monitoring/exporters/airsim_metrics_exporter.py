#!/usr/bin/env python3
"""
AirSim API Metrics Exporter for Prometheus
Monitors AirSim RPC API performance, vehicle telemetry, and simulation metrics
Exposes metrics on port 9201
"""

import cosysairsim as airsim
from prometheus_client import start_http_server, Gauge, Counter, Histogram
import time
import os
from typing import Dict, List

# Prometheus metrics
airsim_api_latency = Histogram('airsim_api_latency_seconds', 'AirSim API call latency', ['api_call', 'vehicle_name'])
airsim_api_calls_total = Counter('airsim_api_calls_total', 'Total number of AirSim API calls', ['api_call', 'vehicle_name'])
airsim_api_errors_total = Counter('airsim_api_errors_total', 'Total API call errors', ['api_call', 'vehicle_name'])

# Vehicle telemetry
airsim_vehicle_position_x = Gauge('airsim_vehicle_position_x_meters', 'Vehicle X position (NED)', ['vehicle_name'])
airsim_vehicle_position_y = Gauge('airsim_vehicle_position_y_meters', 'Vehicle Y position (NED)', ['vehicle_name'])
airsim_vehicle_position_z = Gauge('airsim_vehicle_position_z_meters', 'Vehicle Z position (NED)', ['vehicle_name'])

airsim_vehicle_velocity_x = Gauge('airsim_vehicle_velocity_x_mps', 'Vehicle X velocity (NED) m/s', ['vehicle_name'])
airsim_vehicle_velocity_y = Gauge('airsim_vehicle_velocity_y_mps', 'Vehicle Y velocity (NED) m/s', ['vehicle_name'])
airsim_vehicle_velocity_z = Gauge('airsim_vehicle_velocity_z_mps', 'Vehicle Z velocity (NED) m/s', ['vehicle_name'])

airsim_vehicle_altitude = Gauge('airsim_vehicle_altitude_meters', 'Vehicle altitude above ground (meters)', ['vehicle_name'])
airsim_vehicle_armed = Gauge('airsim_vehicle_armed', 'Vehicle armed status (1=armed, 0=disarmed)', ['vehicle_name'])
airsim_vehicle_landed = Gauge('airsim_vehicle_landed', 'Vehicle landed state (1=landed, 0=airborne)', ['vehicle_name'])

# Collision metrics
airsim_collisions_total = Counter('airsim_collisions_total', 'Total collisions detected', ['vehicle_name', 'object_name'])
airsim_collision_impact = Gauge('airsim_collision_impact_force', 'Last collision impact force', ['vehicle_name'])

# Simulation performance
airsim_sim_fps = Gauge('airsim_simulation_fps', 'Simulation frames per second')
airsim_sim_clock_speed = Gauge('airsim_simulation_clock_speed', 'Simulation clock speed multiplier')
airsim_connection_status = Gauge('airsim_connection_status', 'AirSim connection status (1=connected, 0=disconnected)')

# Image retrieval metrics
airsim_image_retrieval_latency = Histogram('airsim_image_retrieval_seconds', 'Image retrieval latency', ['vehicle_name', 'camera_name', 'image_type'])
airsim_images_retrieved_total = Counter('airsim_images_retrieved_total', 'Total images retrieved', ['vehicle_name', 'camera_name', 'image_type'])


class AirSimMetricsExporter:
    """
    AirSim metrics collector and Prometheus exporter
    """

    def __init__(self, airsim_host: str = 'airsim-container', airsim_port: int = 41451):
        self.airsim_host = airsim_host
        self.airsim_port = airsim_port
        self.client = None
        self.vehicle_names: List[str] = []
        self.last_collision_counts: Dict[str, int] = {}

        print(f"Initializing AirSim Metrics Exporter (connecting to {airsim_host}:{airsim_port})...")

    def connect(self) -> bool:
        """Connect to AirSim API"""
        try:
            self.client = airsim.MultirotorClient(ip=self.airsim_host, port=self.airsim_port)
            self.client.confirmConnection()
            airsim_connection_status.set(1)
            print(f"Successfully connected to AirSim at {self.airsim_host}:{self.airsim_port}")

            # Get list of vehicles
            self.vehicle_names = self.client.listVehicles()
            print(f"Discovered vehicles: {self.vehicle_names}")
            return True

        except Exception as e:
            print(f"Failed to connect to AirSim: {e}")
            airsim_connection_status.set(0)
            return False

    def collect_metrics(self):
        """Collect all AirSim metrics"""
        if not self.client:
            if not self.connect():
                return

        try:
            # Update connection status
            airsim_connection_status.set(1)

            # Collect metrics for each vehicle
            for vehicle_name in self.vehicle_names:
                self._collect_vehicle_metrics(vehicle_name)

            # Collect simulation-wide metrics
            self._collect_simulation_metrics()

        except Exception as e:
            print(f"Error collecting metrics: {e}")
            airsim_connection_status.set(0)
            # Try to reconnect on next iteration
            self.client = None

    def _collect_vehicle_metrics(self, vehicle_name: str):
        """Collect metrics for a specific vehicle"""
        try:
            # Get multirotor state
            start_time = time.time()
            state = self.client.getMultirotorState(vehicle_name=vehicle_name)
            latency = time.time() - start_time

            airsim_api_latency.labels(api_call='getMultirotorState', vehicle_name=vehicle_name).observe(latency)
            airsim_api_calls_total.labels(api_call='getMultirotorState', vehicle_name=vehicle_name).inc()

            # Position (NED coordinates)
            pos = state.kinematics_estimated.position
            airsim_vehicle_position_x.labels(vehicle_name=vehicle_name).set(pos.x_val)
            airsim_vehicle_position_y.labels(vehicle_name=vehicle_name).set(pos.y_val)
            airsim_vehicle_position_z.labels(vehicle_name=vehicle_name).set(pos.z_val)

            # Velocity (NED coordinates)
            vel = state.kinematics_estimated.linear_velocity
            airsim_vehicle_velocity_x.labels(vehicle_name=vehicle_name).set(vel.x_val)
            airsim_vehicle_velocity_y.labels(vehicle_name=vehicle_name).set(vel.y_val)
            airsim_vehicle_velocity_z.labels(vehicle_name=vehicle_name).set(vel.z_val)

            # Flight state
            airsim_vehicle_armed.labels(vehicle_name=vehicle_name).set(1 if state.armed else 0)
            airsim_vehicle_landed.labels(vehicle_name=vehicle_name).set(1 if state.landed_state == airsim.LandedState.Landed else 0)

            # Collision info
            collision_info = self.client.simGetCollisionInfo(vehicle_name=vehicle_name)
            if collision_info.has_collided:
                object_name = collision_info.object_name if collision_info.object_name else "Unknown"
                airsim_collisions_total.labels(vehicle_name=vehicle_name, object_name=object_name).inc()
                airsim_collision_impact.labels(vehicle_name=vehicle_name).set(collision_info.impact_point.z_val)

        except Exception as e:
            print(f"Error collecting metrics for vehicle {vehicle_name}: {e}")
            airsim_api_errors_total.labels(api_call='getMultirotorState', vehicle_name=vehicle_name).inc()

    def _collect_simulation_metrics(self):
        """Collect simulation-wide performance metrics"""
        try:
            # In future implementations, these could query actual simulation stats
            # For now, we'll set placeholder values
            # Real implementation would need AirSim API extensions for FPS/clock speed

            # Placeholder for simulation FPS (would need custom API call)
            airsim_sim_fps.set(60.0)  # Placeholder
            airsim_sim_clock_speed.set(1.0)  # Placeholder

        except Exception as e:
            print(f"Error collecting simulation metrics: {e}")

    def run(self, poll_interval: float = 1.0):
        """Main monitoring loop"""
        print(f"Starting metrics collection (poll interval: {poll_interval}s)...")

        while True:
            try:
                self.collect_metrics()
                time.sleep(poll_interval)
            except KeyboardInterrupt:
                print("\nShutting down AirSim Metrics Exporter...")
                break
            except Exception as e:
                print(f"Unexpected error in monitoring loop: {e}")
                time.sleep(poll_interval)


def main():
    """Main entry point"""
    print("Starting AirSim Metrics Exporter for Prometheus...")

    # Get configuration from environment variables
    airsim_host = os.getenv('AIRSIM_HOST_IP', 'airsim-container')
    airsim_port = int(os.getenv('AIRSIM_HOST_PORT', '41451'))
    metrics_port = int(os.getenv('METRICS_PORT', '9201'))
    poll_interval = float(os.getenv('POLL_INTERVAL', '1.0'))

    # Start Prometheus HTTP server
    start_http_server(metrics_port)
    print(f"Prometheus metrics server started on port {metrics_port}")

    # Create and run exporter
    exporter = AirSimMetricsExporter(airsim_host=airsim_host, airsim_port=airsim_port)
    exporter.run(poll_interval=poll_interval)


if __name__ == '__main__':
    main()
