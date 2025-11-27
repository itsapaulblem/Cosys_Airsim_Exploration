#!/usr/bin/env python3
"""
PX4 MAVLink Metrics Exporter for Prometheus
Connects to multiple PX4 SITL instances via MAVLink and exposes telemetry metrics
Exposes metrics on port 9202
"""

from pymavlink import mavutil
from prometheus_client import start_http_server, Gauge, Counter
import time
import os
import threading
from typing import Dict, List

# Prometheus metrics for PX4 telemetry

# Position and navigation
px4_position_lat = Gauge('px4_position_latitude_degrees', 'GPS latitude', ['drone_id'])
px4_position_lon = Gauge('px4_position_longitude_degrees', 'GPS longitude', ['drone_id'])
px4_position_alt = Gauge('px4_position_altitude_meters', 'GPS altitude MSL', ['drone_id'])
px4_position_relative_alt = Gauge('px4_position_relative_altitude_meters', 'Relative altitude above home', ['drone_id'])

# Velocity
px4_velocity_vx = Gauge('px4_velocity_x_ms', 'Velocity X (m/s)', ['drone_id'])
px4_velocity_vy = Gauge('px4_velocity_y_ms', 'Velocity Y (m/s)', ['drone_id'])
px4_velocity_vz = Gauge('px4_velocity_z_ms', 'Velocity Z (m/s)', ['drone_id'])
px4_velocity_ground_speed = Gauge('px4_ground_speed_ms', 'Ground speed (m/s)', ['drone_id'])

# Attitude
px4_attitude_roll = Gauge('px4_attitude_roll_degrees', 'Roll angle', ['drone_id'])
px4_attitude_pitch = Gauge('px4_attitude_pitch_degrees', 'Pitch angle', ['drone_id'])
px4_attitude_yaw = Gauge('px4_attitude_yaw_degrees', 'Yaw angle', ['drone_id'])

# Battery
px4_battery_voltage = Gauge('px4_battery_voltage_volts', 'Battery voltage', ['drone_id'])
px4_battery_current = Gauge('px4_battery_current_amps', 'Battery current', ['drone_id'])
px4_battery_remaining = Gauge('px4_battery_remaining_percent', 'Battery remaining percentage', ['drone_id'])

# Flight mode and status
px4_armed_status = Gauge('px4_armed_status', 'Armed status (1=armed, 0=disarmed)', ['drone_id'])
px4_flight_mode = Gauge('px4_flight_mode', 'Current flight mode (enum)', ['drone_id', 'mode_name'])
px4_system_status = Gauge('px4_system_status', 'System status (enum)', ['drone_id', 'status_name'])

# Connection health
px4_mavlink_connection = Gauge('px4_mavlink_connection_status', 'MAVLink connection status (1=connected, 0=disconnected)', ['drone_id'])
px4_mavlink_messages_received = Counter('px4_mavlink_messages_received_total', 'Total MAVLink messages received', ['drone_id', 'message_type'])
px4_heartbeat_age_seconds = Gauge('px4_heartbeat_age_seconds', 'Time since last heartbeat', ['drone_id'])

# GPS quality
px4_gps_fix_type = Gauge('px4_gps_fix_type', 'GPS fix type (0=none, 3=3D fix)', ['drone_id'])
px4_gps_satellites = Gauge('px4_gps_satellites_visible', 'Number of GPS satellites visible', ['drone_id'])

# Safety
px4_fence_breached = Gauge('px4_geofence_breached', 'Geofence breach status (1=breached)', ['drone_id'])
px4_failsafe_active = Gauge('px4_failsafe_active', 'Failsafe active status', ['drone_id'])


class PX4MAVLinkExporter:
    """
    Connects to multiple PX4 drones via MAVLink and exports telemetry to Prometheus
    """

    def __init__(self, drone_connections: List[Dict[str, str]]):
        self.drone_connections = drone_connections
        self.mavlink_connections: Dict[str, mavutil.mavlink_connection] = {}
        self.last_heartbeat: Dict[str, float] = {}
        self.running = True

        print(f"Initializing PX4 MAVLink Exporter for {len(drone_connections)} drones...")

    def connect_drones(self):
        """Establish MAVLink connections to all drones"""
        for drone in self.drone_connections:
            drone_id = drone['id']
            connection_string = drone['connection']

            try:
                print(f"Connecting to {drone_id} at {connection_string}...")
                mav = mavutil.mavlink_connection(connection_string, source_system=255)

                # Wait for heartbeat with timeout
                print(f"Waiting for heartbeat from {drone_id}...")
                heartbeat = mav.wait_heartbeat(timeout=10)

                if heartbeat:
                    self.mavlink_connections[drone_id] = mav
                    self.last_heartbeat[drone_id] = time.time()
                    px4_mavlink_connection.labels(drone_id=drone_id).set(1)
                    print(f"Successfully connected to {drone_id}")
                else:
                    print(f"No heartbeat received from {drone_id} within timeout")
                    px4_mavlink_connection.labels(drone_id=drone_id).set(0)

            except Exception as e:
                print(f"Failed to connect to {drone_id}: {e}")
                px4_mavlink_connection.labels(drone_id=drone_id).set(0)

    def collect_drone_metrics(self, drone_id: str):
        """Collect metrics from a single drone"""
        if drone_id not in self.mavlink_connections:
            return

        mav = self.mavlink_connections[drone_id]

        try:
            # Process all pending messages (non-blocking)
            while True:
                msg = mav.recv_match(blocking=False, timeout=0.1)
                if msg is None:
                    break

                msg_type = msg.get_type()

                # Count message
                px4_mavlink_messages_received.labels(drone_id=drone_id, message_type=msg_type).inc()

                # Process specific message types
                if msg_type == 'HEARTBEAT':
                    self.last_heartbeat[drone_id] = time.time()
                    px4_armed_status.labels(drone_id=drone_id).set(1 if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 0)

                elif msg_type == 'GLOBAL_POSITION_INT':
                    px4_position_lat.labels(drone_id=drone_id).set(msg.lat / 1e7)
                    px4_position_lon.labels(drone_id=drone_id).set(msg.lon / 1e7)
                    px4_position_alt.labels(drone_id=drone_id).set(msg.alt / 1000.0)
                    px4_position_relative_alt.labels(drone_id=drone_id).set(msg.relative_alt / 1000.0)

                    px4_velocity_vx.labels(drone_id=drone_id).set(msg.vx / 100.0)
                    px4_velocity_vy.labels(drone_id=drone_id).set(msg.vy / 100.0)
                    px4_velocity_vz.labels(drone_id=drone_id).set(msg.vz / 100.0)

                elif msg_type == 'VFR_HUD':
                    px4_velocity_ground_speed.labels(drone_id=drone_id).set(msg.groundspeed)

                elif msg_type == 'ATTITUDE':
                    import math
                    px4_attitude_roll.labels(drone_id=drone_id).set(math.degrees(msg.roll))
                    px4_attitude_pitch.labels(drone_id=drone_id).set(math.degrees(msg.pitch))
                    px4_attitude_yaw.labels(drone_id=drone_id).set(math.degrees(msg.yaw))

                elif msg_type == 'BATTERY_STATUS' or msg_type == 'SYS_STATUS':
                    if hasattr(msg, 'voltage_battery'):
                        px4_battery_voltage.labels(drone_id=drone_id).set(msg.voltage_battery / 1000.0)
                    if hasattr(msg, 'current_battery'):
                        px4_battery_current.labels(drone_id=drone_id).set(msg.current_battery / 100.0)
                    if hasattr(msg, 'battery_remaining'):
                        px4_battery_remaining.labels(drone_id=drone_id).set(msg.battery_remaining)

                elif msg_type == 'GPS_RAW_INT':
                    px4_gps_fix_type.labels(drone_id=drone_id).set(msg.fix_type)
                    px4_gps_satellites.labels(drone_id=drone_id).set(msg.satellites_visible)

            # Update heartbeat age
            if drone_id in self.last_heartbeat:
                age = time.time() - self.last_heartbeat[drone_id]
                px4_heartbeat_age_seconds.labels(drone_id=drone_id).set(age)

                # Update connection status based on heartbeat age
                if age > 5.0:  # No heartbeat for 5 seconds
                    px4_mavlink_connection.labels(drone_id=drone_id).set(0)
                else:
                    px4_mavlink_connection.labels(drone_id=drone_id).set(1)

        except Exception as e:
            print(f"Error collecting metrics from {drone_id}: {e}")
            px4_mavlink_connection.labels(drone_id=drone_id).set(0)

    def monitor_drone(self, drone_id: str):
        """Continuous monitoring thread for a single drone"""
        print(f"Starting monitoring thread for {drone_id}")

        while self.running:
            try:
                self.collect_drone_metrics(drone_id)
                time.sleep(0.1)  # High frequency monitoring (10 Hz)
            except Exception as e:
                print(f"Error in monitoring thread for {drone_id}: {e}")
                time.sleep(1.0)

    def run(self):
        """Start monitoring all drones"""
        print("Starting PX4 MAVLink monitoring...")

        # Connect to all drones
        self.connect_drones()

        # Start monitoring thread for each drone
        threads = []
        for drone_id in self.mavlink_connections.keys():
            thread = threading.Thread(target=self.monitor_drone, args=(drone_id,), daemon=True)
            thread.start()
            threads.append(thread)

        print(f"Monitoring {len(threads)} drones...")

        try:
            # Keep main thread alive
            while self.running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nShutting down PX4 MAVLink Exporter...")
            self.running = False

        # Wait for threads to finish
        for thread in threads:
            thread.join(timeout=2.0)


def main():
    """Main entry point"""
    print("Starting PX4 MAVLink Metrics Exporter for Prometheus...")

    # Define drone connections (can be configured via environment variables)
    num_drones = int(os.getenv('PX4_NUM_DRONES', '4'))
    base_port = int(os.getenv('PX4_BASE_PORT', '14550'))

    drone_connections = []
    for i in range(num_drones):
        drone_id = f"drone-{i+1}"
        # Connect via UDP to PX4 containers (bridge network DNS)
        connection_string = f"udpout:px4-drone-{i+1}:{base_port + (i*10)}"
        drone_connections.append({'id': drone_id, 'connection': connection_string})

    # Start Prometheus HTTP server
    metrics_port = int(os.getenv('METRICS_PORT', '9202'))
    start_http_server(metrics_port)
    print(f"Prometheus metrics server started on port {metrics_port}")

    # Create and run exporter
    exporter = PX4MAVLinkExporter(drone_connections)
    exporter.run()


if __name__ == '__main__':
    main()
