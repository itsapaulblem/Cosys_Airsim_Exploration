#!/usr/bin/env python3
"""
MAVSDK Python Remote Connection Example
Demonstrates connecting to remote PX4 server via different methods
"""

import asyncio
import argparse
import sys
import time
from typing import Optional

try:
    from mavsdk import System
    from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
    from mavsdk.mission import MissionItem, MissionPlan
except ImportError:
    print("❌ MAVSDK not installed. Install with: pip install mavsdk")
    sys.exit(1)

class PX4RemoteClient:
    """Remote PX4 client using MAVSDK"""
    
    def __init__(self, connection_string: str):
        self.connection_string = connection_string
        self.drone = System()
        self.connected = False
        
    async def connect(self, timeout: int = 30) -> bool:
        """Connect to remote PX4 server"""
        print(f"🔗 Connecting to PX4 server: {self.connection_string}")
        
        try:
            await self.drone.connect(system_address=self.connection_string)
            
            # Wait for connection with timeout
            connection_timeout = time.time() + timeout
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    print("✅ Connected to PX4 server")
                    return True
                    
                if time.time() > connection_timeout:
                    print(f"❌ Connection timeout after {timeout} seconds")
                    return False
                    
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
            
    async def get_system_info(self) -> None:
        """Get and display system information"""
        if not self.connected:
            print("❌ Not connected to PX4")
            return
            
        print("\n📊 System Information:")
        
        try:
            # Get vehicle info
            async for health in self.drone.telemetry.health():
                print(f"   Health: {'✅ Healthy' if health.is_global_position_ok else '⚠️ Unhealthy'}")
                break
                
            # Get GPS info
            async for gps_info in self.drone.telemetry.gps_info():
                print(f"   GPS Satellites: {gps_info.num_satellites}")
                print(f"   GPS Fix Type: {gps_info.fix_type}")
                break
                
            # Get battery info
            async for battery in self.drone.telemetry.battery():
                print(f"   Battery: {battery.remaining_percent:.1f}%")
                print(f"   Voltage: {battery.voltage_v:.2f}V")
                break
                
            # Get position
            async for position in self.drone.telemetry.position():
                print(f"   Position: {position.latitude_deg:.6f}, {position.longitude_deg:.6f}")
                print(f"   Altitude: {position.relative_altitude_m:.2f}m")
                break
                
        except Exception as e:
            print(f"❌ Error getting system info: {e}")
            
    async def arm_and_takeoff(self, altitude: float = 5.0) -> bool:
        """Arm drone and takeoff to specified altitude"""
        if not self.connected:
            print("❌ Not connected to PX4")
            return False
            
        try:
            print(f"\n🚁 Arming and taking off to {altitude}m...")
            
            # Check if vehicle is ready
            async for health in self.drone.telemetry.health():
                if not health.is_global_position_ok:
                    print("❌ Vehicle not ready for takeoff (no GPS lock)")
                    return False
                break
                
            # Arm
            print("   Arming...")
            await self.drone.action.arm()
            
            # Takeoff
            print(f"   Taking off to {altitude}m...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            
            # Wait for takeoff completion
            print("   Waiting for takeoff completion...")
            while True:
                async for position in self.drone.telemetry.position():
                    if position.relative_altitude_m > altitude * 0.9:
                        print("✅ Takeoff completed")
                        return True
                    break
                await asyncio.sleep(1)
                
        except Exception as e:
            print(f"❌ Takeoff failed: {e}")
            return False
            
    async def land(self) -> bool:
        """Land the drone"""
        if not self.connected:
            print("❌ Not connected to PX4")
            return False
            
        try:
            print("\n🛬 Landing...")
            await self.drone.action.land()
            
            # Wait for landing completion
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    print("✅ Landing completed")
                    return True
                await asyncio.sleep(1)
                
        except Exception as e:
            print(f"❌ Landing failed: {e}")
            return False
            
    async def execute_mission(self) -> bool:
        """Execute a simple mission"""
        if not self.connected:
            print("❌ Not connected to PX4")
            return False
            
        try:
            print("\n🗺️ Executing simple mission...")
            
            # Get current position for mission waypoints
            async for position in self.drone.telemetry.position():
                lat = position.latitude_deg
                lon = position.longitude_deg
                break
            else:
                print("❌ Could not get current position")
                return False
                
            # Create mission waypoints
            mission_items = [
                MissionItem(lat, lon, 10, 10, True, float('nan'), float('nan'), 
                           MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')),
                MissionItem(lat + 0.0001, lon, 10, 10, True, float('nan'), float('nan'),
                           MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')),
                MissionItem(lat + 0.0001, lon + 0.0001, 10, 10, True, float('nan'), float('nan'),
                           MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')),
                MissionItem(lat, lon + 0.0001, 10, 10, True, float('nan'), float('nan'),
                           MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')),
                MissionItem(lat, lon, 10, 10, True, float('nan'), float('nan'),
                           MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
            ]
            
            mission_plan = MissionPlan(mission_items)
            
            # Upload mission
            print("   Uploading mission...")
            await self.drone.mission.upload_mission(mission_plan)
            
            # Start mission
            print("   Starting mission...")
            await self.drone.mission.start_mission()
            
            # Monitor mission progress
            async for mission_progress in self.drone.mission.mission_progress():
                print(f"   Mission progress: {mission_progress.current}/{mission_progress.total}")
                if mission_progress.current == mission_progress.total:
                    print("✅ Mission completed")
                    return True
                    
        except Exception as e:
            print(f"❌ Mission execution failed: {e}")
            return False
            
    async def monitor_telemetry(self, duration: int = 30) -> None:
        """Monitor telemetry for specified duration"""
        if not self.connected:
            print("❌ Not connected to PX4")
            return
            
        print(f"\n📡 Monitoring telemetry for {duration} seconds...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                # Get latest telemetry
                async for position in self.drone.telemetry.position():
                    async for attitude in self.drone.telemetry.attitude_euler():
                        async for battery in self.drone.telemetry.battery():
                            print(f"\r   Alt: {position.relative_altitude_m:.1f}m | "
                                  f"Yaw: {attitude.yaw_deg:.1f}° | "
                                  f"Bat: {battery.remaining_percent:.0f}% | "
                                  f"Time: {time.time() - start_time:.0f}s", end="")
                            break
                        break
                    break
                    
                await asyncio.sleep(1)
                
            except Exception as e:
                print(f"\n❌ Telemetry error: {e}")
                break
                
        print(f"\n✅ Telemetry monitoring completed")

def get_connection_string(method: str, server_ip: str) -> str:
    """Generate connection string based on method"""
    if method == "direct":
        return f"udp://{server_ip}:14540"
    elif method == "vpn":
        return "udp://10.10.0.11:14540"  # VPN drone 1
    elif method == "local":
        return "udp://localhost:14540"
    else:
        return f"udp://{server_ip}:14540"

async def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="MAVSDK Remote PX4 Connection Example")
    parser.add_argument("--method", choices=["direct", "vpn", "local", "custom"], 
                       default="vpn", help="Connection method")
    parser.add_argument("--server-ip", default="localhost", 
                       help="Server IP address (for direct/custom connection)")
    parser.add_argument("--connection", help="Custom connection string")
    parser.add_argument("--demo", choices=["info", "takeoff", "mission", "monitor"], 
                       default="info", help="Demo to run")
    parser.add_argument("--timeout", type=int, default=30, help="Connection timeout")
    
    args = parser.parse_args()
    
    # Determine connection string
    if args.connection:
        connection_string = args.connection
    else:
        connection_string = get_connection_string(args.method, args.server_ip)
    
    print("🚁 MAVSDK Remote PX4 Connection Example")
    print("=" * 50)
    print(f"Connection method: {args.method}")
    print(f"Connection string: {connection_string}")
    print(f"Demo: {args.demo}")
    print("")
    
    # Create client and connect
    client = PX4RemoteClient(connection_string)
    
    if not await client.connect(timeout=args.timeout):
        print("❌ Failed to connect to PX4 server")
        sys.exit(1)
        
    # Run selected demo
    try:
        if args.demo == "info":
            await client.get_system_info()
            
        elif args.demo == "takeoff":
            await client.get_system_info()
            if await client.arm_and_takeoff():
                await asyncio.sleep(5)  # Hover for 5 seconds
                await client.land()
                
        elif args.demo == "mission":
            await client.get_system_info()
            if await client.arm_and_takeoff():
                await asyncio.sleep(2)
                await client.execute_mission()
                await client.land()
                
        elif args.demo == "monitor":
            await client.get_system_info()
            await client.monitor_telemetry(duration=60)
            
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
        
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        
    finally:
        print("\n👋 Disconnecting...")

if __name__ == "__main__":
    # Connection examples for different scenarios:
    
    # VPN connection (recommended for remote access)
    # python mavsdk-python-example.py --method vpn --demo info
    
    # Direct internet connection
    # python mavsdk-python-example.py --method direct --server-ip YOUR_SERVER_IP --demo info
    
    # Custom connection string
    # python mavsdk-python-example.py --connection udp://10.10.0.12:14540 --demo info
    
    # Local connection (development)
    # python mavsdk-python-example.py --method local --demo info
    
    asyncio.run(main())