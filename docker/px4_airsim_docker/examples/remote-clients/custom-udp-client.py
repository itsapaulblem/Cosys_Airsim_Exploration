#!/usr/bin/env python3
"""
Custom UDP MAVLink Client Example
Demonstrates low-level MAVLink communication with remote PX4 server
"""

import socket
import struct
import time
import argparse
import sys
from typing import Optional, Tuple, Dict, Any

class MAVLinkMessage:
    """Simple MAVLink message parser and generator"""
    
    # MAVLink v2.0 constants
    MAVLINK_STX = 0xFD
    
    # Common message IDs
    MSG_ID_HEARTBEAT = 0
    MSG_ID_SYS_STATUS = 1
    MSG_ID_SYSTEM_TIME = 2
    MSG_ID_PING = 4
    MSG_ID_CHANGE_OPERATOR_CONTROL = 5
    MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6
    MSG_ID_AUTH_KEY = 7
    MSG_ID_SET_MODE = 11
    MSG_ID_PARAM_REQUEST_READ = 20
    MSG_ID_PARAM_REQUEST_LIST = 21
    MSG_ID_PARAM_VALUE = 22
    MSG_ID_PARAM_SET = 23
    MSG_ID_GPS_RAW_INT = 24
    MSG_ID_GPS_STATUS = 25
    MSG_ID_SCALED_IMU = 26
    MSG_ID_RAW_IMU = 27
    MSG_ID_RAW_PRESSURE = 28
    MSG_ID_SCALED_PRESSURE = 29
    MSG_ID_ATTITUDE = 30
    MSG_ID_ATTITUDE_QUATERNION = 31
    MSG_ID_LOCAL_POSITION_NED = 32
    MSG_ID_GLOBAL_POSITION_INT = 33
    MSG_ID_RC_CHANNELS_SCALED = 34
    MSG_ID_RC_CHANNELS_RAW = 35
    MSG_ID_SERVO_OUTPUT_RAW = 36
    MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37
    MSG_ID_MISSION_WRITE_PARTIAL_LIST = 38
    MSG_ID_MISSION_ITEM = 39
    MSG_ID_MISSION_REQUEST = 40
    MSG_ID_MISSION_SET_CURRENT = 41
    MSG_ID_MISSION_CURRENT = 42
    MSG_ID_MISSION_REQUEST_LIST = 43
    MSG_ID_MISSION_COUNT = 44
    MSG_ID_MISSION_CLEAR_ALL = 45
    MSG_ID_MISSION_ITEM_REACHED = 46
    MSG_ID_MISSION_ACK = 47
    MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48
    MSG_ID_GPS_GLOBAL_ORIGIN = 49
    MSG_ID_PARAM_MAP_RC = 50
    MSG_ID_MISSION_REQUEST_INT = 51
    MSG_ID_SAFETY_SET_ALLOWED_AREA = 54
    MSG_ID_SAFETY_ALLOWED_AREA = 55
    MSG_ID_ATTITUDE_QUATERNION_COV = 61
    MSG_ID_NAV_CONTROLLER_OUTPUT = 62
    MSG_ID_GLOBAL_POSITION_INT_COV = 63
    MSG_ID_LOCAL_POSITION_NED_COV = 64
    MSG_ID_RC_CHANNELS = 65
    MSG_ID_REQUEST_DATA_STREAM = 66
    MSG_ID_DATA_STREAM = 67
    MSG_ID_MANUAL_CONTROL = 69
    MSG_ID_RC_CHANNELS_OVERRIDE = 70
    MSG_ID_MISSION_ITEM_INT = 73
    MSG_ID_VFR_HUD = 74
    MSG_ID_COMMAND_INT = 75
    MSG_ID_COMMAND_LONG = 76
    MSG_ID_COMMAND_ACK = 77
    MSG_ID_MANUAL_SETPOINT = 81
    MSG_ID_SET_ATTITUDE_TARGET = 82
    MSG_ID_ATTITUDE_TARGET = 83
    MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84
    MSG_ID_POSITION_TARGET_LOCAL_NED = 85
    MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86
    MSG_ID_POSITION_TARGET_GLOBAL_INT = 87
    MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89
    MSG_ID_HIL_STATE = 90
    MSG_ID_HIL_CONTROLS = 91
    MSG_ID_HIL_RC_INPUTS_RAW = 92
    MSG_ID_OPTICAL_FLOW = 100
    MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE = 101
    MSG_ID_VISION_POSITION_ESTIMATE = 102
    MSG_ID_VISION_SPEED_ESTIMATE = 103
    MSG_ID_VICON_POSITION_ESTIMATE = 104
    MSG_ID_HIGHRES_IMU = 105
    MSG_ID_OPTICAL_FLOW_RAD = 106
    MSG_ID_HIL_SENSOR = 107
    MSG_ID_SIM_STATE = 108
    MSG_ID_RADIO_STATUS = 109
    MSG_ID_FILE_TRANSFER_PROTOCOL = 110
    MSG_ID_TIMESYNC = 111
    MSG_ID_CAMERA_TRIGGER = 112
    MSG_ID_HIL_GPS = 113
    MSG_ID_HIL_OPTICAL_FLOW = 114
    MSG_ID_HIL_STATE_QUATERNION = 115
    MSG_ID_SCALED_IMU2 = 116
    MSG_ID_LOG_REQUEST_LIST = 117
    MSG_ID_LOG_ENTRY = 118
    MSG_ID_LOG_REQUEST_DATA = 119
    MSG_ID_LOG_DATA = 120
    MSG_ID_LOG_ERASE = 121
    MSG_ID_LOG_REQUEST_END = 122
    MSG_ID_GPS_INJECT_DATA = 123
    MSG_ID_GPS2_RAW = 124
    MSG_ID_POWER_STATUS = 125
    MSG_ID_SERIAL_CONTROL = 126
    MSG_ID_GPS_RTK = 127
    MSG_ID_GPS2_RTK = 128
    MSG_ID_SCALED_IMU3 = 129
    MSG_ID_DATA_TRANSMISSION_HANDSHAKE = 130
    MSG_ID_ENCAPSULATED_DATA = 131
    MSG_ID_DISTANCE_SENSOR = 132
    MSG_ID_TERRAIN_REQUEST = 133
    MSG_ID_TERRAIN_DATA = 134
    MSG_ID_TERRAIN_CHECK = 135
    MSG_ID_TERRAIN_REPORT = 136
    MSG_ID_SCALED_PRESSURE2 = 137
    MSG_ID_ATT_POS_MOCAP = 138
    MSG_ID_SET_ACTUATOR_CONTROL_TARGET = 139
    MSG_ID_ACTUATOR_CONTROL_TARGET = 140
    MSG_ID_ALTITUDE = 141
    MSG_ID_RESOURCE_REQUEST = 142
    MSG_ID_SCALED_PRESSURE3 = 143
    MSG_ID_FOLLOW_TARGET = 144
    MSG_ID_CONTROL_SYSTEM_STATE = 146
    MSG_ID_BATTERY_STATUS = 147
    MSG_ID_AUTOPILOT_VERSION = 148
    MSG_ID_LANDING_TARGET = 149
    
    @staticmethod
    def create_heartbeat(system_id: int = 255, component_id: int = 0) -> bytes:
        """Create heartbeat message"""
        # Heartbeat payload: type(1), autopilot(1), base_mode(1), custom_mode(4), system_status(1), mavlink_version(1)
        payload = struct.pack('<BBBIBB', 
                             6,    # type: MAV_TYPE_GCS
                             0,    # autopilot: MAV_AUTOPILOT_INVALID
                             0,    # base_mode
                             0,    # custom_mode
                             4,    # system_status: MAV_STATE_ACTIVE
                             3)    # mavlink_version
        
        return MAVLinkMessage._create_message(MAVLinkMessage.MSG_ID_HEARTBEAT, payload, system_id, component_id)
    
    @staticmethod
    def create_request_autopilot_version(system_id: int = 255, component_id: int = 0) -> bytes:
        """Create request for autopilot version"""
        # Command_long payload for REQUEST_AUTOPILOT_CAPABILITIES
        payload = struct.pack('<fffffffHBBBB',
                             520,   # command: MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                             0,     # confirmation
                             1,     # param1: Request autopilot version
                             0, 0, 0, 0, 0,  # param2-7
                             1,     # target_system
                             1,     # target_component
                             0)     # padding
        
        return MAVLinkMessage._create_message(MAVLinkMessage.MSG_ID_COMMAND_LONG, payload, system_id, component_id)
    
    @staticmethod
    def create_param_request_list(system_id: int = 255, component_id: int = 0) -> bytes:
        """Create parameter request list message"""
        payload = struct.pack('<BB', 1, 1)  # target_system, target_component
        return MAVLinkMessage._create_message(MAVLinkMessage.MSG_ID_PARAM_REQUEST_LIST, payload, system_id, component_id)
    
    @staticmethod
    def _create_message(msg_id: int, payload: bytes, system_id: int, component_id: int) -> bytes:
        """Create complete MAVLink message"""
        payload_len = len(payload)
        seq = 0  # Sequence number (would increment in real implementation)
        
        # Create header
        header = struct.pack('<BBBBBBB',
                           MAVLinkMessage.MAVLINK_STX,  # STX
                           payload_len,                  # Payload length
                           0,                           # Incompatible flags
                           0,                           # Compatible flags
                           seq,                         # Sequence
                           system_id,                   # System ID
                           component_id)                # Component ID
        
        # Add message ID (3 bytes for MAVLink 2.0)
        msg_id_bytes = struct.pack('<I', msg_id)[:3]
        
        # Calculate checksum
        checksum_data = header[1:] + msg_id_bytes + payload
        checksum = MAVLinkMessage._calculate_checksum(checksum_data, msg_id)
        
        return header + msg_id_bytes + payload + struct.pack('<H', checksum)
    
    @staticmethod
    def _calculate_checksum(data: bytes, msg_id: int) -> int:
        """Calculate MAVLink checksum"""
        # Simple checksum calculation (real implementation would use CRC table)
        checksum = 0xFFFF
        for byte in data:
            checksum ^= byte << 8
            for _ in range(8):
                if checksum & 0x8000:
                    checksum = (checksum << 1) ^ 0x1021
                else:
                    checksum = checksum << 1
                checksum &= 0xFFFF
        return checksum
    
    @staticmethod
    def parse_message(data: bytes) -> Optional[Dict[str, Any]]:
        """Parse received MAVLink message"""
        if len(data) < 12:  # Minimum MAVLink 2.0 message size
            return None
            
        if data[0] != MAVLinkMessage.MAVLINK_STX:
            return None
            
        payload_len = data[1]
        if len(data) < 12 + payload_len:
            return None
            
        # Extract header fields
        incompat_flags = data[2]
        compat_flags = data[3]
        seq = data[4]
        system_id = data[5]
        component_id = data[6]
        
        # Extract message ID (3 bytes)
        msg_id = struct.unpack('<I', data[7:10] + b'\x00')[0]
        
        # Extract payload
        payload = data[10:10+payload_len]
        
        return {
            'msg_id': msg_id,
            'system_id': system_id,
            'component_id': component_id,
            'seq': seq,
            'payload': payload,
            'payload_len': payload_len
        }

class RemoteMAVLinkClient:
    """Remote MAVLink client for PX4 server communication"""
    
    def __init__(self, server_ip: str, server_port: int = 14540, local_port: int = 14550):
        self.server_ip = server_ip
        self.server_port = server_port
        self.local_port = local_port
        self.socket = None
        self.connected = False
        self.system_id = 255  # Ground Control Station ID
        self.component_id = 0
        self.last_heartbeat = 0
        self.message_stats = {}
        
    def connect(self) -> bool:
        """Connect to remote PX4 server"""
        try:
            print(f"🔗 Connecting to PX4 server at {self.server_ip}:{self.server_port}")
            
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(5.0)  # 5 second timeout
            
            # Bind to local port for receiving
            try:
                self.socket.bind(('', self.local_port))
                print(f"   Listening on local port: {self.local_port}")
            except OSError:
                print(f"   Warning: Could not bind to port {self.local_port}, using random port")
                self.socket.bind(('', 0))
                self.local_port = self.socket.getsockname()[1]
                print(f"   Using local port: {self.local_port}")
            
            # Send initial heartbeat
            heartbeat = MAVLinkMessage.create_heartbeat(self.system_id, self.component_id)
            self.socket.sendto(heartbeat, (self.server_ip, self.server_port))
            
            print("   Sent initial heartbeat, waiting for response...")
            
            # Wait for response
            try:
                data, addr = self.socket.recvfrom(4096)
                message = MAVLinkMessage.parse_message(data)
                
                if message:
                    print(f"✅ Connected! Received message ID {message['msg_id']} from system {message['system_id']}")
                    self.connected = True
                    self.last_heartbeat = time.time()
                    return True
                else:
                    print("❌ Received invalid response")
                    return False
                    
            except socket.timeout:
                print("❌ Connection timeout - no response from server")
                return False
                
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
            
    def disconnect(self) -> None:
        """Disconnect from server"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        print("👋 Disconnected from server")
        
    def send_heartbeat(self) -> bool:
        """Send heartbeat to maintain connection"""
        if not self.connected or not self.socket:
            return False
            
        try:
            heartbeat = MAVLinkMessage.create_heartbeat(self.system_id, self.component_id)
            self.socket.sendto(heartbeat, (self.server_ip, self.server_port))
            self.last_heartbeat = time.time()
            return True
        except Exception as e:
            print(f"❌ Failed to send heartbeat: {e}")
            return False
            
    def request_autopilot_version(self) -> bool:
        """Request autopilot version information"""
        if not self.connected or not self.socket:
            return False
            
        try:
            message = MAVLinkMessage.create_request_autopilot_version(self.system_id, self.component_id)
            self.socket.sendto(message, (self.server_ip, self.server_port))
            print("📤 Sent autopilot version request")
            return True
        except Exception as e:
            print(f"❌ Failed to request autopilot version: {e}")
            return False
            
    def request_parameters(self) -> bool:
        """Request parameter list"""
        if not self.connected or not self.socket:
            return False
            
        try:
            message = MAVLinkMessage.create_param_request_list(self.system_id, self.component_id)
            self.socket.sendto(message, (self.server_ip, self.server_port))
            print("📤 Sent parameter list request")
            return True
        except Exception as e:
            print(f"❌ Failed to request parameters: {e}")
            return False
            
    def receive_messages(self, duration: int = 30) -> Dict[int, int]:
        """Receive and process messages for specified duration"""
        if not self.connected or not self.socket:
            return {}
            
        print(f"📡 Listening for messages for {duration} seconds...")
        
        start_time = time.time()
        message_count = {}
        total_messages = 0
        
        while time.time() - start_time < duration:
            try:
                # Set short timeout for responsive heartbeat sending
                self.socket.settimeout(1.0)
                
                # Receive message
                data, addr = self.socket.recvfrom(4096)
                message = MAVLinkMessage.parse_message(data)
                
                if message:
                    msg_id = message['msg_id']
                    message_count[msg_id] = message_count.get(msg_id, 0) + 1
                    total_messages += 1
                    
                    # Update message stats
                    self.message_stats[msg_id] = self.message_stats.get(msg_id, 0) + 1
                    
                    # Print important messages
                    if msg_id == MAVLinkMessage.MSG_ID_HEARTBEAT:
                        print(f"   💓 Heartbeat from system {message['system_id']}")
                    elif msg_id == MAVLinkMessage.MSG_ID_AUTOPILOT_VERSION:
                        print(f"   ℹ️ Autopilot version received")
                    elif msg_id == MAVLinkMessage.MSG_ID_PARAM_VALUE:
                        print(f"   ⚙️ Parameter value received")
                    elif msg_id == MAVLinkMessage.MSG_ID_SYS_STATUS:
                        print(f"   📊 System status received")
                    elif msg_id == MAVLinkMessage.MSG_ID_GLOBAL_POSITION_INT:
                        print(f"   🌍 Global position received")
                    
                # Send periodic heartbeat
                if time.time() - self.last_heartbeat > 5:
                    self.send_heartbeat()
                    
            except socket.timeout:
                # Send heartbeat on timeout
                if time.time() - self.last_heartbeat > 5:
                    self.send_heartbeat()
                continue
                
            except Exception as e:
                print(f"❌ Error receiving message: {e}")
                break
                
        print(f"✅ Received {total_messages} messages")
        return message_count
        
    def print_message_stats(self, message_count: Dict[int, int]) -> None:
        """Print message statistics"""
        if not message_count:
            print("📊 No messages received")
            return
            
        print("\n📊 Message Statistics:")
        print("-" * 40)
        
        # Known message names
        message_names = {
            MAVLinkMessage.MSG_ID_HEARTBEAT: "HEARTBEAT",
            MAVLinkMessage.MSG_ID_SYS_STATUS: "SYS_STATUS",
            MAVLinkMessage.MSG_ID_SYSTEM_TIME: "SYSTEM_TIME",
            MAVLinkMessage.MSG_ID_ATTITUDE: "ATTITUDE",
            MAVLinkMessage.MSG_ID_GLOBAL_POSITION_INT: "GLOBAL_POSITION_INT",
            MAVLinkMessage.MSG_ID_LOCAL_POSITION_NED: "LOCAL_POSITION_NED",
            MAVLinkMessage.MSG_ID_GPS_RAW_INT: "GPS_RAW_INT",
            MAVLinkMessage.MSG_ID_VFR_HUD: "VFR_HUD",
            MAVLinkMessage.MSG_ID_SERVO_OUTPUT_RAW: "SERVO_OUTPUT_RAW",
            MAVLinkMessage.MSG_ID_RC_CHANNELS: "RC_CHANNELS",
            MAVLinkMessage.MSG_ID_BATTERY_STATUS: "BATTERY_STATUS",
            MAVLinkMessage.MSG_ID_AUTOPILOT_VERSION: "AUTOPILOT_VERSION",
            MAVLinkMessage.MSG_ID_PARAM_VALUE: "PARAM_VALUE"
        }
        
        # Sort by count
        sorted_messages = sorted(message_count.items(), key=lambda x: x[1], reverse=True)
        
        for msg_id, count in sorted_messages:
            name = message_names.get(msg_id, f"UNKNOWN_{msg_id}")
            print(f"   {name:<25} : {count:>6} messages")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Custom UDP MAVLink Client Example")
    parser.add_argument("--server-ip", default="localhost", help="PX4 server IP address")
    parser.add_argument("--server-port", type=int, default=14540, help="PX4 server port")
    parser.add_argument("--local-port", type=int, default=14550, help="Local listening port")
    parser.add_argument("--duration", type=int, default=30, help="Message listening duration")
    parser.add_argument("--demo", choices=["connect", "version", "params", "monitor"], 
                       default="connect", help="Demo to run")
    
    args = parser.parse_args()
    
    print("🚁 Custom UDP MAVLink Client Example")
    print("=" * 50)
    print(f"Server: {args.server_ip}:{args.server_port}")
    print(f"Local port: {args.local_port}")
    print(f"Demo: {args.demo}")
    print("")
    
    # Create client
    client = RemoteMAVLinkClient(args.server_ip, args.server_port, args.local_port)
    
    try:
        # Connect to server
        if not client.connect():
            print("❌ Failed to connect to PX4 server")
            sys.exit(1)
            
        # Run selected demo
        if args.demo == "connect":
            print("✅ Connection test successful")
            time.sleep(5)
            
        elif args.demo == "version":
            client.request_autopilot_version()
            message_count = client.receive_messages(10)
            client.print_message_stats(message_count)
            
        elif args.demo == "params":
            client.request_parameters()
            message_count = client.receive_messages(20)
            client.print_message_stats(message_count)
            
        elif args.demo == "monitor":
            message_count = client.receive_messages(args.duration)
            client.print_message_stats(message_count)
            
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        
    finally:
        client.disconnect()

if __name__ == "__main__":
    # Usage examples:
    
    # Basic connection test
    # python custom-udp-client.py --server-ip localhost --demo connect
    
    # VPN connection
    # python custom-udp-client.py --server-ip 10.10.0.11 --demo monitor --duration 60
    
    # Remote server connection
    # python custom-udp-client.py --server-ip YOUR_SERVER_IP --demo version
    
    # Monitor messages with custom ports
    # python custom-udp-client.py --server-ip 10.10.0.12 --server-port 14541 --demo monitor
    
    main()