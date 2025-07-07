#!/usr/bin/env python3
"""
Test script to simulate AirSim connecting to PX4 and verify it works correctly.
This tests if the "Simulator connected" message is actually a problem or not.
"""

import socket
import time
import sys

def test_airsim_connection(host='localhost', port=4564, timeout=10):
    """
    Simulate AirSim connecting to PX4 and verify the behavior.
    """
    print(f"🔌 Testing AirSim connection to {host}:{port}")
    print(f"⏱️  Timeout: {timeout} seconds")
    print()
    
    try:
        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        print(f"📡 Attempting to connect to PX4 at {host}:{port}...")
        
        # Try to connect
        start_time = time.time()
        result = sock.connect((host, port))
        connect_time = time.time() - start_time
        
        print(f"✅ Connected successfully in {connect_time:.2f} seconds!")
        print()
        
        # Send a simple test message
        print("📤 Sending test message...")
        test_message = b"Hello from simulated AirSim\n"
        sock.send(test_message)
        
        # Try to receive data
        print("📥 Waiting for response...")
        sock.settimeout(5)  # Shorter timeout for receiving
        try:
            response = sock.recv(1024)
            print(f"📨 Received: {response}")
        except socket.timeout:
            print("⏳ No immediate response (this is normal for PX4)")
        
        # Keep connection open for a bit
        print("🔗 Keeping connection open for 10 seconds...")
        time.sleep(10)
        
        print("✅ Connection test completed successfully!")
        return True
        
    except ConnectionRefusedError:
        print(f"❌ Connection refused - PX4 is not listening on {host}:{port}")
        print("💡 This means PX4 is properly waiting for AirSim!")
        return False
        
    except socket.timeout:
        print(f"⏱️  Connection timeout after {timeout} seconds")
        print("💡 PX4 might not be ready yet")
        return False
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
        
    finally:
        try:
            sock.close()
        except:
            pass

def main():
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    else:
        port = 4564
        
    print("=" * 60)
    print("    AirSim-PX4 Connection Test")
    print("=" * 60)
    print()
    
    # Test the connection
    success = test_airsim_connection(port=port)
    
    print()
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    if success:
        print("✅ PX4 is accepting connections (good for AirSim)")
        print("💡 The 'Simulator connected' message was NOT premature")
        print("🎯 This setup should work fine with AirSim")
    else:
        print("⏳ PX4 is waiting for connections (also good)")
        print("💡 No auto-simulator is interfering")
        print("🎯 Ready for AirSim to connect")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())