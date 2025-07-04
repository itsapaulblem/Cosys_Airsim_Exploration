#!/usr/bin/env python3

import socket
import time

def test_container_connectivity():
    """Test UDP connectivity to container IP"""
    container_ip = "172.25.0.10"
    port = 14581
    
    print(f"🔍 Testing UDP connectivity to {container_ip}:{port}")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.sendto(b'test', (container_ip, port))
        print(f"✅ UDP to container IP works!")
        sock.close()
        return True
    except Exception as e:
        print(f"❌ UDP to container IP failed: {e}")
        return False

def test_localhost_connectivity():
    """Test UDP connectivity to localhost"""
    port = 14581
    
    print(f"🔍 Testing UDP connectivity to localhost:{port}")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.sendto(b'test', ('127.0.0.1', port))
        print(f"✅ UDP to localhost works!")
        sock.close()
        return True
    except Exception as e:
        print(f"❌ UDP to localhost failed: {e}")
        return False

if __name__ == "__main__":
    print("🧪 Container Connectivity Test")
    print("=" * 30)
    
    container_works = test_container_connectivity()
    localhost_works = test_localhost_connectivity()
    
    print("\n📊 Results:")
    print(f"Container IP (172.25.0.10): {'✅ Works' if container_works else '❌ Failed'}")
    print(f"Localhost (127.0.0.1): {'✅ Works' if localhost_works else '❌ Failed'}")
    
    if container_works:
        print("\n🎉 Container IP connectivity confirmed!")
        print("💡 AirSim should work with ControlIp: '172.25.0.10'")
    elif localhost_works:
        print("\n⚠️  Only localhost works - use port forwarding approach")
        print("💡 AirSim should work with ControlIp: '127.0.0.1'")
    else:
        print("\n❌ Neither approach works - check Docker configuration") 