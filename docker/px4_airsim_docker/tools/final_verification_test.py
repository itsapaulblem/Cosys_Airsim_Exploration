#!/usr/bin/env python3
"""
Final verification test for MAVLink connectivity after all fixes
"""
import socket
import subprocess
import time

def test_port_accessibility():
    """Test if QGC ports are accessible from WSL2"""
    print("🔍 Testing Port Accessibility from WSL2")
    print("-" * 40)
    
    ports_accessible = []
    for port in [14541, 14542, 14543, 14544, 14545]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            sock.connect(('127.0.0.1', port))
            ports_accessible.append(port)
            print(f"✅ Port {port}: Accessible")
            sock.close()
        except:
            print(f"❌ Port {port}: Not accessible")
    
    return ports_accessible

def test_container_health():
    """Test container health"""
    print("\n🐳 Testing Container Health")
    print("-" * 30)
    
    healthy_containers = []
    for i in range(1, 6):
        container = f"px4-bridge-drone-{i}"
        try:
            result = subprocess.run(['docker', 'inspect', '--format={{.State.Health.Status}}', container], 
                                 capture_output=True, text=True)
            if 'healthy' in result.stdout:
                healthy_containers.append(i)
                print(f"✅ Drone {i}: Healthy")
            else:
                print(f"⚠️ Drone {i}: {result.stdout.strip()}")
        except:
            print(f"❌ Drone {i}: Not found")
    
    return healthy_containers

def test_windows_connectivity_readiness():
    """Test readiness for Windows host connectivity"""
    print("\n🪟 Windows Connectivity Readiness")
    print("-" * 35)
    
    # Check WSL2 to Windows IP
    try:
        result = subprocess.run(['ip', 'route', 'show'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'default via' in line:
                windows_ip = line.split()[2]
                print(f"✅ Windows Host IP: {windows_ip}")
                break
    except:
        print("❌ Could not determine Windows host IP")
    
    # Check Docker port mappings
    try:
        result = subprocess.run(['docker', 'port', 'px4-bridge-drone-1'], capture_output=True, text=True)
        qgc_mapped = '14541/udp -> 0.0.0.0:14541' in result.stdout
        if qgc_mapped:
            print("✅ QGC ports mapped to 0.0.0.0 (accessible from Windows)")
        else:
            print("❌ QGC ports not properly mapped for Windows access")
            
        return qgc_mapped
    except:
        print("❌ Could not check Docker port mappings")
        return False

def main():
    print("🚁 Final MAVLink Connectivity Verification")
    print("=" * 50)
    print("Testing all fixes applied for WSL2 + Docker + Windows setup")
    print()
    
    # Test 1: Port accessibility
    accessible_ports = test_port_accessibility()
    
    # Test 2: Container health  
    healthy_containers = test_container_health()
    
    # Test 3: Windows readiness
    windows_ready = test_windows_connectivity_readiness()
    
    # Summary
    print("\n" + "=" * 50)
    print("📊 FINAL VERIFICATION SUMMARY")
    print("=" * 50)
    
    print(f"🔗 Accessible Ports: {len(accessible_ports)}/5 ({accessible_ports})")
    print(f"🐳 Healthy Containers: {len(healthy_containers)}/5 ({healthy_containers})")
    print(f"🪟 Windows Ready: {'✅ YES' if windows_ready else '❌ NO'}")
    
    overall_success = (len(accessible_ports) >= 3 and len(healthy_containers) >= 3 and windows_ready)
    
    print(f"\n🎯 Overall Status: {'✅ READY FOR TESTING' if overall_success else '⚠️ NEEDS ATTENTION'}")
    
    if overall_success:
        print("\n🎮 Next Steps for QGroundControl:")
        print("1. Run Windows Firewall script (as Administrator):")
        print("   .\\tools\\setup_windows_firewall.ps1")
        print("2. Configure QGroundControl:")
        print("   • Connection Type: UDP")
        print("   • Host: localhost (or 127.0.0.1)")
        print(f"   • Port: {accessible_ports[0] if accessible_ports else 14541}")
        print("   • Auto Connect: Enable")
        print("3. Test connection - should receive MAVLink heartbeat")
    else:
        print("\n🔧 Issues requiring attention:")
        if len(accessible_ports) < 3:
            print("   • Few accessible ports - check Docker port mappings")
        if len(healthy_containers) < 3:
            print("   • Container health issues - restart containers")
        if not windows_ready:
            print("   • Windows connectivity not ready - check port bindings")

if __name__ == "__main__":
    main()