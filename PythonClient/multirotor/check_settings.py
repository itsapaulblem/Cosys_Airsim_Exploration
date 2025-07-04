#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim

def check_airsim_settings():
    """Check what settings AirSim is actually using"""
    print("🔍 Checking AirSim Settings")
    print("=" * 30)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        settings = client.getSettingsString()
        
        # Check for key settings
        container_ip_found = '172.25.0.10' in settings
        localhost_ip_found = '127.0.0.1' in settings
        px4_found = 'PX4Multirotor' in settings
        
        print(f"📊 Settings Analysis:")
        print(f"   Settings length: {len(settings)} characters")
        print(f"   Container IP (172.25.0.10): {'✅ Found' if container_ip_found else '❌ Not found'}")
        print(f"   Localhost IP (127.0.0.1): {'✅ Found' if localhost_ip_found else '❌ Not found'}")
        print(f"   PX4Multirotor: {'✅ Found' if px4_found else '❌ Not found'}")
        
        # Look for ControlIp specifically
        if 'ControlIp' in settings:
            print(f"   ControlIp setting: ✅ Found")
            # Try to extract the ControlIp value
            lines = settings.split('\n')
            for line in lines:
                if 'ControlIp' in line:
                    print(f"   -> {line.strip()}")
        else:
            print(f"   ControlIp setting: ❌ Not found")
        
        if container_ip_found:
            print("\n🎉 AirSim is using container IP settings!")
            return True
        elif localhost_ip_found:
            print("\n⚠️  AirSim is using localhost settings")
            print("💡 AirSim may need to be restarted to use new settings")
            return False
        else:
            print("\n❌ Unknown IP configuration")
            return False
            
    except Exception as e:
        print(f"❌ Failed to check settings: {e}")
        return False

if __name__ == "__main__":
    check_airsim_settings() 