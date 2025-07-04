import json
from pathlib import Path

try:
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    print(f"Checking: {settings_path}")
    print(f"Exists: {settings_path.exists()}")
    
    if settings_path.exists():
        with open(settings_path, 'r') as f:
            settings = json.load(f)
        
        vehicles = settings.get("Vehicles", {})
        print(f"Found {len(vehicles)} vehicles:")
        for name, config in vehicles.items():
            tcp_port = config.get("TcpPort", 0)
            print(f"  {name}: TCP={tcp_port}")
            
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc() 