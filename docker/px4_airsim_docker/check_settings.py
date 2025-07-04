import json
from pathlib import Path

settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"

with open(settings_path, 'r') as f:
    settings = json.load(f)

print("PawnPaths in settings:", "PawnPaths" in settings)

if "PawnPaths" in settings:
    print("PawnPaths configuration:")
    print(json.dumps(settings["PawnPaths"], indent=2))
else:
    print("PawnPaths not found in settings!")

print(f"\nSettings keys: {list(settings.keys())}")
print(f"Number of vehicles: {len(settings.get('Vehicles', {}))}") 