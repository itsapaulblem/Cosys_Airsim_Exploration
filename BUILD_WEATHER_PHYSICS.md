# Building AirSim with Weather Physics

## Quick Build Instructions

### Windows (PowerShell)
```powershell
# Clean previous builds
.\clean_rebuild.bat

# Build AirSim
.\build.cmd

# Build Unreal Plugin (if using Unreal)
cd Unreal
.\build.bat
```

### Linux/WSL
```bash
# Clean previous builds
./clean_rebuild.sh

# Build AirSim
./build.sh

# Build Unreal Plugin (if using Unreal) 
cd Unreal
./build.sh
```

## What's Been Modified

The following files have been updated to implement weather physics:

### Core Weather System
- `AirLib/include/api/WeatherApi.hpp` - Enhanced weather parameter management with physics callbacks
- `AirLib/include/physics/Environment.hpp` - Air density modifications and weather state tracking
- `AirLib/src/api/RpcLibServerBase.cpp` - Weather physics callback registration

### Physics Integration  
- `AirLib/include/vehicles/multirotor/MultiRotorPhysicsBody.hpp` - Vehicle-specific weather forces
- `AirLib/include/physics/FastPhysicsEngine.hpp` - Enhanced drag calculations with weather effects

### Unreal Integration
- `Unreal/Plugins/AirSim/Source/Weather/WeatherLib.cpp` - UI to physics integration

## Key Features Added

1. **Enhanced Wind Generation**: Weather conditions automatically generate realistic wind patterns
2. **Progressive Physics Effects**: Weather intensity directly affects flight dynamics
3. **Multiple Weather Types**: Rain, snow, dust, and fog each have unique physics characteristics
4. **Turbulence System**: Random forces that shake the drone in severe weather
5. **Atmospheric Changes**: Weather modifies air density affecting drag and lift
6. **Downdraft Effects**: Heavy precipitation creates downward forces

## Testing

After building, test the weather physics by:

1. Launch AirSim with a multirotor
2. Take off and hover
3. Press F10 to open weather menu
4. Enable weather and adjust sliders
5. Observe physics effects on the drone

See `WEATHER_PHYSICS_TEST_GUIDE.md` for detailed testing instructions.

## Troubleshooting Build Issues

### Missing Headers
If you get missing header errors, ensure all AirLib headers are properly included.

### Linking Errors  
Clean and rebuild completely using the clean scripts.

### Runtime Errors
Check that all weather physics callbacks are properly registered in RpcLibServerBase.

## Performance Impact

The weather physics system adds:
- Minimal CPU overhead for weather calculations
- Real-time turbulence generation
- Enhanced drag computations
- Atmospheric density modifications

Effects are optimized and should not significantly impact frame rates.