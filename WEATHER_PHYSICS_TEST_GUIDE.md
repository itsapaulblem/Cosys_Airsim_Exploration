# Weather Physics Test Guide

## Testing Weather Physics Effects in AirSim

This guide will help you test the enhanced weather physics effects that have been implemented.

### Prerequisites
1. Build AirSim with the updated code
2. Launch AirSim with a multirotor vehicle
3. Make sure the vehicle is airborne

### Manual Testing Steps

#### 1. Basic Weather Effects Test
1. **Launch AirSim** and spawn a drone
2. **Take off** and hover at a stable altitude (10-20 meters)
3. **Press F10** to open the weather menu
4. **Enable Weather** by checking the weather enabled checkbox
5. **Test individual weather conditions:**

   **Rain Test:**
   - Set Rain slider to 0.5 (50%)
   - Keep other sliders at 0
   - Observe: Drone should experience increased drag and slight turbulence
   - Expected: Harder to maintain stable hover, slight shaking

   **Snow Test:**
   - Set Rain to 0, Snow to 0.7 (70%)
   - Observe: Drone should experience heavy drag and turbulence
   - Expected: Significant difficulty maintaining position, strong wind effects

   **Dust Storm Test:**
   - Set all to 0, Dust to 0.8 (80%)
   - Observe: Moderate drag increase with turbulence
   - Expected: Reduced stability, moderate shaking

   **Fog Test:**
   - Set all to 0, Fog to 0.9 (90%)
   - Observe: Atmospheric drag effects
   - Expected: Slight resistance to movement

#### 2. Severe Weather Test
1. **Set multiple weather conditions simultaneously:**
   - Rain: 0.6
   - Snow: 0.4
   - Dust: 0.3
   - Fog: 0.5
2. **Observe combined effects:**
   - Drone should be very difficult to control
   - Strong turbulence and wind forces
   - Random shaking and movement
   - Downward forces in heavy precipitation

#### 3. Progressive Weather Test
1. **Start with clear weather** (all sliders at 0)
2. **Gradually increase rain** from 0 to 1.0 in 0.2 increments
3. **Observe how effects intensify:**
   - 0.0-0.3: Minimal effects
   - 0.3-0.6: Noticeable drag and wind
   - 0.6-0.8: Strong turbulence
   - 0.8-1.0: Severe weather effects with downdrafts

### Expected Physics Effects

#### Rain (0.0 - 1.0)
- **Drag increase:** Up to 40% additional drag
- **Wind generation:** Automatic wind based on intensity
- **Turbulence:** Proportional to rain intensity
- **Downdrafts:** At 70%+ intensity

#### Snow (0.0 - 1.0)
- **Drag increase:** Up to 60% additional drag (highest)
- **Strong turbulence:** More than rain
- **Heavy downdrafts:** At 60%+ intensity
- **Air density changes:** Affects overall flight dynamics

#### Dust (0.0 - 1.0)
- **Moderate drag:** Up to 25% increase
- **Atmospheric interference:** Affects air density
- **Medium turbulence:** Less than rain/snow

#### Fog (0.0 - 1.0)
- **Light drag:** Up to 30% increase
- **Air density changes:** Affects lift characteristics
- **Mild turbulence:** Subtle effects

### Troubleshooting

**If no weather effects are observed:**
1. Ensure weather is enabled (checkbox checked)
2. Verify weather sliders are above 0.3 for noticeable effects
3. Check that the drone is airborne (ground effects differ)
4. Try more extreme values (0.8-1.0) first

**If effects are too strong:**
- Adjust the multipliers in the code:
  - `MultiRotorPhysicsBody.hpp`: Reduce drag multiplier values
  - `WeatherApi.hpp`: Reduce wind speed and turbulence scales

**If effects are too weak:**
- Increase the multipliers in the same files
- Ensure air density modifications are applied in `Environment.hpp`

### Performance Notes
- Weather physics calculations add computational overhead
- Effects are most noticeable when the drone is actively trying to maintain position
- Wind and turbulence effects are random and will vary each frame
- Heavy weather conditions (>0.8) are designed to be challenging to control

### Code Verification
The weather physics effects are implemented in:
- `WeatherApi.hpp`: Weather parameter management and wind generation
- `MultiRotorPhysicsBody.hpp`: Vehicle-specific weather force application
- `FastPhysicsEngine.hpp`: Enhanced drag calculations with weather
- `Environment.hpp`: Air density modifications
- `WeatherLib.cpp`: UI integration with physics

### Success Criteria
✅ **Rain at 0.8:** Drone shakes noticeably, harder to control
✅ **Snow at 0.9:** Strong downward forces, significant turbulence  
✅ **Dust at 0.7:** Moderate resistance and shaking
✅ **Combined weather:** Very difficult to maintain stable flight
✅ **Progressive increase:** Effects gradually intensify as sliders increase