#include "WeatherPhysicsBridge.h"
#include "Engine/World.h"
#include "Weather/WeatherLib.h"
#include <random>

FWeatherForces UWeatherPhysicsBridge::GetWeatherForces(
	UWorld* World,
	const FVector& Position,
	const FVector& Velocity,
	float DeltaTime)
{
	FWeatherForces forces;
	
	if (!World || !IsWeatherEnabled(World)) {
		return forces;
	}

	// Get weather intensities from F10 menu sliders
	float rain_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_RAIN);
	float snow_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_SNOW);
	float dust_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_DUST);
	float fog_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_FOG);
	float falling_leaves_intensity = GetWeatherSliderValue(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_MAPLELEAF);

	float total_intensity = rain_intensity + snow_intensity + dust_intensity + fog_intensity + falling_leaves_intensity;
	
	if (total_intensity <= 0.001f) {
		return forces; // No weather effects
	}

	forces.HasEffects = true;

	// Calculate weather effects directly in Unreal
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<float> dist(0.0f, 1.0f);

	// Rain creates downward force and turbulence
	if (rain_intensity > 0.0f) {
		forces.TurbulenceForce.Z += rain_intensity * 250.0f; // Downward (cm units)
		forces.TurbulenceForce.X += dist(gen) * rain_intensity * 150.0f;
		forces.TurbulenceForce.Y += dist(gen) * rain_intensity * 150.0f;
	}

	// Snow creates swirling effects
	if (snow_intensity > 0.0f) {
		forces.TurbulenceForce.Z += snow_intensity * 120.0f;
		forces.TurbulenceForce.X += dist(gen) * snow_intensity * 200.0f;
		forces.TurbulenceForce.Y += dist(gen) * snow_intensity * 200.0f;
	}

	// Dust creates strong chaotic turbulence
	if (dust_intensity > 0.0f) {
		forces.TurbulenceForce.X += dist(gen) * dust_intensity * 350.0f;
		forces.TurbulenceForce.Y += dist(gen) * dust_intensity * 350.0f;
		forces.TurbulenceForce.Z -= dust_intensity * 100.0f; // Updraft
	}

	// Fog creates subtle effects
	if (fog_intensity > 0.0f) {
		forces.TurbulenceForce += FVector(dist(gen), dist(gen), dist(gen)) * fog_intensity * 80.0f * 0.5f;
	}

	// Falling leaves create gentle effects
	if (falling_leaves_intensity > 0.0f) {
		forces.TurbulenceForce.X += dist(gen) * falling_leaves_intensity * 100.0f * 0.7f;
		forces.TurbulenceForce.Y += dist(gen) * falling_leaves_intensity * 100.0f * 0.7f;
		forces.TurbulenceForce.Z += falling_leaves_intensity * 80.0f;
	}

	// Generate random wind gusts
	forces.WindGust = FVector(
		dist(gen) * total_intensity * 300.0f,
		dist(gen) * total_intensity * 300.0f,
		dist(gen) * total_intensity * 300.0f * 0.6f
	);

	// Calculate drag and air density multipliers
	forces.DragMultiplier = 1.0f + (rain_intensity * 0.3f) + (snow_intensity * 0.2f) + (dust_intensity * 0.5f) + (fog_intensity * 0.1f);
	forces.AirDensityMultiplier = 1.0f + (fog_intensity * 0.05f) + (dust_intensity * 0.15f) + (snow_intensity * 0.03f);

	return forces;
}

float UWeatherPhysicsBridge::GetWeatherSliderValue(UWorld* World, EWeatherParamScalar Parameter)
{
	if (!World) {
		return 0.0f;
	}

	return UWeatherLib::getWeatherParamScalar(World, Parameter);
}

bool UWeatherPhysicsBridge::IsWeatherEnabled(UWorld* World)
{
	if (!World) {
		return false;
	}
	
	return UWeatherLib::getIsWeatherEnabled(World);
}