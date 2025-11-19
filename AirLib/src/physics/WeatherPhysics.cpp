#include "physics/WeatherPhysics.hpp"
#include "common/VectorMath.hpp"
#include <random>
#include <cmath>

using namespace msr::airlib;

WeatherPhysics::WeatherForces WeatherPhysics::calculateWeatherEffects(
	float rain_intensity,
	float snow_intensity,
	float dust_intensity,
	float fog_intensity,
	float falling_leaves_intensity,
	const Vector3r& drone_position,
	const Vector3r& drone_velocity,
	real_T delta_time)
{
	// Suppress unused parameter warnings
	(void)drone_position;
	(void)delta_time;
	
	WeatherForces forces;
	float total_intensity = rain_intensity + snow_intensity + dust_intensity + fog_intensity + falling_leaves_intensity;
		if (total_intensity <= 0.001f) {
			return forces;
	}

	forces.has_effects = true;

	Vector3r rain_turbulence = calculateRainTurbulence(rain_intensity, drone_velocity);
	Vector3r snow_turbulence = calculateSnowTurbulence(snow_intensity, drone_velocity);
	Vector3r dust_turbulence = calculateDustTurbulence(dust_intensity, drone_velocity);
	Vector3r fog_turbulence = calculateFogTurbulence(fog_intensity, drone_velocity);
	Vector3r leaves_turbulence = calculateLeavesTurbulence(falling_leaves_intensity, drone_velocity);

	forces.turbulence_force = rain_turbulence + snow_turbulence + dust_turbulence + fog_turbulence + leaves_turbulence;

	forces.wind_gust = generateRandomTurbulence(total_intensity, 3.0f);

	forces.drag_multiplier = calculateDragMultiplier(rain_intensity, snow_intensity, dust_intensity, fog_intensity);
	forces.air_density_multiplier = calculateAirDensityMultiplier(fog_intensity, dust_intensity, snow_intensity);

	return forces;
}

Vector3r WeatherPhysics::calculateRainTurbulence(float intensity, const Vector3r& velocity)
{
	// Suppress unused parameter warning
	(void)velocity;
	
	if (intensity <= 0.0f) {
		return Vector3r::Zero();
	}

	Vector3r turbulence = Vector3r::Zero();

	turbulence.z() = intensity * 2.5f;

	Vector3r random_lateral = generateRandomTurbulence(intensity, 1.5f);
	turbulence.x() = random_lateral.x();
	turbulence.y() = random_lateral.y();

	return turbulence; 
}

Vector3r WeatherPhysics::calculateSnowTurbulence(float intensity, const Vector3r& velocity)
{
	// Suppress unused parameter warning
	(void)velocity;
	
	if (intensity <= 0.0f) {
		return Vector3r::Zero();
	}

	Vector3r turbulence = Vector3r::Zero();

	turbulence.z() = intensity * 1.2f;

	Vector3r random_swirl = generateRandomTurbulence(intensity, 2.0f);
	turbulence.x() = random_swirl.x();
	turbulence.y() = random_swirl.y();
	turbulence.z() += random_swirl.z() * 0.3f;

	return turbulence;
}

Vector3r WeatherPhysics::calculateDustTurbulence(float intensity, const Vector3r& velocity)
{
	// Suppress unused parameter warning
	(void)velocity;
	
	if (intensity <= 0.0f) {
		return Vector3r::Zero();
	}

	Vector3r turbulence = Vector3r::Zero();

	Vector3r random_chaos = generateRandomTurbulence(intensity, 3.5f);
	turbulence = random_chaos;

	turbulence.z() -= intensity * 1.0f;
	return turbulence;
}

Vector3r WeatherPhysics::calculateFogTurbulence(float intensity, const Vector3r& velocity)
{
	// Suppress unused parameter warning
	(void)velocity;
	
	if (intensity <= 0.0f) {
		return Vector3r::Zero();
	}
	Vector3r turbulence = Vector3r::Zero();
	Vector3r random_damped = generateRandomTurbulence(intensity, 0.8f);
	turbulence = random_damped * 0.5f;

	return turbulence;
}

Vector3r WeatherPhysics::calculateLeavesTurbulence(float intensity, const Vector3r& velocity)
{
	// Suppress unused parameter warning
	(void)velocity;
	
	if (intensity <= 0.0f) {
		return Vector3r::Zero();
	}

	Vector3r turbulence = Vector3r::Zero();

	Vector3r random_swirl = generateRandomTurbulence(intensity, 1.0f);
	turbulence.x() = random_swirl.x() * 0.7f;
	turbulence.y() = random_swirl.y() * 0.7f;
	turbulence.z() = intensity * 0.8f;

	return turbulence;
}

real_T WeatherPhysics::calculateDragMultiplier(float rain, float snow, float dust, float fog)
{
	real_T multiplier = 1.0f;

	multiplier += rain * 0.3f;
	multiplier += snow * 0.2f;
	multiplier += dust * 0.5f;
	multiplier += fog * 0.1f;

	return multiplier;
}

real_T WeatherPhysics::calculateAirDensityMultiplier(float fog, float dust, float snow)
{
	real_T multiplier = 1.0f;

	multiplier += fog * 0.05f;
	multiplier += dust * 0.15f;
	multiplier += snow * 0.03f;

	return multiplier;
}


Vector3r WeatherPhysics::generateRandomTurbulence(float intensity, float scale)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<real_T> dist(0.0, 1.0);

	Vector3r turbulence(
		dist(gen) * intensity * scale,
		dist(gen) * intensity * scale,
		dist(gen) * intensity * scale * 0.6f
	);
	return turbulence;
}