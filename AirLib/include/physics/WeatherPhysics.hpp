#ifndef airsim_core_WeatherPhysics_hpp
#define airsim_core_WeatherPhysics_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

namespace msr
{
	namespace airlib
	{
		class WeatherPhysics
		{
		public:
			struct WeatherForces
			{
				Vector3r turbulence_force = Vector3r::Zero();
				Vector3r wind_gust = Vector3r::Zero();
				real_T drag_multiplier = 1.0f;
				real_T air_density_multiplier = 1.0f;
				bool has_effects = false;
			};

			static WeatherForces calculateWeatherEffects(
				float rain_intensity,
				float snow_intensity,
				float dust_intensity,
				float fog_intensity,
				float falling_leaves_intensity,
				const Vector3r& drone_position,
				const Vector3r& drone_velocity,
				real_T delta_time
			);

		private:
			static Vector3r calculateRainTurbulence(float intensity, const Vector3r& velocity);
			static Vector3r calculateSnowTurbulence(float intensity, const Vector3r& velocity);
			static Vector3r calculateDustTurbulence(float intensity, const Vector3r& velocity);
			static Vector3r calculateFogTurbulence(float intensity, const Vector3r& velocity);
			static Vector3r calculateLeavesTurbulence(float intensity, const Vector3r& velocity);

			static real_T calculateDragMultiplier(float rain, float snow, float dust, float fog);
			static real_T calculateAirDensityMultiplier(float fog, float dust, float snow);

			static Vector3r generateRandomTurbulence(float intensity, float scale = 1.0f);
		};
	}
}
#endif
