#ifndef msr_airlib_WeatherApi_hpp
#define msr_airlib_WeatherApi_hpp

#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include <unordered_map>

namespace msr 
{
namespace airlib
{
    class WeatherApi
    {
        public:
            struct WeatherParams
            {
                real_T rain_intensity = 0.0f;
                real_T snow_intensity = 0.0f;
                real_T dust_intensity = 0.0f;
                real_T fog_intensity = 0.0f;
                Vector3r wind_velocity = Vector3r::Zero();
                real_T turbulence_intensity = 0.0f;
            };

            static WeatherApi& getInstance()
            {
                static WeatherApi instance;
                return instance;
            }

            void registerEnvironment(const std::string& vehicle_name, Environment* environment)
            {
                environments_[vehicle_name] = environment;
            }

            void setWeatherFromUI(real_T rain, real_T snow, real_T dust, real_T fog) {
                current_weather_.rain_intensity = std::clamp(rain, 0.0f, 1.0f); 
                current_weather_.snow_intensity = std::clamp(snow, 0.0f, 1.0f);
                current_weather_.dust_intensity = std::clamp(dust, 0.0f, 1.0f);
                current_weather_.fog_intensity = std::clamp(fog, 0.0f, 1.0f);

                generateWeatherWind();
                updateAllEnvironments();
            }

            void setWind(const Vector3r& velocity, real_T turbulence = 0.0f) {
                current_weather_.wind_velocity = velocity;
                current_weather_.turbulence_intensity = std::clamp(turbulence, 0.0f, 1.0f);
                updateAllEnvironments();
            }

            const WeatherParams& getCurrentWeather() const {
                return current_weather_;
            }
        
        private:
            void generateWeatherWind() {
                real_T max_intensity = std::max({
                    current_weather_.rain_intensity,
                    current_weather_.snow_intensity,
                    current_weather_.dust_intensity,
                    current_weather_.fog_intensity
                });

                if (max_intensity > 0.3f) {
                    real_T wind_speed = 5.0f + max_intensity * 15.0f;
                    real_T wind_direction = static_cast<real_T>(rand()) / RAND_MAX * 2.0f * M_PI;
                    
                    current_weather_.wind_velocity = Vector3r(
                        wind_speed * cos(wind_direction),
                        wind_speed * sin(wind_direction),
                        wind_speed * 0.2f
                    );

                    current_weather_.turbulence_intensity = max_intensity * 0.8f;
                }
            }
        void updateAllEnvironments() {
            for (auto& [vehicle_name, env] : environments_) {
                if (env) {
                    env->setWindVelocity(current_weather_.wind_velocity);
                    env->setWeatherConditions(
                        current_weather_.rain_intensity,
                        current_weather_.snow_intensity,
                        current_weather_.dust_intensity,
                        current_weather_.fog_intensity
                    );
                    env->setTurbulenceIntensity(current_weather_.turbulence_intensity);
                }
            }
        }
        WeatherParams current_weather_;
        std::unordered_map<std::string, Environment*> environments_;
    };
}
}
#endif