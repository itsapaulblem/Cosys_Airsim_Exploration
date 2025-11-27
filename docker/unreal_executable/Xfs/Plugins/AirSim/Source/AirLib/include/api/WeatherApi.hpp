#ifndef msr_airlib_WeatherApi_hpp
#define msr_airlib_WeatherApi_hpp

#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

            // Function pointer types for physics callbacks
            using WindCallback = std::function<void(const Vector3r& wind)>;
            using WeatherPhysicsCallback = std::function<void(const WeatherParams& weather)>;

            static WeatherApi& getInstance()
            {
                static WeatherApi instance;
                return instance;
            }

            void registerEnvironment(const std::string& vehicle_name, Environment* environment)
            {
                environments_[vehicle_name] = environment;
            }

            // Register physics callbacks
            void registerWindCallback(WindCallback callback)
            {
                wind_callback_ = callback;
            }

            void registerWeatherPhysicsCallback(WeatherPhysicsCallback callback)
            {
                weather_physics_callback_ = callback;
            }

        void setWeatherFromUI(real_T rain, real_T snow, real_T dust, real_T fog) {
            current_weather_.rain_intensity = std::clamp(rain, 0.0f, 1.0f); 
            current_weather_.snow_intensity = std::clamp(snow, 0.0f, 1.0f);
            current_weather_.dust_intensity = std::clamp(dust, 0.0f, 1.0f);
            current_weather_.fog_intensity = std::clamp(fog, 0.0f, 1.0f);

            generateWeatherWind();
            applyWeatherPhysics();
        }            void setWind(const Vector3r& velocity, real_T turbulence = 0.0f) {
                current_weather_.wind_velocity = velocity;
                current_weather_.turbulence_intensity = std::clamp(turbulence, 0.0f, 1.0f);
                applyWeatherPhysics();
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

                if (max_intensity > 0.1f) {  // Start effects at lower intensity
                    real_T wind_speed = 15.0f + max_intensity * 45.0f; // Much stronger wind
                    real_T wind_direction = static_cast<real_T>(rand()) / static_cast<real_T>(RAND_MAX) * 2.0f * M_PI;
                    
                    current_weather_.wind_velocity = Vector3r(
                        wind_speed * cos(wind_direction),
                        wind_speed * sin(wind_direction),
                        wind_speed * 0.6f // Much stronger vertical component
                    );

                    current_weather_.turbulence_intensity = max_intensity * 2.5f; // Much stronger turbulence
                } else {
                    current_weather_.wind_velocity = Vector3r::Zero();
                    current_weather_.turbulence_intensity = 0.0f;
                }
            }

            // Apply both wind and weather physics
            void applyWeatherPhysics() {
                if (wind_callback_) {
                    wind_callback_(current_weather_.wind_velocity);
                }
                
                if (weather_physics_callback_) {
                    weather_physics_callback_(current_weather_);
                }
            }



        WeatherParams current_weather_;
        std::unordered_map<std::string, Environment*> environments_;
        WindCallback wind_callback_;
        WeatherPhysicsCallback weather_physics_callback_;
    };
}
}
#endif