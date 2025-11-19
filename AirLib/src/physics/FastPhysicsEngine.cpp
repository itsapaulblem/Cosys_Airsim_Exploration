#include "physics/FastPhysicsEngine.hpp"

using namespace msr::airlib;

// Define the static member
WeatherPhysics::WeatherForces FastPhysicsEngine::current_weather_forces_ = WeatherPhysics::WeatherForces();