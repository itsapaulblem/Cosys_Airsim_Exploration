// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_Environment_hpp
#define airsim_core_Environment_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "common/EarthUtils.hpp"
#include "common/GeodeticConverter.hpp"

namespace msr
{
namespace airlib
{

    class Environment : public UpdatableObject
    {
    public:
        struct State
        {
            //these fields must be set at initialization time
            Vector3r position;
            GeoPoint geo_point;

            //these fields are computed
            Vector3r gravity;
            real_T air_pressure;
            real_T temperature;
            real_T air_density;

            // Weather physics fields
            Vector3r wind_velocity = Vector3r::Zero();
            real_T wind_turbulence = 0.0f;
            real_T precipitation_rate = 0.0f;
            real_T dust_density = 0.0f;
            real_T fog_density = 0.0f;
            real_T falling_leaves_density = 0.0f;
            real_T visibility = 10000.0f;
            real_T humidity = 0.5f;

            State()
            {
            }
            State(const Vector3r& position_val, const GeoPoint& geo_point_val)
                : position(position_val), geo_point(geo_point_val)
            {
            }
        };

        void setWeatherConditions(real_T rain, real_T snow, real_T dust, real_T fog) {
            current_.precipitation_rate = std::max(rain, snow);
            current_.dust_density = dust;
            current_.fog_density = fog;

            real_T base_visibility = 10000.0f;
            base_visibility *= (1.0f - current_.precipitation_rate * 0.8f);
            base_visibility *= (1.0f - current_.dust_density * 0.9f);
            base_visibility *= (1.0f - current_.fog_density * 0.95f);
            current_.visibility = std::max(base_visibility, 50.0f);
        }

        Vector3r getWindVelocity() const
        {
            return current_.wind_velocity;
        }

        real_T getWindTurbulence() const
        {
            return current_.wind_turbulence;
        }

        real_T getPrecipitationRate() const
        {
            return current_.precipitation_rate;
        }

        real_T getDustDensity() const
        {
            return current_.dust_density;
        }

        real_T getFogDensity() const
        {
            return current_.fog_density;
        }

        real_T getVisibility() const
        {
            return current_.visibility;
        }

    public:
        Environment()
        {
            //allow default constructor with later call for initialize
        }
        Environment(const State& initial)
        {
            initialize(initial);
        }
        void initialize(const State& initial)
        {
            initial_ = initial;

            setHomeGeoPoint(initial_.geo_point);

            updateState(initial_);
        }

        void setHomeGeoPoint(const GeoPoint& home_geo_point)
        {
            home_geo_point_ = HomeGeoPoint(home_geo_point);
            geodetic_converter_.setHome(home_geo_point);
        }

        GeoPoint getHomeGeoPoint() const
        {
            return home_geo_point_.home_geo_point;
        }

        //in local NED coordinates
        void setPosition(const Vector3r& position)
        {
            current_.position = position;
        }

        const State& getInitialState() const
        {
            return initial_;
        }
        const State& getState() const
        {
            return current_;
        }
        State& getState()
        {
            return current_;
        }

        virtual void update(float delta = 0) override
        {
        	unused(delta);
            updateState(current_);
        }

        // Weather control methods
        void setRainIntensity(real_T intensity) {
            current_.precipitation_rate = std::max(current_.precipitation_rate, intensity);
        }

        void setSnowIntensity(real_T intensity) {
            current_.precipitation_rate = std::max(current_.precipitation_rate, intensity);
        }

        void setDustIntensity(real_T intensity) {
            current_.dust_density = intensity;
        }

        void setFogIntensity(real_T intensity) {
            current_.fog_density = intensity;
        }

        void setFallingLeavesIntensity(real_T intensity) {
            current_.falling_leaves_density = intensity;
        }

        void setWindVelocity(const Vector3r& wind_velocity) {
            current_.wind_velocity = wind_velocity;
        }

        void setTurbulenceIntensity(real_T intensity) {
            current_.wind_turbulence = intensity;
        }

    protected:
        virtual void resetImplementation() override
        {
            current_ = initial_;
        }

        virtual void failResetUpdateOrdering(std::string err) override
        {
            unused(err);
            //Do nothing.
            //The environment gets reset() twice without an update() inbetween,
            //via MultirotorPawnSimApi::reset() and CarSimApi::reset(), because
            //those functions directly reset an environment, and also call other reset()s that reset the same environment.
        }

    private:
        void updateState(State& state)
        {
            geodetic_converter_.ned2Geodetic(state.position, state.geo_point);

            real_T geo_pot = EarthUtils::getGeopotential(state.geo_point.altitude / 1000.0f);
            state.temperature = EarthUtils::getStandardTemperature(geo_pot);
            state.air_pressure = EarthUtils::getStandardPressure(geo_pot, state.temperature);
            state.air_density = EarthUtils::getAirDensity(state.air_pressure, state.temperature);

            if (state.humidity > 0) {
                real_T humidity_factor = 1.0f - (state.humidity * 0.025f);
                state.air_density *= humidity_factor;
            }

            if (state.dust_density > 0) {
                real_T dust_factor = 1.0f + (state.dust_density * 0.03f);
                state.air_density *= dust_factor;
            }

            if (state.precipitation_rate > 0) {
                real_T rain_factor = 1.0f + (state.precipitation_rate * 0.015f);
                state.air_density *= rain_factor;
            }
            
            //TODO: avoid recalculating square roots
            state.gravity = Vector3r(0, 0, EarthUtils::getGravity(state.geo_point.altitude));
        }

    private:
        State initial_, current_;
        HomeGeoPoint home_geo_point_;
        GeodeticConverter geodetic_converter_;
    };
}
} //namespace
#endif
