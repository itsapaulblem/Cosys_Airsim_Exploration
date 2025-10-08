// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_multirotorphysicsbody_hpp
#define msr_airlib_multirotorphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "RotorActuator.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"

namespace msr
{
namespace airlib
{

    class MultiRotorPhysicsBody : public PhysicsBody
    {
    public:
        MultiRotorPhysicsBody(MultiRotorParams* params, VehicleApiBase* vehicle_api,
                              Kinematics* kinematics, Environment* environment)
            : params_(params), vehicle_api_(vehicle_api)
        {
            setName("MultiRotorPhysicsBody");
            vehicle_api_->setParent(this);
            initialize(kinematics, environment);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            //reset rotors, kinematics and environment
            PhysicsBody::resetImplementation();

            //reset sensors last after their ground truth has been reset
            resetSensors();
        }

        virtual void update(float delta = 0) override
        {

            applyWeatherEffects(delta);
            //update forces on vertices that we will use next
            PhysicsBody::update(delta);

            //Note that controller gets updated after kinematics gets updated in updateKinematics
            //otherwise sensors will have values from previous cycle causing lags which will appear
            //as crazy jerks whenever commands like velocity is issued
        }
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            PhysicsBody::reportState(reporter);

            reportSensors(*params_, reporter);

            //report rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                reporter.startHeading("", 1);
                reporter.writeValue("Rotor", rotor_index);
                reporter.endHeading(false, 1);
                rotors_.at(rotor_index).reportState(reporter);
            }
        }
        //*** End: UpdatableState implementation ***//

        //Fast Physics engine calls this method to set next kinematics
        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            PhysicsBody::updateKinematics(kinematics);

            updateSensorsAndController();
        }

        //External Physics engine calls this method to keep physics bodies updated and move rotors
        virtual void updateKinematics() override
        {
            PhysicsBody::updateKinematics();

            updateSensorsAndController();
        }

        void updateSensorsAndController()
        {
            updateSensors(*params_, getKinematics(), getEnvironment());

            //update controller which will update actuator control signal
            vehicle_api_->update();

            //transfer new input values from controller to rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));
            }
        }

        //sensor getter
        const SensorCollection& getSensors() const
        {
            return params_->getSensors();
        }

        //physics body interface
        virtual uint wrenchVertexCount() const override
        {
            return params_->getParams().rotor_count;
        }
        virtual PhysicsBodyVertex& getWrenchVertex(uint index) override
        {
            return rotors_.at(index);
        }
        virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
        {
            return rotors_.at(index);
        }

        virtual uint dragVertexCount() const override
        {
            return static_cast<uint>(drag_faces_.size());
        }
        virtual PhysicsBodyVertex& getDragVertex(uint index) override
        {
            return drag_faces_.at(index);
        }
        virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
        {
            return drag_faces_.at(index);
        }

        virtual real_T getRestitution() const override
        {
            return params_->getParams().restitution;
        }
        virtual real_T getFriction() const override
        {
            return params_->getParams().friction;
        }

        RotorActuator::Output getRotorOutput(uint rotor_index) const
        {
            return rotors_.at(rotor_index).getOutput();
        }

        virtual ~MultiRotorPhysicsBody() = default;

        // Getter for external weather force (needed by FastPhysicsEngine)
        Vector3r getExternalWeatherForce() const
        {
            return external_weather_force_;
        }

    private: //methods
        void applyWeatherEffects(float delta)
        {
            const Environment::State& env_state = getEnvironment().getState();

            // Calculate specific weather effects based on your requirements
            Vector3r rain_force = calculateRainEffects(env_state, delta);
            Vector3r snow_force = calculateSnowEffects(env_state, delta);
            Vector3r dust_force = calculateDustEffects(env_state, delta);
            Vector3r fog_force = calculateFogEffects(env_state, delta);
            Vector3r leaves_force = calculateLeavesEffects(env_state, delta);

            external_weather_force_ = rain_force + snow_force + dust_force + fog_force + leaves_force;
        }

        Vector3r calculateRainEffects(const Environment::State& env_state, float delta)
        {
            real_T rain = env_state.precipitation_rate;
            if (rain <= 0) return Vector3r::Zero();

            // Rain makes drone sway left and right
            static float rain_time = 0.0f;
            rain_time += delta;
            
            real_T sway_force = rain * 8.0f * sin(rain_time * 3.0f); // Oscillating left-right force
            Vector3r rain_force = Vector3r(0, sway_force, 0);
            
            return rain_force;
        }

        Vector3r calculateSnowEffects(const Environment::State& env_state, float delta)
        {
            real_T snow = env_state.precipitation_rate; // Assuming snow uses precipitation_rate
            if (snow <= 0) return Vector3r::Zero();

            // Snow makes drone more shaky left-right but more intense than rain
            static float snow_time = 0.0f;
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_real_distribution<real_T> shake_dist(-1.0f, 1.0f);
            
            snow_time += delta;
            
            real_T shake_intensity = snow * 15.0f; // More intense than rain
            real_T left_right_shake = shake_dist(gen) * shake_intensity;
            real_T additional_sway = snow * 6.0f * sin(snow_time * 4.5f); // Faster oscillation than rain
            
            Vector3r snow_force = Vector3r(0, left_right_shake + additional_sway, 0);
            
            return snow_force;
        }

        Vector3r calculateDustEffects(const Environment::State& env_state, float delta)
        {
            real_T dust = env_state.dust_density;
            if (dust <= 0) return Vector3r::Zero();

            Vector3r current_velocity = getKinematics().twist.linear;
            
            // Dust slows down the drone and pushes it backwards
            real_T slowdown_factor = dust * 12.0f; // Strong slowdown
            real_T backward_push = dust * 10.0f;   // Constant backward force
            
            Vector3r dust_force = Vector3r::Zero();
            
            // Apply drag to slow down forward movement
            if (current_velocity.x() > 0) { // Moving forward
                dust_force.x() -= slowdown_factor;
            }
            
            // Apply backward push
            dust_force.x() -= backward_push;
            
            return dust_force;
        }

        Vector3r calculateFogEffects(const Environment::State& env_state, float delta)
        {
            real_T fog = env_state.fog_density;
            if (fog <= 0) return Vector3r::Zero();

            Vector3r current_velocity = getKinematics().twist.linear;
            
            // Fog further slows down the drone (affects all directions)
            real_T fog_drag = fog * 8.0f;
            Vector3r fog_force = -current_velocity * fog_drag;
            
            return fog_force;
        }

        Vector3r calculateLeavesEffects(const Environment::State& env_state, float delta)
        {
            real_T leaves = env_state.falling_leaves_density; // You'll need to add this to Environment
            if (leaves <= 0) return Vector3r::Zero();

            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_real_distribution<real_T> buffet_dist(-1.0f, 1.0f);
            
            // Falling leaves create random buffeting in all directions
            real_T buffet_strength = leaves * 5.0f;
            Vector3r leaves_force = Vector3r(
                buffet_dist(gen) * buffet_strength,
                buffet_dist(gen) * buffet_strength,
                buffet_dist(gen) * buffet_strength * 0.5f // Less vertical effect
            );
            
            return leaves_force;
        }

        Vector3r calculateWindForces(const Environment::State& env_state)
        {
            Vector3r wind_vel = env_state.wind_velocity;
            Vector3r drone_vel = getKinematics().twist.linear;
            Vector3r relative_wind = wind_vel - drone_vel;

            real_T air_density = env_state.air_density;
            Vector3r wind_force = Vector3r::Zero();

            for (size_t i = 0; i < drag_faces_.size(); ++i) {
                const auto& drag_face = drag_faces_[i];
                Vector3r face_normal = drag_face.getNormal();

                real_T wind_component = relative_wind.dot(face_normal);
                if (wind_component > 0) {
                    real_T drag_coefficient = 1.2f;
                    real_T drag_area = drag_face.getDragFactor() * 0.1f;

                    // F = 0.5 * ρ * A * Cd * v^2
                    Vector3r face_force = 0.5f * air_density * drag_coefficient * drag_area * wind_component * face_normal;
                    wind_force += face_force;
                }
            }
            return wind_force;
        }

        Vector3r calculateTurbulenceForces(const Environment::State& env_state, float delta)
        {
            real_T turbulence = env_state.wind_turbulence;
            if (turbulence <= 0){
                return Vector3r::Zero();
            }
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::normal_distribution<real_T> dist(0.0f, 1.0f);

            real_T mass = params_->getParams().mass;
            real_T air_density = env_state.air_density;
            real_T turbulence_scale = turbulence * air_density * mass * 0.5f * delta;

            Vector3r turbulence_force = Vector3r(
                dist(gen) * turbulence_scale,
                dist(gen) * turbulence_scale,
                dist(gen) * turbulence_scale * 0.3f
            );
            return turbulence_force;
        }

        Vector3r calculatePrecipitationDrag(const Environment::State& env_state)
        {
            real_T precip = env_state.precipitation_rate;
            if (precip <= 0) {
                return Vector3r::Zero();
            }
            Vector3r drone_vel = getKinematics().twist.linear;
            real_T speed = drone_vel.norm();

            if (speed < 0.1f) {
                return Vector3r::Zero();
            }

            real_T precip_drag_coeff = precip * 0.02f;
            real_T air_density = env_state.air_density;
            real_T frontal_area = 0.5f;

            real_T drag_magnitude = precip_drag_coeff * air_density * frontal_area * speed * speed;
            Vector3r drag_direction = -drone_vel.normalized();
            return drag_direction * drag_magnitude;
        }

        Vector3r calculateDustStormEffects(const Environment::State& env_state)
        {
            real_T dust = env_state.dust_density;
            if (dust <= 0) {
                return Vector3r::Zero();
            }

            Vector3r drone_vel = getKinematics().twist.linear;
            real_T speed = drone_vel.norm();

            Vector3r dust_force = Vector3r::Zero();
            if (speed > 0.1f) {
                real_T dust_drag_coeff = dust * 0.03f;
                real_T air_density = env_state.air_density;
                real_T frontal_area = 0.5f;

                real_T drag_magnitude = dust_drag_coeff * air_density * frontal_area * speed * speed;
                Vector3r drag_direction = -drone_vel.normalized();
                dust_force = drag_direction * drag_magnitude;
            }

            static std::random_device rd; 
            static std::mt19937 gen(rd());
            static std::normal_distribution<real_T> dist(0.0f, 1.0f);

            real_T buffet_scale = dust * 2.0f;
            Vector3r buffet_force = Vector3r(
                dist(gen) * buffet_scale,
                dist(gen) * buffet_scale,
                dist(gen) * buffet_scale * 0.5f
            );
            dust_force += buffet_force;
            return dust_force;
        }

        real_T calculateWeatherRotorEfficiency(const Environment::State& env_state)
        {
            real_T efficiency = 1.0f;

            real_T precip = env_state.precipitation_rate;
            if (precip > 0) {
                efficiency *= (1.0f - precip * 0.15f);
            }

            real_T dust = env_state.dust_density;
            if (dust > 0) {
                efficiency *= (1.0f - dust * 0.25f);
            }

            real_T fog = env_state.fog_density;
            if (fog > 0) {
                efficiency *= (1.0f - fog * 0.05f);
            }
            return std::max(efficiency, 0.5f);
        }

        void initialize(Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

            createRotors(*params_, rotors_, environment);
            createDragVertices();

            initSensors(*params_, getKinematics(), getEnvironment());
        }

        static void createRotors(const MultiRotorParams& params, vector<RotorActuator>& rotors, const Environment* environment)
        {
            rotors.clear();
            //for each rotor pose
            for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
                const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
                rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
            }
        }

        void reportSensors(MultiRotorParams& params, StateReporter& reporter)
        {
            params.getSensors().reportState(reporter);
        }

        void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            unused(state);
            unused(environment);
            params.getSensors().update();
        }

        void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            params.getSensors().initialize(&state, &environment);
        }

        void resetSensors()
        {
            params_->getSensors().reset();
        }

        void createDragVertices()
        {
            const auto& params = params_->getParams();

            //Drone is seen as central body that is connected to propellers via arm. We approximate central body as box of size x, y, z.
            //The drag depends on area exposed so we also add area of propellers to approximate drag they may introduce due to their area.
            //while moving along any axis, we find area that will be exposed in that direction
            real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
            real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;

            real_T top_bottom_area = params.body_box.x() * params.body_box.y();
            real_T left_right_area = params.body_box.x() * params.body_box.z();
            real_T front_back_area = params.body_box.y() * params.body_box.z();
            Vector3r drag_factor_unit = Vector3r(
                                            front_back_area + rotors_.size() * propeller_xsection,
                                            left_right_area + rotors_.size() * propeller_xsection,
                                            top_bottom_area + rotors_.size() * propeller_area) *
                                        params.linear_drag_coefficient / 2;

            //add six drag vertices representing 6 sides
            drag_faces_.clear();
            drag_faces_.emplace_back(Vector3r(0, 0, -params.body_box.z() / 2.0f), Vector3r(0, 0, -1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, 0, params.body_box.z() / 2.0f), Vector3r(0, 0, 1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, -params.body_box.y() / 2.0f, 0), Vector3r(0, -1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(0, params.body_box.y() / 2.0f, 0), Vector3r(0, 1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(-params.body_box.x() / 2.0f, 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
            drag_faces_.emplace_back(Vector3r(params.body_box.x() / 2.0f, 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());
        }

    private: //fields
        Vector3r external_weather_force_;
        MultiRotorParams* params_;

        //let us be the owner of rotors object
        vector<RotorActuator> rotors_;
        vector<PhysicsBodyVertex> drag_faces_;

        std::unique_ptr<Environment> environment_;
        VehicleApiBase* vehicle_api_;
    };
}
} //namespace
#endif
