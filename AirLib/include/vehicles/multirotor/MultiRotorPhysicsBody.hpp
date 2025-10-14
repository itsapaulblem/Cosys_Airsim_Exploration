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
            initialize(params->getParams().mass, params->getParams().inertia, kinematics, environment);
        }

        virtual void resetImplementation() override
        {
            PhysicsBody::resetImplementation();
            params_->getSensors().reset();
        }

        virtual void update(float delta = 0) override
        {
            PhysicsBody::update(delta);
        }

        virtual void reportState(StateReporter& reporter) override
        {
            PhysicsBody::reportState(reporter);
            params_->getSensors().reportState(reporter);
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                reporter.startHeading("", 1);
                reporter.writeValue("Rotor", rotor_index);
                reporter.endHeading(false, 1);
                rotors_.at(rotor_index).reportState(reporter);
            }
        }

        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            PhysicsBody::updateKinematics(kinematics);
            updateSensorsAndController();
        }

        virtual void updateKinematics() override
        {
            PhysicsBody::updateKinematics();
            updateSensorsAndController();
        }

        void updateSensorsAndController()
        {
            params_->getSensors().update();
            vehicle_api_->update();
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));
            }
        }

        const SensorCollection& getSensors() const
        {
            return params_->getSensors();
        }

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

    private:
        void initialize(real_T mass, const Matrix3x3r& inertia, Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(mass, inertia, kinematics, environment);
            createRotors(*params_, rotors_, environment);
            createDragVertices();
            params_->getSensors().initialize(&getKinematics(), &getEnvironment());
        }

        static void createRotors(const MultiRotorParams& params, vector<RotorActuator>& rotors, const Environment* environment)
        {
            rotors.clear();
            for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
                const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
                rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
            }
        }

        void createDragVertices()
        {
            const auto& params = params_->getParams();
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

            drag_faces_.clear();
            drag_faces_.emplace_back(Vector3r(0, 0, -params.body_box.z() / 2.0f), Vector3r(0, 0, -1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, 0, params.body_box.z() / 2.0f), Vector3r(0, 0, 1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, -params.body_box.y() / 2.0f, 0), Vector3r(0, -1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(0, params.body_box.y() / 2.0f, 0), Vector3r(0, 1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(-params.body_box.x() / 2.0f, 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
            drag_faces_.emplace_back(Vector3r(params.body_box.x() / 2.0f, 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());
        }

        MultiRotorParams* params_;
        vector<RotorActuator> rotors_;
        vector<PhysicsBodyVertex> drag_faces_;
        VehicleApiBase* vehicle_api_;
    };
}
} //namespace
#endif
