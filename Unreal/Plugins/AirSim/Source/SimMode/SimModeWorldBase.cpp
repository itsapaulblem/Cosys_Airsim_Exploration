#include "SimModeWorldBase.h"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/ExternalPhysicsEngine.hpp"
#include <exception>
#include "AirBlueprintLib.h"
#include "api/WeatherApi.hpp"

void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();
}

void ASimModeWorldBase::initializeForPlay()
{
    std::vector<msr::airlib::UpdatableObject*> vehicles;
    for (auto& api : getApiProvider()->getVehicleSimApis())
        vehicles.push_back(api);
    //TODO: directly accept getVehicleSimApis() using generic container

    std::unique_ptr<PhysicsEngineBase> physics_engine = createPhysicsEngine();
    physics_engine_ = physics_engine.get();
    physics_world_.reset(new msr::airlib::PhysicsWorld(std::move(physics_engine),
                                                       vehicles,
                                                       getPhysicsLoopPeriod()));
}

void ASimModeWorldBase::registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody)
{
    // Reset the vehicle as well before registering it
    // Similar to what happens in initializeForPlay() above
    physicsBody->reset();
    physics_world_.get()->addBody(physicsBody);
}

void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //remove everything that we created in BeginPlay
    physics_world_.reset();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::startAsyncUpdator()
{
    physics_world_->startAsyncUpdator();
}

void ASimModeWorldBase::stopAsyncUpdator()
{
    physics_world_->stopAsyncUpdator();
}

long long ASimModeWorldBase::getPhysicsLoopPeriod() const //nanoseconds
{
    return physics_loop_period_;
}
void ASimModeWorldBase::setPhysicsLoopPeriod(long long period)
{
    physics_loop_period_ = period;
}

std::unique_ptr<ASimModeWorldBase::PhysicsEngineBase> ASimModeWorldBase::createPhysicsEngine()
{
    std::unique_ptr<PhysicsEngineBase> physics_engine;
    std::string physics_engine_name = getSettings().physics_engine_name;
    if (physics_engine_name == "")
        physics_engine.reset(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine") {
        msr::airlib::Settings fast_phys_settings;
        if (msr::airlib::Settings::singleton().getChild("FastPhysicsEngine", fast_phys_settings)) {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine(fast_phys_settings.getBool("EnableGroundLock", true)));
        }
        else {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine());
        }

        physics_engine->setWind(getSettings().wind);
        physics_engine->setExtForce(getSettings().ext_force);
    }
    else if (physics_engine_name == "ExternalPhysicsEngine") {
        physics_engine.reset(new msr::airlib::ExternalPhysicsEngine());
    }
    else {
        physics_engine.reset();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ", physics_engine_name, LogDebugLevel::Failure);
    }

    return physics_engine;
}

bool ASimModeWorldBase::isPaused() const
{
    return physics_world_->isPaused();
}

void ASimModeWorldBase::pause(bool is_paused)
{
    physics_world_->pause(is_paused);
    ASimModeBase::pause(is_paused);
}

void ASimModeWorldBase::continueForTime(double seconds)
{
    int64 start_frame_number = UKismetSystemLibrary::GetFrameCount();
    if (physics_world_->isPaused()) {
        physics_world_->pause(false);
        UGameplayStatics::SetGamePaused(this->GetWorld(), false);
    }

    physics_world_->continueForTime(seconds);
    while (!physics_world_->isPaused()) {
        continue;
    }
    // wait if no new frame is renderd
    while (start_frame_number == UKismetSystemLibrary::GetFrameCount()) {
        continue;
    }
    UGameplayStatics::SetGamePaused(this->GetWorld(), true);
}

void ASimModeWorldBase::continueForFrames(uint32_t frames)
{
    if (physics_world_->isPaused()) {
        physics_world_->pause(false);
        UGameplayStatics::SetGamePaused(this->GetWorld(), false);
    }

    physics_world_->setFrameNumber((uint32_t)GFrameNumber);
    physics_world_->continueForFrames(frames);
    while (!physics_world_->isPaused()) {
        physics_world_->setFrameNumber((uint32_t)GFrameNumber);
    }
    UGameplayStatics::SetGamePaused(this->GetWorld(), true);
}

void ASimModeWorldBase::setWind(const msr::airlib::Vector3r& wind) const
{
    physics_engine_->setWind(wind);
}

void ASimModeWorldBase::setExtForce(const msr::airlib::Vector3r& ext_force) const
{
    physics_engine_->setExtForce(ext_force);
}

void ASimModeWorldBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
    unused(debug_reporter);
    //we use custom debug reporting for this class
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    { //keep this lock as short as possible
        physics_world_->lock();

        physics_world_->enableStateReport(EnableReport);
        physics_world_->updateStateReport();

        for (auto& api : getApiProvider()->getVehicleSimApis())
            api->updateRenderedState(DeltaSeconds);

        physics_world_->unlock();
    }

    // Update weather physics every frame
    updateWeatherPhysics();

    //perform any expensive rendering update outside of lock region
    for (auto& api : getApiProvider()->getVehicleSimApis())
        api->updateRendering(DeltaSeconds);

    Super::Tick(DeltaSeconds);
// --- Physics-based weather sync ---
void ASimModeWorldBase::updateWeatherPhysics()
{
    // Get current weather parameters from UE5 weather system
    float rain = getWeatherParamScalar(EWeatherParamScalar::Rain);
    float snow = getWeatherParamScalar(EWeatherParamScalar::Snow);
    float fog = getWeatherParamScalar(EWeatherParamScalar::Fog);
    float dust = getWeatherParamScalar(EWeatherParamScalar::Dust);
    float wind_speed = getWeatherParamScalar(EWeatherParamScalar::WindSpeed);
    FVector wind_direction = getWeatherParamVector(EWeatherParamVector::WindDirection);

    msr::airlib::Vector3r wind_velocity(
        wind_direction.X * wind_speed,
        wind_direction.Y * wind_speed,
        wind_direction.Z * wind_speed
    );

    // Update environment for all vehicles
    auto api_provider = getApiProvider();
    if (api_provider) {
        auto vehicle_apis = api_provider->getVehicleSimApis();
        for (auto& pair : vehicle_apis) {
            auto* vehicle_sim_api = pair.second;
            if (vehicle_sim_api) {
                auto* physics_body = vehicle_sim_api->getPhysicsBody();
                if (physics_body && physics_body->hasEnvironment()) {
                    auto& environment = physics_body->getEnvironment();
                    environment.setWindVelocity(wind_velocity);
                    environment.setRainIntensity(rain);
                    environment.setSnowIntensity(snow);
                    environment.setFogIntensity(fog);
                    environment.setDustIntensity(dust);
                    environment.setTurbulenceIntensity(wind_speed); // Optionally scale turbulence with wind

                    msr::airlib::Vector3r wind_force = wind_velocity * physics_body->getMass();

                    msr::airlib::Vector3r rain_force(
                        FMath::FRandRange(-1.0f, 1.0f) * rain * 8.0f * physics_body->getMass(),
                        FMath::FRandRange(-1.0f, 1.0f) * rain * 8.0f * physics_body->getMass(),
                        0.0f
                    );

                    msr::airlib::Vector3r snow_force(
                        FMath::FRandRange(-0.5f, 0.5f) * snow * 5.0f * physics_body->getMass(),
                        FMath::FRandRange(-0.5f, 0.5f) * snow * 5.0f * physics_body->getMass(),
                        FMath::FRandRange(-0.2f, 0.2f) * snow * 3.0f * physics_body->getMass()
                    );

                    msr::airlib::Vector3r dust_force(
                        FMath::FRandRange(-0.3f, 0.3f) * dust * 4.0f * physics_body->getMass(),
                        FMath::FRandRange(-0.3f, 0.3f) * dust * 4.0f * physics_body->getMass(),
                        0.0f
                    );

                    float drag_factor = 1.0f - (rain * 0.15f + snow * 0.25f + dust * 0.1f);
                    drag_factor = FMath::Clamp(drag_factor, 0.5f, 1.0f);
                    
                    msr::airlib::Vector3r total_force = wind_force + rain_force + snow_force + dust_force;

                    physics_body->setExternalForce(total_force);
                    physics_body->setDragFactor(drag_factor);

                }
            }
        }
    }
}
// --- End physics-based weather sync ---
}

void ASimModeWorldBase::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        physics_world_->reset();
    },
    true);

    //no need to call base reset because of our custom implementation
}

std::string ASimModeWorldBase::getDebugReport()
{
    return physics_world_->getDebugReport();
}

