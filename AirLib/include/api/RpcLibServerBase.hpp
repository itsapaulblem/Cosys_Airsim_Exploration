// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"

namespace msr
{
namespace airlib
{

    class RpcLibServerBase : public ApiServerBase
    {
    public:
        RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port = RpcLibPort);
        virtual ~RpcLibServerBase() override;

        virtual void start(bool block, std::size_t thread_count) override;
        virtual void stop() override;

        // Wind physics API
        void setWind(const msr::airlib::Vector3r& wind);
        msr::airlib::Vector3r getWind() const;

        class ApiNotSupported : public std::runtime_error
        {
        public:
            ApiNotSupported(const std::string& message)
                : std::runtime_error(message)
            {
            }
        };

    protected:
        void* getServer() const;

        virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name);
        virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name);
        virtual WorldSimApiBase* getWorldSimApi();

        // Register weather wind callback with simSetWind API
        void registerWeatherWindCallback();

    private:
        ApiProvider* api_provider_;

        struct impl;
        std::unique_ptr<impl> pimpl_;
    };
}
} //namespace
#endif