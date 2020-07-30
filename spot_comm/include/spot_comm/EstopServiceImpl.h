#ifndef ESTOP_SERVICE_IMPL_H
#define ESTOP_SERVICE_IMPL_H

#include "bosdyn/api/estop_service.grpc.pb.h"

using bosdyn::api::RegisterEstopEndpointRequest;
using bosdyn::api::RegisterEstopEndpointResponse;
using bosdyn::api::DeregisterEstopEndpointRequest;
using bosdyn::api::DeregisterEstopEndpointResponse;
using bosdyn::api::EstopCheckInRequest;
using bosdyn::api::EstopCheckInResponse;
using bosdyn::api::GetEstopConfigRequest;
using bosdyn::api::GetEstopConfigResponse;
using bosdyn::api::SetEstopConfigRequest;
using bosdyn::api::SetEstopConfigResponse;
using bosdyn::api::GetEstopSystemStatusRequest;
using bosdyn::api::GetEstopSystemStatusResponse;
using bosdyn::api::EstopService;
using bosdyn::api::EstopEndpoint;
using bosdyn::api::EstopConfig;
using bosdyn::api::EstopSystemStatus;
using bosdyn::api::EstopStopLevel;
using bosdyn::api::EstopEndpointWithStatus;
using google::protobuf::Duration;
using grpc::Status;
using grpc::ServerContext;

class EstopServiceImpl final : public EstopService::Service {
public:
  Status RegisterEstopEndpoint(ServerContext* context, const RegisterEstopEndpointRequest* request, RegisterEstopEndpointResponse* response) override;
  Status DeregisterEstopEndpoint(ServerContext* context, const DeregisterEstopEndpointRequest* request, DeregisterEstopEndpointResponse* response) override;
  Status EstopCheckIn(ServerContext* context, const EstopCheckInRequest* request, EstopCheckInResponse* response) override;
  Status GetEstopConfig(ServerContext* context, const GetEstopConfigRequest* request, GetEstopConfigResponse* response) override;
  Status SetEstopConfig(ServerContext* context, const SetEstopConfigRequest* request, SetEstopConfigResponse* response) override;
  Status GetEstopSystemStatus(ServerContext* context, const GetEstopSystemStatusRequest* request, GetEstopSystemStatusResponse* response) override;
};

#endif
