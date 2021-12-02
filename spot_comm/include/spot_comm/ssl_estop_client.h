#ifndef SSL_ESTOP_CLIENT_H
#define SSL_ESTOP_CLIENT_H


#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

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
using bosdyn::api::EstopEndpoint;
using bosdyn::api::EstopService;
using google::protobuf::Duration;

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

class EstopClient {
public:
  EstopClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  std::string RegisterEstopEndpoint(EstopEndpoint new_endpoint);

 private:
  std::unique_ptr<EstopService::Stub> stub_;
};

#endif
