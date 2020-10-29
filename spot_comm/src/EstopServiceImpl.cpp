#include <spot_comm/EstopServiceImpl.h>
#include <spot_comm/Header.h>

Status EstopServiceImpl::RegisterEstopEndpoint(ServerContext* context, const RegisterEstopEndpointRequest* request, RegisterEstopEndpointResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->mutable_request()->CopyFrom(*request);
  response->mutable_new_endpoint()->CopyFrom(request->new_endpoint());
  response->set_status(RegisterEstopEndpointResponse::STATUS_SUCCESS);
  return Status::OK;
}

Status EstopServiceImpl::DeregisterEstopEndpoint(ServerContext* context, const DeregisterEstopEndpointRequest* request, DeregisterEstopEndpointResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->mutable_request()->CopyFrom(*request);
  response->set_status(DeregisterEstopEndpointResponse::STATUS_SUCCESS);
  return Status::OK;
}

Status EstopServiceImpl::EstopCheckIn(ServerContext* context, const EstopCheckInRequest* request, EstopCheckInResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->mutable_request()->CopyFrom(*request);
  response->set_challenge(0);
  response->set_status(EstopCheckInResponse::STATUS_OK);
  return Status::OK;
}

Status EstopServiceImpl::GetEstopConfig(ServerContext* context, const GetEstopConfigRequest* request, GetEstopConfigResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->mutable_request()->CopyFrom(*request);
  std::string unique_id("testID");
  EstopConfig active_config;
  EstopEndpoint endpoints;
  std::string role("testRole");
  endpoints.set_role(role);
  std::string name("testName");
  endpoints.set_name(name);
  endpoints.set_unique_id(unique_id);
  Duration timeout;
  timeout.set_seconds(999.0);
  timeout.set_nanos(1.0);
  endpoints.mutable_timeout()->CopyFrom(timeout);
  active_config.add_endpoints()->CopyFrom(endpoints);
  active_config.set_unique_id(unique_id);
  response->mutable_active_config()->CopyFrom(active_config);
  return Status::OK;
}

Status EstopServiceImpl::SetEstopConfig(ServerContext* context, const SetEstopConfigRequest* request, SetEstopConfigResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->mutable_request()->CopyFrom(*request);
  response->mutable_active_config()->CopyFrom(request->config());
  response->set_status(SetEstopConfigResponse::STATUS_SUCCESS);
  return Status::OK;
}

Status EstopServiceImpl::GetEstopSystemStatus(ServerContext* context, const GetEstopSystemStatusRequest* request, GetEstopSystemStatusResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  EstopSystemStatus status;
  status.set_stop_level(EstopStopLevel::ESTOP_LEVEL_NONE);
  EstopEndpointWithStatus endpoints;
  EstopEndpoint endpoint;
  std::string role("testRole");
  endpoint.set_role(role);
  std::string name("testName");
  endpoint.set_name(name);
  std::string unique_id("testID");
  endpoint.set_unique_id(unique_id);
  Duration timeout;
  timeout.set_seconds(999.0);
  timeout.set_nanos(1.0);
  endpoint.mutable_timeout()->CopyFrom(timeout);
  endpoints.mutable_endpoint()->CopyFrom(endpoint);
  endpoints.set_stop_level(EstopStopLevel::ESTOP_LEVEL_NONE);
  Duration time_since_valid_response;
  time_since_valid_response.set_seconds(1.0);
  time_since_valid_response.set_nanos(1.0);
  endpoints.mutable_time_since_valid_response()->CopyFrom(time_since_valid_response);
  response->mutable_status()->CopyFrom(status);
  return Status::OK;
}
