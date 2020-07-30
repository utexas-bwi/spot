#include <spot_comm/EstopServiceImpl.h>

Status EstopServiceImpl::RegisterEstopEndpoint(ServerContext* context, const RegisterEstopEndpointRequest* request, RegisterEstopEndpointResponse* response) {
  RegisterEstopEndpointRequest request_copy = RegisterEstopEndpointRequest(*request);
  response->set_allocated_request(&request_copy);
  EstopEndpoint new_endpoint_copy = EstopEndpoint(request->new_endpoint());
  response->set_allocated_new_endpoint(&new_endpoint_copy);
  response->set_status(RegisterEstopEndpointResponse::STATUS_SUCCESS);
}

Status EstopServiceImpl::DeregisterEstopEndpoint(ServerContext* context, const DeregisterEstopEndpointRequest* request, DeregisterEstopEndpointResponse* response) {
  DeregisterEstopEndpointRequest request_copy = DeregisterEstopEndpointRequest(*request);
  response->set_allocated_request(&request_copy);
  response->set_status(DeregisterEstopEndpointResponse::STATUS_SUCCESS);
}

Status EstopServiceImpl::EstopCheckIn(ServerContext* context, const EstopCheckInRequest* request, EstopCheckInResponse* response) {
  EstopCheckInRequest request_copy = EstopCheckInRequest(*request);
  response->set_allocated_request(&request_copy);
  response->set_challenge(0);
  response->set_status(EstopCheckInResponse::STATUS_OK);
}

Status EstopServiceImpl::GetEstopConfig(ServerContext* context, const GetEstopConfigRequest* request, GetEstopConfigResponse* response) {
  GetEstopConfigRequest request_copy = GetEstopConfigRequest(*request);
  response->set_allocated_request(&request_copy);
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
  endpoints.set_allocated_timeout(&timeout);
  *active_config.add_endpoints() = endpoints;
  active_config.set_unique_id(unique_id);
  response->set_allocated_active_config(&active_config);
}

Status EstopServiceImpl::SetEstopConfig(ServerContext* context, const SetEstopConfigRequest* request, SetEstopConfigResponse* response) {
  SetEstopConfigRequest request_copy = SetEstopConfigRequest(*request);
  response->set_allocated_request(&request_copy);
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
  endpoints.set_allocated_timeout(&timeout);
  *active_config.add_endpoints() = endpoints;
  active_config.set_unique_id(unique_id);
  response->set_allocated_active_config(&active_config);
  response->set_status(SetEstopConfigResponse::STATUS_SUCCESS);
}

Status EstopServiceImpl::GetEstopSystemStatus(ServerContext* context, const GetEstopSystemStatusRequest* request, GetEstopSystemStatusResponse* response) {
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
  endpoint.set_allocated_timeout(&timeout);
  endpoints.set_allocated_endpoint(&endpoint);
  endpoints.set_stop_level(EstopStopLevel::ESTOP_LEVEL_NONE);
  Duration time_since_valid_response;
  time_since_valid_response.set_seconds(1.0);
  time_since_valid_response.set_nanos(1.0);
  endpoints.set_allocated_time_since_valid_response(&time_since_valid_response);
  response->set_allocated_status(&status);
}
