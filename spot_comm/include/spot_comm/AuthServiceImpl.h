#ifndef AUTH_SERVICE_IMPL_H
#define AUTH_SERVICE_IMPL_H

#include "bosdyn/api/auth_service.grpc.pb.h"

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;
using grpc::Status;
using grpc::ServerContext;

class AuthServiceImpl final : public AuthService::Service {
public:
  Status GetAuthToken(ServerContext* context, const GetAuthTokenRequest* request,
                  GetAuthTokenResponse* response) override;
};

#endif
