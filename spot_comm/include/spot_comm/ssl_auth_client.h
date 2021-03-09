#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>
#include <ros/package.h>

#include "bosdyn/api/auth_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

class AuthClient {
public:
  AuthClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  std::string GetAuthToken(const std::string& user, const std::string& pass, const std::string& appToken);

 private:
  std::unique_ptr<AuthService::Stub> stub_;
};