
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/auth_service.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

class AuthServiceImpl final : public AuthService::Service {
  Status GetAuthToken(ServerContext* context, const GetAuthTokenRequest* request,
                  GetAuthTokenResponse* response) override {
    std::string token("testToken");
    std::cout << "Username: " << request->username() << std::endl << "Pasword: " << request->password() << std::endl;
    response->set_status(GetAuthTokenResponse::STATUS_OK);
    response->set_token(token);
    return Status::OK;
  }
};

void RunServer() {
  std::string server_address("127.0.0.1:50051");
  AuthServiceImpl service;

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  grpc_init();
  RunServer();
  return 0;
}

