
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/auth_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

class AuthClient {
 public:
  AuthClient(std::shared_ptr<Channel> channel)
      : stub_(AuthService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string GetAuthToken(const std::string& user, const std::string& pass, const std::string& appToken) {
    // Data we are sending to the server.
    GetAuthTokenRequest request;
    request.set_username(user);
    request.set_password(pass);
    request.set_application_token(appToken);

    // Container for the data we expect from the server.
    GetAuthTokenResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->GetAuthToken(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Token Status: " << reply.status() << ", Token: " << reply.token() << std::endl;
      return reply.token();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }


  // Overloaded service call using existing token that needs reminting
  std::string GetAuthToken(const std::string& token, const std::string& appToken) {

    // Data we are sending to the server.
    GetAuthTokenRequest request;
    request.set_token(token);
    request.set_application_token(appToken);

    // Container for the data we expect from the server.
    GetAuthTokenResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->GetAuthToken(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Token Status: " << reply.status() << ", Token: " << reply.token() << std::endl;
      return reply.token();
    } else {
      std::cout << status.error_code() << ": " << status.error_message() << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<AuthService::Stub> stub_;
};

int main(int argc, char** argv) {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  // We indicate that the channel isn't authenticated (use of
  // InsecureChannelCredentials()).
  grpc_init();
  std::string target_str;
  std::string arg_str("--target");
  if (argc > 1) {
    std::string arg_val = argv[1];
    size_t start_pos = arg_val.find(arg_str);
    if (start_pos != std::string::npos) {
      start_pos += arg_str.size();
      if (arg_val[start_pos] == '=') {
        target_str = arg_val.substr(start_pos + 1);
      } else {
        std::cout << "The only correct argument syntax is --target=" << std::endl;
        return 0;
      }
    } else {
      std::cout << "The only acceptable argument is --target=" << std::endl;
      return 0;
    }
  } else {
    target_str = "127.0.0.1:50051";
  }
  AuthClient authClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  std::string user("testUser");
  std::string pass("testPassword");
  std::string appToken("testAppToken");
  std::string reply = authClient.GetAuthToken(user, pass, appToken);
  std::cout << "Token received: " << reply << std::endl;

  return 0;
}

