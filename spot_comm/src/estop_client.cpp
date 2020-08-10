
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/estop_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
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

class EstopClient {
 public:
  EstopClient(std::shared_ptr<Channel> channel)
      : stub_(EstopService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string RegisterEstopEndpoint(EstopEndpoint new_endpoint) {
    // Data we are sending to the server.
    RegisterEstopEndpointRequest request;
    *(request.mutable_new_endpoint()) = new_endpoint;
    std::cout << "Endpoint unique ID: " << request.new_endpoint().unique_id();

    // Container for the data we expect from the server.
    RegisterEstopEndpointResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->RegisterEstopEndpoint(&context, request, &reply);
    
    std::cout << "Hi again" << std::endl;

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.status() << ", Response: " << reply.new_endpoint().unique_id() << std::endl;
      return reply.new_endpoint().unique_id();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<EstopService::Stub> stub_;
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
  EstopClient estopClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  EstopEndpoint new_endpoint;
  std::string role("testRole");
  new_endpoint.set_role(role);
  std::string name("testName");
  new_endpoint.set_name(name);
  std::string unique_id("testID");
  new_endpoint.set_unique_id(unique_id);
  Duration timeout;
  timeout.set_seconds(999.0);
  timeout.set_nanos(1.0);
  std::cout << "Hello once" << std::endl;
  *(new_endpoint.mutable_timeout()) = timeout;
  std::cout << "Hello" << std::endl;
  std::string reply = estopClient.RegisterEstopEndpoint(new_endpoint);
  //std::cout << "Message recieved:" << reply << std::endl;

  return 0;
}
