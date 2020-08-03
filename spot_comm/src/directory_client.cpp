
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/directory_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::ServiceEntry;
using bosdyn::api::Endpoint;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::DirectoryService;

class DirectoryClient {
 public:
  DirectoryClient(std::shared_ptr<Channel> channel)
      : stub_(DirectoryService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  ServiceEntry GetServiceEntry(const std::string& service_name) {
    // Data we are sending to the server.
    GetServiceEntryRequest request;
    request.set_service_name(service_name);

    // Container for the data we expect from the server.
    GetServiceEntryResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->GetServiceEntry(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.status() << ", Service Name: " << reply.service_entry().name() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply.service_entry();
  }


 private:
  std::unique_ptr<DirectoryService::Stub> stub_;
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
  DirectoryClient dirClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  std::string serviceName("service_test");
  ServiceEntry reply = dirClient.GetServiceEntry(serviceName);
  std::cout << "Service Entry received: " << reply.name() << std::endl;

  return 0;
}

