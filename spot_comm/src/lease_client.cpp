
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/lease_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::AcquireLeaseRequest;
using bosdyn::api::AcquireLeaseResponse;
using bosdyn::api::Lease;
using bosdyn::api::LeaseOwner;
using bosdyn::api::LeaseResource;
using bosdyn::api::LeaseUseResult;
using bosdyn::api::ListLeasesRequest;
using bosdyn::api::ListLeasesResponse;
using bosdyn::api::RetainLeaseRequest;
using bosdyn::api::RetainLeaseResponse;
using bosdyn::api::ReturnLeaseRequest;
using bosdyn::api::ReturnLeaseResponse;
using bosdyn::api::TakeLeaseRequest;
using bosdyn::api::TakeLeaseResponse;
using bosdyn::api::LeaseService;

class LeaseClient {
 public:
  LeaseClient(std::shared_ptr<Channel> channel)
      : stub_(LeaseService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  AcquireLeaseResponse AcquireLease(const std::string& resource) {
    // Data we are sending to the server.
    AcquireLeaseRequest request;
    request.set_resource(resource);

    // Container for the data we expect from the server.
    AcquireLeaseResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->AcquireLease(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
      ", New Owner: " << reply.lease_owner().user_name() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply;
  }

  TakeLeaseResponse TakeLease(const std::string& resource) {
    // Data we are sending to the server.
    TakeLeaseRequest request;
    request.set_resource(resource);

    // Container for the data we expect from the server.
    TakeLeaseResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->TakeLease(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.status() << ", Lease Taken: " << reply.lease().resource() <<
      ", New Owner: " << reply.lease_owner().user_name() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply;
  }

  ReturnLeaseResponse ReturnLease(Lease* lease) {
    // Data we are sending to the server.
    ReturnLeaseRequest request;
    request.set_allocated_lease(lease);

    // Container for the data we expect from the server.
    ReturnLeaseResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->ReturnLease(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.status() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply;
  }

  ListLeasesResponse ListLeases() {
    // Data we are sending to the server.
    ListLeasesRequest request;

    // Container for the data we expect from the server.
    ListLeasesResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->ListLeases(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << status.ok() << std::endl << "1st Lease: " << reply.resources(0).resource() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply;
  }  

 private:
  std::unique_ptr<LeaseService::Stub> stub_;
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
  LeaseClient leaseClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  std::string resource("resource_test");

  // test AcquireLease service
  AcquireLeaseResponse acquireReply = leaseClient.AcquireLease(resource);

  std::cout << "Acquire lease successful: " << acquireReply.status() << std::endl << std::endl;

  // test TakeLease service
  TakeLeaseResponse takeReply = leaseClient.TakeLease(resource);

  std::cout << "Take lease successful: " << takeReply.status() << std::endl << std::endl;
  
  // test ReturnLease service
  Lease* toReturn;
  ReturnLeaseResponse returnReply = leaseClient.ReturnLease(toReturn);

  std::cout << "Return lease successful: " << returnReply.status() << std::endl << std::endl;
  
  ListLeasesResponse listReply = leaseClient.ListLeases();

  std::cout << "List lease successful: " << (listReply.resources_size() > 0) << std::endl; 
  return 0;
}

