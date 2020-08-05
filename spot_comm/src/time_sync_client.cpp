
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/time_sync_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class TimeSyncClient {
 public:
  TimeSyncClient(std::shared_ptr<Channel> channel)
      : stub_(TimeSyncService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  TimeSyncUpdateResponse TimeSyncUpdate(TimeSyncRoundTrip* previous_round_trip, const std::string& clock_identifier) {
    // Data we are sending to the server.
    TimeSyncUpdateRequest request;
    request.set_allocated_previous_round_trip(previous_round_trip);
    request.set_clock_identifier(clock_identifier);

    // Container for the data we expect from the server.
    TimeSyncUpdateResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    std::cout << "Client start got here" << std::endl;

    // The actual RPC.
    Status status = stub_->TimeSyncUpdate(&context, request, &reply);

    std::cout << "Client end got here" << std::endl;

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Status: " << reply.state().status() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return reply;
  }


 private:
  std::unique_ptr<TimeSyncService::Stub> stub_;
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
  
  TimeSyncClient timeClient(grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));
  TimeSyncRoundTrip* prev_trip = new TimeSyncRoundTrip();
  std::string clock_identifier("spot-time-sync");

  TimeSyncUpdateResponse reply = timeClient.TimeSyncUpdate(prev_trip, clock_identifier);
  std::cout << "Time sync round completed: " << reply.state().status() << std::endl;

  return 0;
}