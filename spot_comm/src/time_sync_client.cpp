
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
  TimeSyncUpdateResponse TimeSyncUpdate(TimeSyncUpdateRequest request, const std::string& clock_identifier) {
    // Container for the data we expect from the server.
    TimeSyncUpdateResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    std::cout << "Client start got here" << std::endl;

    request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());

    // The actual RPC.
    Status status = stub_->TimeSyncUpdate(&context, request, &reply);

    // std::cout << reply.state().status() << std::endl;
    // std::cout << reply.header().request_received_timestamp() << std::endl;
    // std::cout << reply.header().response_timestamp() << std::endl;
    // std::cout << reply.previous_estimate().round_trip_time() << std::endl;
    // std::cout << reply.previous_estimate().clock_skew() << std::endl;

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

  TimeSyncUpdateResponse EstablishTimeSync(int numRounds) {
    const std::string clock_identifier("spot_time_sync");
    TimeSyncUpdateResponse reply;
    TimeSyncUpdateRequest request;
    TimeSyncEstimate best;
    best.mutable_round_trip_time()->set_seconds(10000000); // is this needed?

    for(int i = 0; i <= numRounds && (reply.state().status() == 1 || i == 0); i++) {
      request.set_clock_identifier(clock_identifier);
      reply = TimeSyncUpdate(request, clock_identifier);

      request.mutable_previous_round_trip()->mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
      request.mutable_previous_round_trip()->mutable_client_tx()->CopyFrom(reply.header().request_header().request_timestamp());
      request.mutable_previous_round_trip()->mutable_server_tx()->CopyFrom(reply.header().response_timestamp());
      request.mutable_previous_round_trip()->mutable_server_rx()->CopyFrom(reply.header().request_received_timestamp());

      std::cout << i << std::endl;

      std::cout << reply.clock_identifier() << std::endl;
      
      if(i > 0) {
        if(reply.previous_estimate().round_trip_time() == best.round_trip_time()) {
          std::cout << "averaging" << std::endl;
          Duration averageSkew = best.clock_skew() + reply.previous_estimate().clock_skew();
          averageSkew /= 2; 
          std::cout << "new skew: " << averageSkew << std::endl;
          best.mutable_clock_skew()->CopyFrom(averageSkew);
        }
        else if(reply.previous_estimate().round_trip_time() < best.round_trip_time()) {
          std::cout << "copying" << std::endl;
          best.mutable_round_trip_time()->CopyFrom(reply.previous_estimate().round_trip_time());
          best.mutable_clock_skew()->CopyFrom(reply.previous_estimate().clock_skew());
        }
        else {
          std::cout << "not copying" << std::endl;
        }
      }
    }

    reply.mutable_state()->mutable_best_estimate()->CopyFrom(best);

    std::cout << "best estimate rt: " << reply.state().best_estimate().round_trip_time() << std::endl;
    std::cout << "best estimate cs: " << reply.state().best_estimate().clock_skew() << std::endl;
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
  
  TimeSyncUpdateResponse reply = timeClient.EstablishTimeSync(25);

  if(reply.state().status() == 1) {
    std::cout << "Time sync completed: " << reply.state().status() << std::endl;
  }

  return 0;
}