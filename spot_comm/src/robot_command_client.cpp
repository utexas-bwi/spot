#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/robot_command_service.grpc.pb.h"
// #include "bosdyn/api/header.grpc.pb.h"
// #include "bosdyn/api/lease_service.grpc.pb.h"
// #include "bosdyn/api/geometry.grpc.pb.h"
// #include "bosdyn/api/full_body_command.grpc.pb.h"
// #include "bosdyn/api/mobility_command.grpc.pb.h"
// #include "bosdyn/api/basic_command.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::RobotCommandService;
using bosdyn::api::RobotCommandRequest;
using bosdyn::api::RobotCommandResponse;
using bosdyn::api::RobotCommandFeedbackRequest;
using bosdyn::api::RobotCommandFeedbackResponse;
using bosdyn::api::ClearBehaviorFaultRequest;
using bosdyn::api::ClearBehaviorFaultResponse;
using bosdyn::api::RobotCommand;
using bosdyn::api::RobotCommandFeedback;
using bosdyn::api::FullBodyCommand;
using bosdyn::api::MobilityCommand;
using bosdyn::api::Lease;
using bosdyn::api::LeaseUseResult;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class RobotCommandClient {
 public:
  RobotCommandClient(std::shared_ptr<Channel> channel)
      : stub_(RobotCommandService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  RobotCommandResponse RobotCommand() {
    // Data we are sending to the server.
    RobotCommandRequest request;
    
    // Container for the data we expect from the server.
    RobotCommandResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->RobotCommand(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
      std::cout << "Success" << std::endl;
      // return "reply.token()";
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      // return "RPC failed";
    }

    return reply;
  }

 private:
  std::unique_ptr<RobotCommandService::Stub> stub_;
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
  
  RobotCommandClient commandClient(grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  // RobotCommandResponse reply = commandClient.RobotCommand();
  // std::cout << "Token received: " << reply.status() << std::endl;

  return 0;
}