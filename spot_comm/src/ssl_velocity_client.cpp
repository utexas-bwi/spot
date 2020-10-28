#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/robot_command_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/lease_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include "spot_comm/VelocityCommand.h"

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
using bosdyn::api::FullBodyCommand_Feedback;
using bosdyn::api::StopCommand;
using bosdyn::api::StopCommand_Feedback;
using bosdyn::api::FreezeCommand;
using bosdyn::api::FreezeCommand_Feedback;
using bosdyn::api::SelfRightCommand;
using bosdyn::api::SelfRightCommand_Feedback;
using bosdyn::api::SafePowerOffCommand;
using bosdyn::api::SafePowerOffCommand_Feedback;
using bosdyn::api::MobilityCommand;
using bosdyn::api::MobilityCommand_Feedback;
using bosdyn::api::SE2TrajectoryCommand;
using bosdyn::api::SE2TrajectoryCommand_Feedback;
using bosdyn::api::SE2VelocityCommand;
using bosdyn::api::SE2VelocityCommand_Feedback;
using bosdyn::api::SitCommand;
using bosdyn::api::SitCommand_Feedback;
using bosdyn::api::StandCommand;
using bosdyn::api::StandCommand_Feedback;
using bosdyn::api::Lease;
using bosdyn::api::LeaseUseResult;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class RobotCommandClient {
public:
  RobotCommandClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
    grpc::SslCredentialsOptions opts = {root, key, cert};
    stub_ = RobotCommandService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
  }
  
  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  RobotCommandResponse startRobotCommand(Lease lease, RobotCommand command) {
    // Data we are sending to the server.
    RobotCommandRequest request;
    request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    request.mutable_lease()->CopyFrom(lease);
    request.mutable_command()->CopyFrom(command);
    request.set_clock_identifier("spot_time_sync");
    
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
      // std::cout << reply.message() << std::endl;
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

void read(const std::string& filename, std::string& data) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (file.is_open()) {
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    data = ss.str();
  }
  return;
}

int main (int argc, char** argv) {
  std::string server {"localhost:50051"};

  std::string pathToPackage = ros::package::getPath("spot_comm");
  std::string key, cert, root;
  read(pathToPackage + "/include/certs/client.key", key);
  read(pathToPackage + "/include/certs/client.crt", cert);
  read(pathToPackage + "/include/certs/ca.crt", root);

  RobotCommandClient robotClient(cert, key, root, server);

  Lease lease;
  lease.set_resource("testResource");
  lease.set_epoch("testEpoch");
  lease.add_sequence(100);
  RobotCommand command;
  Duration commandDuration;
  commandDuration.set_seconds(10);
  Timestamp end = TimeUtil::GetCurrentTime() + commandDuration;
  command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_end_time()->CopyFrom(end);
  command.mutable_mobility_command()->mutable_se2_velocity_request()->set_se2_frame_name("testFrame");
  command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_x(1);
  command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_y(0);
  command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->set_angular(1);
  RobotCommandResponse response = robotClient.startRobotCommand(lease, command);
  std::cout << "Response received: " << response.DebugString() << std::endl;

  return 0;
}
