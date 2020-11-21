#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/robot_command_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/lease_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

#include <ros/ros.h>
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
  RobotCommandClient(std::shared_ptr<Channel> channel)
      : stub_(RobotCommandService::NewStub(channel)) {
        response = RobotCommandResponse();
        response.set_status(RobotCommandResponse::STATUS_OK);
  }

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  RobotCommandResponse StartRobotCommand(Lease lease, RobotCommand command) {
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

  void twist_callback(const geometry_msgs::Twist::ConstPtr& msg) {
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
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_x(msg->linear.x);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_y(msg->linear.y);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->set_angular(msg->angular.z);

    // slew_rate_limit - how quickly velocity can change relative to se2_frame_name's
    // double xDiff = 0;
    // double yDiff = 0;
    // double angularDiff = 0;
    // command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_slew_rate_limit()->mutable_linear()->set_x(xDiff);
    // command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_slew_rate_limit()->mutable_linear()->set_y(yDiff);
    // command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_slew_rate_limit()->set_angular(angularDiff);

    response = this->StartRobotCommand(lease, command);
  }

  RobotCommandResponse getResponse() {
    return response;
  }

 private:
  std::unique_ptr<RobotCommandService::Stub> stub_;
  RobotCommandResponse response;
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
  ros::init(argc, argv, "twist_node");
  ros::NodeHandle n;

  VelocityCommand vel(n);

  ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1, &RobotCommandClient::twist_callback, &commandClient);
  
  double xVel;
  double yVel;
  double angularVel;

  int counter = 0;

  ros::Rate r(1000);

  while(ros::ok() && commandClient.getResponse().status() == RobotCommandResponse::STATUS_OK && counter < 2000) {
    xVel = 1;
    yVel = 1;
    angularVel = 0;

    // vel.executeCommand(xVel, yVel, angularVel);

    // ROS_INFO("Status: %d", commandClient.getResponse().status());

    counter++;
    
    if(counter == 2000) {
      // vel.executeCommand(0, 0, 0);
    }
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}