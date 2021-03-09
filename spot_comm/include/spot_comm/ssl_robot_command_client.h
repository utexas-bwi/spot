#ifndef SSL_ROBOT_COMMAND_CLIENT_H
#define SSL_ROBOT_COMMAND_CLIENT_H

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

#include <geometry_msgs/Twist.h>
#include <spot_comm/VelocityCommand.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::RobotCommandService;
using bosdyn::api::RobotCommandRequest;
using bosdyn::api::RobotCommandResponse;
using bosdyn::api::RobotCommandResponse_Status;
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
  RobotCommandClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  RobotCommandResponse startRobotCommand(Lease lease, RobotCommand command);

 private:
  std::unique_ptr<RobotCommandService::Stub> stub_;
};

#endif