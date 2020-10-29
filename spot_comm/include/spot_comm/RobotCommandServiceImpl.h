#ifndef ROBOTCOMMAND_SERVICE_IMPL_H
#define ROBOTCOMMAND_SERVICE_IMPL_H

#include "bosdyn/api/robot_command_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/lease_service.grpc.pb.h"
#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <spot_comm/VelocityCommand.h>

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
using bosdyn::api::PowerState;
using bosdyn::api::PowerState_MotorPowerState;
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
using grpc::Status;
using grpc::ServerContext;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil; 

class RobotCommandServiceImpl final : public RobotCommandService::Service {
  public:
    RobotCommandServiceImpl(ros::NodeHandle &n);
    Status RobotCommand(ServerContext* context, const RobotCommandRequest* request, RobotCommandResponse* response) override;
    Status RobotCommandFeedback(ServerContext* context, const RobotCommandFeedbackRequest* request, RobotCommandFeedbackResponse* response) override;
    Status ClearBehaviorFault(ServerContext* context, const ClearBehaviorFaultRequest* request, ClearBehaviorFaultResponse* response) override;

  private:
    ros::NodeHandle &nh;
    VelocityCommand vel;
};

#endif
