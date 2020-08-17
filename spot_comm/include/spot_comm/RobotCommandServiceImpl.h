#ifndef ROBOTCOMMAND_SERVICE_IMPL_H
#define ROBOTCOMMAND_SERVICE_IMPL_H

#include "bosdyn/api/robot_command_service.grpc.pb.h"
// #include "bosdyn/api/full_body_command.grpc.pb.h"
// #include "bosdyn/api/mobility_command.grpc.pb.h"
// #include "bosdyn/api/basic_command.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

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
using grpc::Status;
using grpc::ServerContext;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil; 

class RobotCommandServiceImpl final : public RobotCommandService::Service {
  public:
    Status RobotCommand(ServerContext* context, const RobotCommandRequest* request, RobotCommandResponse* response) override;
    Status RobotCommandFeedback(ServerContext* context, const RobotCommandFeedbackRequest* request, RobotCommandFeedbackResponse* response) override;
    Status ClearBehaviorFault(ServerContext* context, const ClearBehaviorFaultRequest* request, ClearBehaviorFaultResponse* response) override;

};

#endif