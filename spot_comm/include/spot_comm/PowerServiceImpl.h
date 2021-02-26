#ifndef POWER_SERVICE_IMPL_H
#define POWER_SERVICE_IMPL_H

#include "bosdyn/api/power_service.grpc.pb.h"
#include "bosdyn/api/robot_state_service.grpc.pb.h"

using bosdyn::api::PowerCommandRequest;
using bosdyn::api::PowerCommandResponse;
using bosdyn::api::PowerCommandFeedbackRequest;
using bosdyn::api::PowerCommandFeedbackResponse;
using bosdyn::api::PowerService;
using bosdyn::api::PowerCommandStatus;
using bosdyn::api::PowerState;
using bosdyn::api::PowerState_MotorPowerState;
using bosdyn::api::LeaseUseResult;
using bosdyn::api::LicenseInfo;
using grpc::Status;
using grpc::ServerContext;

class PowerServiceImpl final : public PowerService::Service {
public:
  Status PowerCommand(ServerContext* context, const PowerCommandRequest* request, PowerCommandResponse* response) override;
  Status PowerCommandFeedback(ServerContext* context, const PowerCommandFeedbackRequest* request, PowerCommandFeedbackResponse* response) override;
};

#endif
