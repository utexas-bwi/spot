#ifndef SSL_POWER_CLIENT_H
#define SSL_POWER_CLIENT_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>

#include "bosdyn/api/power_service.grpc.pb.h"
#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

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

class PowerClient {
public:
  PowerClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  PowerCommandResponse PowerCommand(const PowerCommandRequest& request);
  PowerCommandFeedbackResponse PowerCommandFeedback(const PowerCommandFeedbackRequest& request);

 private:
  std::unique_ptr<PowerService::Stub> stub_;
};

#endif
