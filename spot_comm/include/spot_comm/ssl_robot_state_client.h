#ifndef SSL_ROBOT_STATE_CLIENT_H
#define SSL_ROBOT_STATE_CLIENT_H

#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include <google/protobuf/util/time_util.h>
#include "bosdyn/api/header.grpc.pb.h"
#include <ros/ros.h>
#include <ros/package.h>

using bosdyn::api::RobotStateService;
using bosdyn::api::RobotStateRequest;
using bosdyn::api::RobotStateResponse;
using bosdyn::api::RobotState;
using bosdyn::api::PowerState;
using bosdyn::api::CommsState;
using bosdyn::api::SystemFaultState;
using bosdyn::api::EStopState;
using bosdyn::api::KinematicState;
using bosdyn::api::BehaviorFaultState; 
using bosdyn::api::FootState;
using bosdyn::api::RobotMetricsRequest;
using bosdyn::api::RobotMetricsResponse;
using bosdyn::api::RobotHardwareConfigurationRequest;
using bosdyn::api::RobotHardwareConfigurationResponse;
using bosdyn::api::RobotLinkModelRequest;
using bosdyn::api::RobotLinkModelResponse;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

class RobotStateClient {
 public:
  RobotStateClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server); 
  RobotStateResponse GetRobotState();

 private:
  std::unique_ptr<RobotStateService::Stub> stub_;
};

#endif
