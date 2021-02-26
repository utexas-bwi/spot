#ifndef ROBOT_STATE_SERVICE_IMPL_H
#define ROBOT_STATE_SERVICE_IMPL_H

#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include <google/protobuf/util/time_util.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <spot_comm/PowerServiceImpl.h>

using bosdyn::api::RobotStateService;
using bosdyn::api::RobotStateRequest;
using bosdyn::api::RobotStateResponse;
using bosdyn::api::RobotState;
using bosdyn::api::PowerState;
using bosdyn::api::BatteryState;
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
using bosdyn::api::FrameTreeSnapshot_ParentEdge;
using grpc::Status;
using grpc::ServerContext;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;
using google::protobuf::DoubleValue;

class RobotStateServiceImpl final : public RobotStateService::Service {
  public:
    RobotStateServiceImpl(ros::NodeHandle &n);
    Status GetRobotState(ServerContext* context, const RobotStateRequest* request, RobotStateResponse* response) override;
    Status GetRobotMetrics(ServerContext* context, const RobotMetricsRequest* request, RobotMetricsResponse* response) override;
    Status GetRobotHardwareConfiguration(ServerContext* context, const RobotHardwareConfigurationRequest* request, RobotHardwareConfigurationResponse* response) override;
    Status GetRobotLinkModel(ServerContext* context, const RobotLinkModelRequest* request, RobotLinkModelResponse* response) override;
  private:
    ros::NodeHandle &nh;

};

#endif
