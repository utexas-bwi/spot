#ifndef ROBOT_ID_SERVICE_IMPL_H
#define ROBOT_ID_SERVICE_IMPL_H

#include "bosdyn/api/robot_id_service.grpc.pb.h"

using bosdyn::api::RobotIdRequest;
using bosdyn::api::RobotIdResponse;
using bosdyn::api::RobotIdService;
using bosdyn::api::RobotId;
using bosdyn::api::RobotSoftwareRelease;
using bosdyn::api::SoftwareVersion;
using grpc::Status;
using grpc::ServerContext;

class RobotIdServiceImpl final : public RobotIdService::Service {
public:
  Status GetRobotId(ServerContext* context, const RobotIdRequest* request, RobotIdResponse* response) override;
};

#endif
