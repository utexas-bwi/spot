#ifndef LOCAL_GRID_SERVICE_IMPL_H
#define LOCAL_GRID_SERVICE_IMPL_H

#include "bosdyn/api/local_grid_service.grpc.pb.h"

using bosdyn::api::GetLocalGridTypesRequest;
using bosdyn::api::GetLocalGridTypesResponse;
using bosdyn::api::GetLocalGridsRequest;
using bosdyn::api::GetLocalGridsResponse;
using bosdyn::api::LocalGridService;
using grpc::Status;
using grpc::ServerContext;

class LocalGridServiceImpl final : public LocalGridService::Service {
public:
  Status GetLocalGridTypes(ServerContext* context, const GetLocalGridTypesRequest* request, GetLocalGridTypesResponse* response) override;
  Status GetLocalGrids(ServerContext* context, const GetLocalGridsRequest* request, GetLocalGridsResponse* response) override;
};

#endif
