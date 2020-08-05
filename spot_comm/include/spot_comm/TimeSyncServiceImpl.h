#ifndef TIMESYNC_SERVICE_IMPL_H
#define TIMESYNC_SERVICE_IMPL_H

#include "bosdyn/api/time_sync_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;
using grpc::Status;
using grpc::ServerContext;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil; 

class TimeSyncServiceImpl final : public TimeSyncService::Service {
  public:
    Status TimeSyncUpdate(ServerContext* context, const TimeSyncUpdateRequest* request, TimeSyncUpdateResponse* response) override;
};

#endif