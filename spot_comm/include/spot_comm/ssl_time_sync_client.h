#ifndef SSL_TIME_SYNC_CLIENT_H
#define SSL_TIME_SYNC_CLIENT_H

#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/time_sync_service.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class TimeSyncClient {
    public: 
        TimeSyncClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
        TimeSyncUpdateResponse TimeSyncUpdate(TimeSyncUpdateRequest request, const std::string& clock_identifier);
        TimeSyncUpdateResponse EstablishTimeSync(const int& numRounds);

    private:
        std::unique_ptr<TimeSyncService::Stub> stub_;
};


#endif