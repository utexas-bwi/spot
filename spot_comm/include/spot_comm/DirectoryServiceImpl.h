#ifndef DIRECTORY_SERVICE_IMPL_H
#define DIRECTORY_SERVICE_IMPL_H

#include "bosdyn/api/directory_service.grpc.pb.h"

using bosdyn::api::ServiceEntry;
using bosdyn::api::Endpoint;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::DirectoryService;
using google::protobuf::Timestamp;
using grpc::Status;
using grpc::ServerContext;


class DirectoryServiceImpl final : public DirectoryService::Service {
public:
  Status GetServiceEntry(ServerContext* context, const GetServiceEntryRequest* request,
                  GetServiceEntryResponse* response) override;

  Status ListServiceEntries(ServerContext* context, const ListServiceEntriesRequest* request,
                  ListServiceEntriesResponse* response) override;

};

#endif
