#ifndef LEASE_SERVICE_IMPL_H
#define LEASE_SERVICE_IMPL_H

#include "bosdyn/api/lease_service.grpc.pb.h"

using bosdyn::api::AcquireLeaseRequest;
using bosdyn::api::AcquireLeaseResponse;
using bosdyn::api::Lease;
using bosdyn::api::LeaseOwner;
using bosdyn::api::LeaseResource;
using bosdyn::api::LeaseUseResult;
using bosdyn::api::ListLeasesRequest;
using bosdyn::api::ListLeasesResponse;
using bosdyn::api::RetainLeaseRequest;
using bosdyn::api::RetainLeaseResponse;
using bosdyn::api::ReturnLeaseRequest;
using bosdyn::api::ReturnLeaseResponse;
using bosdyn::api::TakeLeaseRequest;
using bosdyn::api::TakeLeaseResponse;
using bosdyn::api::LeaseService;
using grpc::Status;
using grpc::ServerContext;

class LeaseServiceImpl final : public LeaseService::Service {
public:
  Status AcquireLease(ServerContext* context, const AcquireLeaseRequest* request, AcquireLeaseResponse* response) override;
  Status TakeLease(ServerContext* context, const TakeLeaseRequest* request, TakeLeaseResponse* response) override;
  Status ReturnLease(ServerContext* context, const ReturnLeaseRequest* request, ReturnLeaseResponse* response) override;
  Status ListLeases(ServerContext* context, const ListLeasesRequest* request, ListLeasesResponse* response) override;
  Status RetainLease(ServerContext* context, const RetainLeaseRequest* request, RetainLeaseResponse* response) override;
};

#endif
