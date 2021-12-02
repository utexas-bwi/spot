#ifndef SSL_LEASE_CLIENT_H
#define SSL_LEASE_CLIENT_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>

#include "bosdyn/api/lease_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

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

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using bosdyn::api::LeaseService;

class LeaseClient {
public:
  LeaseClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  AcquireLeaseResponse AcquireLease(const std::string& resource);
  TakeLeaseResponse TakeLease(const std::string& resource);
  ReturnLeaseResponse ReturnLease(Lease* lease);
  ListLeasesResponse ListLeases();
  RetainLeaseResponse RetainLease(); 

 private:
  std::unique_ptr<LeaseService::Stub> stub_;
};

#endif
