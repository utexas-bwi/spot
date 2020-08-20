#include <spot_comm/LeaseServiceImpl.h>

  void fillLease(Lease* lease, std::string resource, std::string epoch, uint32_t sequence) {
      lease->set_resource(resource);
      lease->set_epoch(epoch);
      lease->add_sequence(sequence);
  }

  void fillLeaseOwner(LeaseOwner* owner, std::string clientName, std::string userName) {
      owner->set_client_name(clientName);
      owner->set_user_name(userName);
  }


  Status LeaseServiceImpl::AcquireLease(ServerContext* context, const AcquireLeaseRequest* request, AcquireLeaseResponse* response) {
      std::cout << "Lease Requested: " << request->resource() << std::endl;
      response->set_status(AcquireLeaseResponse::STATUS_OK); // would set after checking ownership
      if (response->status() == AcquireLeaseResponse::STATUS_OK){
          fillLease(response->mutable_lease(), "testResource", "testEpoch", 0);
          fillLeaseOwner(response->mutable_lease_owner(), "testClient", "testUser");
      }
      return Status::OK;
  }

  Status LeaseServiceImpl::TakeLease(ServerContext* context, const TakeLeaseRequest* request, TakeLeaseResponse* response) {
      std::cout << "Lease Requested: " << request->resource() << std::endl;
      response->set_status(TakeLeaseResponse::STATUS_OK); // would set after checking ownership
      if (response->status() == TakeLeaseResponse::STATUS_OK){
          fillLease(response->mutable_lease(), "testResource", "testEpoch", 0);
          fillLeaseOwner(response->mutable_lease_owner(), "testClient", "testUser");
      }
      return Status::OK;
  }
  Status LeaseServiceImpl::ReturnLease(ServerContext* context, const ReturnLeaseRequest* request, ReturnLeaseResponse* response) {
      response->set_status(ReturnLeaseResponse::STATUS_OK); // would set after attempting a lease return
      return Status::OK;
  }
  Status LeaseServiceImpl::ListLeases(ServerContext* context, const ListLeasesRequest* request, ListLeasesResponse* response) {
      LeaseResource resource;
      resource.set_resource("testResource");
      fillLease(resource.mutable_lease(), "testResource", "testEpoch", 0);
      fillLeaseOwner(resource.mutable_lease_owner(), "testClient", "testUser");
      response->add_resources()->CopyFrom(resource);
      return Status::OK;
  }
  Status LeaseServiceImpl::RetainLease(ServerContext* context, const RetainLeaseRequest* request, RetainLeaseResponse* response) {
      LeaseUseResult result;
      result.set_status(LeaseUseResult::STATUS_OK);
      fillLeaseOwner(result.mutable_owner(), "testClient", "testUser");
      fillLease(result.mutable_attempted_lease(), "testResource", "testEpoch", 0);
      fillLease(result.mutable_previous_lease(), "testResource", "testEpoch", 0);
      response->mutable_lease_use_result()->CopyFrom(result);
      return Status::OK;

  }

