#include <spot_comm/LeaseServiceImpl.h>

  void fillLease(Lease* lease) {
      lease->set_resource("testResource");
      lease->set_epoch("testEpoch");
      lease->add_sequence(0);
  }

  void fillLeaseOwner(LeaseOwner* owner) {
      owner->set_client_name("testClient");
      owner->set_user_name("testUser");
  }

  Status LeaseServiceImpl::AcquireLease(ServerContext* context, const AcquireLeaseRequest* request, AcquireLeaseResponse* response) {
      std::cout << "Lease Requested: " << request->resource() << std::endl;
      response->set_status(AcquireLeaseResponse::STATUS_OK);
      if (response->status() == AcquireLeaseResponse::STATUS_OK){
          fillLease(response->mutable_lease());
          fillLeaseOwner(response->mutable_lease_owner());
      }
      return Status::OK;
  }
  Status LeaseServiceImpl::TakeLease(ServerContext* context, const TakeLeaseRequest* request, TakeLeaseResponse* response) {
      return Status::OK;
  }
  Status LeaseServiceImpl::ReturnLease(ServerContext* context, const ReturnLeaseRequest* request, ReturnLeaseResponse* response) {
      return Status::OK;
  }
  Status LeaseServiceImpl::ListLeases(ServerContext* context, const ListLeasesRequest* request, ListLeasesResponse* response) {
      return Status::OK;
  }
  Status LeaseServiceImpl::RetainLease(ServerContext* context, const RetainLeaseRequest* request, RetainLeaseResponse* response) {
      return Status::OK;
  }

