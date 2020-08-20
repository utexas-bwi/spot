#include <spot_comm/PowerServiceImpl.h>
#include <spot_comm/Header.h>

Status PowerServiceImpl::PowerCommand(ServerContext* context, const PowerCommandRequest* request, PowerCommandResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->set_status(PowerCommandStatus::STATUS_SUCCESS);
  response->set_power_command_id(0);
  response->mutable_lease_use_result()->mutable_attempted_lease()->CopyFrom(request->lease());
  response->mutable_lease_use_result()->mutable_owner()->set_client_name("testClientName");
  response->mutable_lease_use_result()->mutable_owner()->set_user_name("testUserName");
  response->mutable_lease_use_result()->set_status(LeaseUseResult::STATUS_OK);
  response->set_license_status(LicenseInfo::STATUS_VALID);
  return Status::OK;
}

Status PowerServiceImpl::PowerCommandFeedback(ServerContext* context, const PowerCommandFeedbackRequest* request, PowerCommandFeedbackResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->set_status(PowerCommandStatus::STATUS_SUCCESS);
  return Status::OK;
}
