#include <spot_comm/AuthServiceImpl.h>
#include <spot_comm/Header.h>

Status AuthServiceImpl::GetAuthToken(ServerContext* context, const GetAuthTokenRequest* request, GetAuthTokenResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  std::string token("testToken");
  std::cout << "Username: " << request->username() << std::endl << "Pasword: " << request->password() << std::endl;
  response->set_status(GetAuthTokenResponse::STATUS_OK);
  response->set_token(token);
  return Status::OK;
}
