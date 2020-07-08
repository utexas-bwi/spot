#include <bosdyn/api/auth_service.grpc.pb.h>
#include <spot_comm/auth_service.h>

GetAuthToken(ServerContext* context, const GetAuthTokenRequest* request,
                  GetAuthTokenResponse* response) {
    std::string token("testToken");
    std::cout << "Username: " << request->username() << std::endl << "Pasword: " << request->password() << std::endl;
    response->set_status(GetAuthTokenResponse::STATUS_OK);
    response->set_token(token);
    return Status::OK;
}
