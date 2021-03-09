#include <spot_comm/ssl_auth_client.h>

AuthClient::AuthClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = AuthService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

std::string AuthClient::GetAuthToken(const std::string& user, const std::string& pass, const std::string& appToken) {
  // Data we are sending to the server.
  GetAuthTokenRequest request;
  request.set_username(user);
  request.set_password(pass);
  // request.set_application_token(appToken); (deprecated)

  // Container for the data we expect from the server.
  GetAuthTokenResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetAuthToken(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Token Status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    return reply.token();
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return "RPC failed";
  }
}
