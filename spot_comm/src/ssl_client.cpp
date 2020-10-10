#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>

#include "bosdyn/api/auth_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

class AuthClient {
public:
  AuthClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
    grpc::SslCredentialsOptions opts = { root, key, cert };

		stub_ = AuthService::NewStub ( grpc::CreateChannel (server, grpc::SslCredentials ( opts ) ) );
	}

  std::string GetAuthToken(const std::string& user, const std::string& pass, const std::string& appToken) {
    // Data we are sending to the server.
    GetAuthTokenRequest request;
    request.set_username(user);
    request.set_password(pass);
    request.set_application_token(appToken);

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

 private:
  std::unique_ptr<AuthService::Stub> stub_;
};

void read ( const std::string& filename, std::string& data )
{
	std::ifstream file ( filename.c_str (), std::ios::in );

	if ( file.is_open () )
	{
		std::stringstream ss;
		ss << file.rdbuf ();

		file.close ();

		data = ss.str ();
	}

	return;
}

int main ( int argc, char** argv )
{
	std::string cert;
	std::string key;
	std::string root;
	std::string server { "localhost:50051" };

	read ( "client.crt", cert );
	read ( "client.key", key );
	read ( "ca.crt", root );

  AuthClient authClient ( cert, key, root, server );

  std::string user("testUser");
  std::string pass("testPassword");
  std::string appToken("testAppToken");
  std::string reply = authClient.GetAuthToken(user, pass, appToken);
  std::cout << "Token received: " << reply << std::endl;

  return 0;
}
