#include <spot_comm/ssl_estop_client.h>

EstopClient::EstopClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = EstopService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

// Assembles the client's payload, sends it and presents the response back
// from the server.
std::string EstopClient::RegisterEstopEndpoint(EstopEndpoint new_endpoint) {
  // Data we are sending to the server.
  RegisterEstopEndpointRequest request;
  request.mutable_new_endpoint()->CopyFrom(new_endpoint);
  std::cout << "Endpoint unique ID: " << request.new_endpoint().unique_id() << std::endl;

  // Container for the data we expect from the server.
  RegisterEstopEndpointResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RegisterEstopEndpoint(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Response: " << reply.new_endpoint().unique_id() << std::endl;
    std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
    return reply.new_endpoint().unique_id();
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return "RPC failed";
  }
}
