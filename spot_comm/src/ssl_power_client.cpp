#include <spot_comm/ssl_power_client.h>

PowerClient::PowerClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = PowerService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

PowerCommandResponse PowerClient::PowerCommand(const PowerCommandRequest& request) {
  // Container for the data we expect from the server.
  PowerCommandResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->PowerCommand(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Power Command Status: " << reply.status() << std::endl;
    return reply;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return reply;
  }
}

PowerCommandFeedbackResponse PowerClient::PowerCommandFeedback(const PowerCommandFeedbackRequest& request) {
  // Container for the data we expect from the server.
  PowerCommandFeedbackResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->PowerCommandFeedback(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Power Command Feedback Status: " << reply.status() << std::endl;
    return reply;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return reply;
  }
}
