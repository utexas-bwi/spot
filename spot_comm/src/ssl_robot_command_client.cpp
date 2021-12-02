#include <spot_comm/ssl_robot_command_client.h>

RobotCommandClient::RobotCommandClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = RobotCommandService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}
  
// Assembles the client's payload, sends it and presents the response back
// from the server.
RobotCommandResponse RobotCommandClient::startRobotCommand(Lease lease, RobotCommand command) {
  // Data we are sending to the server.
  RobotCommandRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_lease()->CopyFrom(lease);
  request.mutable_command()->CopyFrom(command);
  request.set_clock_identifier("spot_time_sync");
  
  // Container for the data we expect from the server.
  RobotCommandResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RobotCommand(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
    // std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}