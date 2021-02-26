
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/log_annotation_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::AddLogAnnotationRequest;
using bosdyn::api::AddLogAnnotationResponse;
using bosdyn::api::LogAnnotations;
using bosdyn::api::LogAnnotationService;

class LogAnnotationClient {
 public:
  LogAnnotationClient(std::shared_ptr<Channel> channel)
      : stub_(LogAnnotationService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string AddLogAnnotation(LogAnnotations annotations) {
    // Data we are sending to the server.
    AddLogAnnotationRequest request;
    request.mutable_annotations()->CopyFrom(annotations);
    std::cout << "Text Message To Send: " << request.annotations().text_messages(0).message() << std::endl;

    // Container for the data we expect from the server.
    AddLogAnnotationResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->AddLogAnnotation(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
      return "RPC successful";
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<LogAnnotationService::Stub> stub_;
};

int main(int argc, char** argv) {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  // We indicate that the channel isn't authenticated (use of
  // InsecureChannelCredentials()).
  grpc_init();
  std::string target_str;
  std::string arg_str("--target");
  if (argc > 1) {
    std::string arg_val = argv[1];
    size_t start_pos = arg_val.find(arg_str);
    if (start_pos != std::string::npos) {
      start_pos += arg_str.size();
      if (arg_val[start_pos] == '=') {
        target_str = arg_val.substr(start_pos + 1);
      } else {
        std::cout << "The only correct argument syntax is --target=" << std::endl;
        return 0;
      }
    } else {
      std::cout << "The only acceptable argument is --target=" << std::endl;
      return 0;
    }
  } else {
    target_str = "127.0.0.1:50051";
  }
  LogAnnotationClient LogAnnotationClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  LogAnnotations annotations;
  annotations.add_text_messages()->set_message("testTextMessage0");
  annotations.add_text_messages()->set_message("testTextMessage1");
  annotations.add_operator_messages()->set_message("testOperatorMessage0");
  annotations.add_blob_data()->set_data("testBlobData0");
  std::string reply = LogAnnotationClient.AddLogAnnotation(annotations);
  std::cout << reply << std::endl;
  return 0;
}
