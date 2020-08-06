
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/image_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::GetImageRequest;
using bosdyn::api::ImageRequest;
using bosdyn::api::Image;
using bosdyn::api::GetImageResponse;
using bosdyn::api::ImageService;

class ImageClient {
 public:
  ImageClient(std::shared_ptr<Channel> channel)
      : stub_(ImageService::NewStub(channel)) {}


  // Assembles the client's payload, sends it and presents the response back
  // from the server.

  std::string ListImageSources() {
    //TODO
  }

  std::string GetImage(const std::string& image_source_name, double quality_percent) {
    // Data we are sending to the server.
    GetImageRequest request;
    ImageRequest* img = request->add_image_requests();
    request.set_image_source_name(image_source_name);
    request.set_quality_percent(quality_percent);
    //Specify JPEG ect.
    request.set_image_formate(Image.format::FORMAT_JPEG); 

    // Container for the data we expect from the server.
    GetImageResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->GetImage(&context, request, &reply);

    // Act upon its status.
    //TODO
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
  std::unique_ptr<ImageService::Stub> stub_;
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
  ImageClient imageClient(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  std::string user("testUser");
  std::string pass("testPassword");
  std::string appToken("testAppToken");
  std::string reply = imageClient.GetImage(user, pass, appToken);
  std::cout << "Token received: " << reply << std::endl;

  return 0;
}
