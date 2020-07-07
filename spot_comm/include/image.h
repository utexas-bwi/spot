#ifndef IMAGE_H
#define IMAGE_H
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/image.grpc.pb.h"
#include "bosdyn/api/image_service.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using bosdyn::api::ListImageSourcesRequest;
using bosdyn::api::ListImageSourcesResponse;
using bosdyn::api::GetImageRequest;
using bosdyn::api::GetImageResponse;
using bosdyn::api::ImageService;
using bosdyn::api::ImageRequest;
namespace bosdyn {
namespace api {
class ImageServiceImpl final : public ImageService::Service {
public:
  Status ListImageSources(ServerContext* context, const ListImageSourcesRequest* request,
                  ListImageSourcesResponse* response);

  Status GetImage(ServerContext* context, const GetImageRequest* request,
                  GetImageResponse* response);
};
}//api
}//bosdyn

#endif