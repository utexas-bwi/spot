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
using bosdyn::api::ImageResponse;
using bosdyn::api::ImageCapture;
using bosdyn::api;
namespace bosdyn{
namespace api{


  Status ListImageSources(ServerContext* context, const ListImageSourcesRequest* request,
                  ListImageSourcesResponse* response) override {
    //create response to list all the possible image sources
    ImageSource* imgsrc = response->add_image_sources();
    imgsrc->set_name("imageName");
    imgsrc->set_rows(480);
    imgsrc->set_cols(640)
    imgsrc->set_depth_scale(1000);
    imgsrc->set_image_type(ImageSource::IMAGE_TYPE_VISUAL);
    
    return Status::OK;
  }

  Status GetImage(ServerContext* context, const GetImageRequest* request,
                  GetImageResponse* response) override {

      
      ImageResponse* img = response->add_image_responses();
      ImageCapture* shot;
      ImageSource* imgsrc;
      img->set_allocated_shot(shot);
      img->set_allocated_source(imgsrc);
      img->set_status(ImageRequest::STATUS_OK);
      return Status::OK;
      
  }

}
}